"""Shared segmented DMP learning/generation. All poses use world coordinates.

The spring/damper, canonical phase and Gaussian forcing terms follow the original DMP implementation.
Gripper commands remain discrete; each segment has one gripper state.
"""
from pathlib import Path
import argparse

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
WEIGHTS_DIR = PROJECT_ROOT / "skill-dmp" / "weights"
TRAJECTORY_DIR = PROJECT_ROOT / "openclaw_control" / "trajectory"


def positive_float(value):
    value = float(value)
    if not np.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("must be finite and greater than zero")
    return value


def control_frequency(value):
    value = float(value)
    if not np.isfinite(value) or value <= 0 or value != int(value):
        raise ValueError("control_freq 必须是正整数")
    return int(value)


def pose(value, reference):
    """Accept XYZ, XYZ+Euler, or a full seven-dimensional action."""
    result = np.array(reference, dtype=float, copy=True)
    value = np.asarray(value, dtype=float)
    if value.shape not in ((3,), (6,), (7,)) or not np.isfinite(value).all():
        raise ValueError("位姿必须是 3、6 或 7 个有限数值")
    result[:len(value)] = value
    return result


def trajectory_array(value):
    value = np.asarray(value, dtype=float)
    if value.ndim != 2 or value.shape[1] != 7 or len(value) == 0:
        raise ValueError("轨迹必须是非空的 (N, 7) 数组")
    if not np.isfinite(value).all():
        raise ValueError("轨迹包含 NaN 或无穷大")
    return value


def split_gripper(trajectory):
    trajectory = trajectory_array(trajectory)
    boundaries = np.flatnonzero(np.abs(np.diff(trajectory[:, 6])) > 0.5) + 1
    return list(np.split(trajectory, boundaries))


def load_recording(path):
    """Read .npz recordings, legacy .npy dictionaries or plain (N, 7) arrays."""
    path = Path(path)
    if path.name == str(path) and (TRAJECTORY_DIR / path).is_file():
        path = TRAJECTORY_DIR / path
    if path.suffix == ".npz":
        with np.load(path, allow_pickle=False) as archive:
            data = {key: archive[key] for key in archive.files}
        trajectory = trajectory_array(data["trajectory"])
        lengths = np.asarray(data["segment_n_points"])
        if (lengths.ndim != 1 or len(lengths) == 0
                or not np.issubdtype(lengths.dtype, np.integer)
                or np.any(lengths <= 0) or lengths.sum() != len(trajectory)):
            raise ValueError("采集文件的 segment_n_points 与轨迹长度不匹配")
        data["segments"] = np.split(trajectory, np.cumsum(lengths)[:-1])
    else:
        raw = np.load(path, allow_pickle=True)
        data = raw.item() if raw.shape == () else {"trajectory": raw}
    if not isinstance(data, dict):
        raise ValueError("轨迹文件必须是字典或 (N, 7) 数组")
    segments = data.get("segments")
    if segments is None or len(segments) == 0:
        segments = [data["trajectory"]]
    # Preserve manual boundaries, and split any mixed-gripper segments as well.
    segments = [part for segment in segments for part in split_gripper(segment)]
    frequency = control_frequency(data.get("control_freq", 50))
    return segments, frequency


class DMP:
    """Discrete DMP for the six continuous TCP dimensions, in normalized time."""

    def __init__(self, n_bfs=100, alpha=25.0, beta=None):
        if n_bfs < 2:
            raise ValueError("n_bfs 必须至少为 2")
        self.n_bfs = int(n_bfs)
        self.alpha = positive_float(alpha)
        self.beta = positive_float(beta if beta is not None else alpha / 4)
        self.centers = np.exp(-np.linspace(0, 1, self.n_bfs))
        self.widths = np.full(self.n_bfs, 1 / np.mean(np.diff(self.centers)) ** 2)

    def basis(self, t):
        phase = np.exp(-np.atleast_1d(t))
        psi = np.exp(-self.widths * (phase[:, None] - self.centers) ** 2)
        return psi / np.maximum(psi.sum(axis=1, keepdims=True), 1e-12) * phase[:, None]

    def learn(self, trajectory):
        trajectory = trajectory_array(trajectory)
        self.start, self.goal = trajectory[0].copy(), trajectory[-1].copy()
        self.n_steps_learned = len(trajectory)
        # Unwrap Euler angles so crossing ±pi does not create a full revolution.
        y = trajectory[:, :6].copy()
        y[:, 3:6] = np.unwrap(y[:, 3:6], axis=0)
        self.start[:6], self.goal[:6] = y[0], y[-1]
        self.scale = self.goal[:6] - self.start[:6]
        # Zero net displacement can still contain motion, such as lifting then lowering.
        self.scale[np.abs(self.scale) < 1e-4] = 1.0
        if len(y) == 1:
            self.weights = np.zeros((self.n_bfs, 6))
            return self
        t = np.linspace(0, 1, len(y))
        edge_order = 2 if len(y) >= 3 else 1
        velocity = np.gradient(y, t, axis=0, edge_order=edge_order)
        acceleration = np.gradient(velocity, t, axis=0, edge_order=edge_order)
        target = (acceleration - self.alpha * (
            self.beta * (self.goal[:6] - y) - velocity)) / self.scale
        basis = self.basis(t)
        self.weights = np.linalg.solve(
            basis.T @ basis + 1e-6 * np.eye(self.n_bfs), basis.T @ target)
        return self

    def generate(self, n_steps, start, goal):
        n_steps = max(2, int(n_steps))
        start, goal = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
        scale = goal[:6] - start[:6]
        learned_delta = self.goal[:6] - self.start[:6]
        scale[np.abs(learned_delta) < 1e-4] = self.scale[np.abs(learned_delta) < 1e-4]
        # Integrate at a fixed fine resolution, even for very short segments.
        internal_steps = max(501, n_steps)
        t = np.linspace(0, 1, internal_steps)
        forces = self.basis(t) @ self.weights * scale
        y, velocity = start[:6].copy(), np.zeros(6)
        generated = np.empty((internal_steps, 6))
        generated[0] = y
        dt = t[1] - t[0]
        for i in range(1, internal_steps):
            acceleration = self.alpha * (self.beta * (goal[:6] - y) - velocity) + forces[i-1]
            velocity += acceleration * dt
            y += velocity * dt
            generated[i] = y
        # Remove terminal integration error smoothly instead of snapping the last point.
        blend = (3 * t**2 - 2 * t**3)[:, None]
        generated += blend * (goal[:6] - generated[-1])
        result = np.empty((n_steps, 7))
        for d in range(6):
            result[:, d] = np.interp(np.linspace(0, 1, n_steps), t, generated[:, d])
        result[:, 6] = start[6]
        result[-1, :6] = goal[:6]
        return result

    def to_dict(self):
        return {name: getattr(self, name) for name in (
            "weights", "start", "goal", "scale", "n_steps_learned", "centers", "widths")}


def learn_weights(segments, control_freq=50, n_bfs=100, alpha=25):
    models = [DMP(n_bfs, alpha).learn(segment) for segment in segments]
    if not models:
        raise ValueError("没有可学习的轨迹段")
    return {
        "format_version": 2, "n_bfs": n_bfs, "alpha": alpha, "beta": alpha / 4,
        "tau": 1.0, "control_freq": control_frequency(control_freq),
        "segment_n_points": [len(s) for s in segments],
        "trajectory": np.vstack(segments),
        "segment_weights": [model.to_dict() for model in models],
    }


def resolve_weights(value):
    path = Path(value).expanduser()
    if path.is_file():
        return path.resolve()
    if path.name == str(path):
        if path.suffix:
            names = [path.name]
        elif path.name.endswith("_weights"):
            names = [f"{path.name}.npy"]
        else:
            names = [f"{path.name}_weights.npy", f"{path.name}.npy"]
        for name in names:
            candidate = WEIGHTS_DIR / name
            if candidate.is_file():
                return candidate
    raise FileNotFoundError(f"找不到权重文件: {value} (默认目录: {WEIGHTS_DIR})")


def load_weights(path):
    data = np.load(resolve_weights(path), allow_pickle=True).item()
    if data.get("format_version") != 2:
        # Existing files include demonstrations. Refit them once in memory using
        # the corrected zero-displacement implementation, without changing the file.
        segments = data.get("segments")
        if segments is None or len(segments) == 0:
            raise ValueError("旧权重缺少 segments，请用 dmp_generate_weight.py 重新生成")
        data = learn_weights(
            [s for segment in segments for s in split_gripper(segment)],
            data.get("control_freq", 50), data.get("n_bfs", 100), data.get("alpha", 25))
    if not data.get("segment_weights"):
        raise ValueError("权重文件没有轨迹段")
    data["control_freq"] = control_frequency(data.get("control_freq", 50))
    return data


def generate_trajectory(data, start=None, goal=None, midpoints=None, time_scale=1.0):
    """midpoints maps zero-based segment-end indices to XYZ/6D/7D poses."""
    time_scale = positive_float(time_scale)
    segments = data["segment_weights"]
    midpoints = midpoints or {}
    if any(index < 0 or index >= len(segments) - 1 for index in midpoints):
        raise ValueError(f"中间点索引范围为 0..{len(segments)-2}；最后一段请用 --goal")
    result = []
    for i, saved in enumerate(segments):
        model = DMP(data["n_bfs"], data["alpha"], data["beta"])
        for name in ("weights", "start", "goal", "scale", "centers", "widths"):
            setattr(model, name, np.asarray(saved[name], dtype=float))
        seg_start = model.start.copy()
        if i == 0 and start is not None:
            seg_start = pose(start, seg_start)
            # A live start preserves current gripper for the first sample; the
            # demonstrated state is applied at the same pose before moving.
            if seg_start[6] != model.start[6]:
                result.append(seg_start[None].copy())
        elif i > 0:
            seg_start[:6] = result[-1][-1, :6]
        seg_start[6] = model.start[6]
        seg_goal = model.goal.copy()
        if i < len(segments) - 1:
            seg_goal[:6] = segments[i+1]["start"][:6]
            if i in midpoints:
                seg_goal = pose(midpoints[i], seg_goal)
        elif goal is not None:
            seg_goal = pose(goal, seg_goal)
        # Equivalent Euler representations should not cause unnecessary turns.
        seg_goal[3:6] = seg_start[3:6] + (
            seg_goal[3:6] - seg_start[3:6] + np.pi) % (2 * np.pi) - np.pi
        count = max(2, round((saved["n_steps_learned"] - 1) * time_scale) + 1)
        generated = model.generate(count, seg_start, seg_goal)
        # Keep boundary samples: identical positions can have different gripper states.
        result.append(generated)
        if seg_goal[6] != generated[-1, 6]:
            final = generated[-1].copy()
            final[6] = seg_goal[6]
            result.append(final[None])
    return trajectory_array(np.vstack(result))


def generated_trajectory_output(name):
    """Keep generated trajectory files in the project trajectory directory."""
    if not name.strip() or name in (".", "..") or "/" in name or "\\" in name:
        raise argparse.ArgumentTypeError("--save-trajectory 只接受文件名，不能包含目录路径")
    path = Path(name)
    if path.suffix not in ("", ".npy"):
        raise argparse.ArgumentTypeError("--save-trajectory 请传名称或 .npy 文件名")
    return TRAJECTORY_DIR / (name if path.suffix else f"{name}.npy")


def add_generation_arguments(parser):
    parser.add_argument("--weights", required=True, help="权重名称（如 pick 自动读取 skill-dmp/weights/pick_weights.npy）、文件名或路径")
    parser.add_argument("--start", type=float, nargs="+", help="起点: XYZ / XYZ+RPY / 七维动作")
    parser.add_argument("--goal", type=float, nargs="+", help="终点: XYZ / XYZ+RPY / 七维动作")
    parser.add_argument("--midpoint", action="append", nargs="+", type=float, default=[],
                        metavar="VALUE", help="索引 X Y Z [ROLL PITCH YAW [GRIPPER]]；可重复，索引从 0 开始")
    parser.add_argument("--time-scale", type=positive_float, default=1.0, help="时长倍数；2 表示慢一倍")
    parser.add_argument("--save-trajectory", type=generated_trajectory_output, metavar="NAME",
                        help="可选：生成轨迹文件名，统一保存到 openclaw_control/trajectory/<名称>.npy")


def midpoint_arguments(values):
    result = {}
    for item in values:
        if len(item) not in (4, 7, 8) or not np.isfinite(item).all() or item[0] != int(item[0]):
            raise ValueError("--midpoint 格式: 整数索引 X Y Z [ROLL PITCH YAW [GRIPPER]]")
        index = int(item[0])
        if index in result:
            raise ValueError(f"中间点索引重复: {index}")
        result[index] = item[1:]
    return result


def save_npy(path, data):
    path = Path(path)
    if path.suffix != ".npy":
        raise ValueError("输出文件扩展名必须为 .npy")
    path.parent.mkdir(parents=True, exist_ok=True)
    np.save(path, data)
    return path.resolve()
