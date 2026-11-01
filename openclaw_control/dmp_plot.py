"""Compare the recorded demonstration against the generated DMP commands."""
from pathlib import Path

import numpy as np

from openclaw_control.dmp_core import TRAJECTORY_DIR, load_recording, trajectory_array


def original_trajectory(data, weights_path, recording=None):
    """Use an explicit recording, embedded demonstration, or companion recording."""
    if recording is not None:
        segments, _ = load_recording(recording)
        return np.vstack(segments), [len(s) for s in segments]
    if "trajectory" in data:
        return trajectory_array(data["trajectory"]), data["segment_n_points"]
    weights_path = Path(weights_path)
    stem = weights_path.stem.removesuffix("_weights")
    # Prefer the dedicated recording directory; keep old companion files readable.
    for candidate in (TRAJECTORY_DIR / f"{stem}_trajectory.npz",
                      weights_path.with_name(f"{stem}_trajectory.npz")):
        if candidate.is_file():
            segments, _ = load_recording(candidate)
            return np.vstack(segments), [len(s) for s in segments]
    raise FileNotFoundError(
        "权重中没有原始轨迹，也未找到同名 _trajectory.npz；"
        "请传 --trajectory <采集文件>、重新生成权重，或用 --no-plot 跳过绘图")


def plot_comparison(original, generated, output=None, segment_lengths=(), show=False):
    """Show comparisons and optionally save them when an output is supplied."""
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    original, generated = trajectory_array(original), trajectory_array(generated)
    output_3d = None
    if output is not None:
        output = Path(output)
        if output.suffix.lower() != ".png":
            raise ValueError("--plot-output 必须以 .png 结尾")
        output.parent.mkdir(parents=True, exist_ok=True)
        output_3d = output.with_name(f"{output.stem}_3d.png")
    colors = ("#475569", "#ea580c")
    labels = ("Original demonstration", "Generated DMP")

    def draw_3d(ax):
        for values, color, label in zip((original, generated), colors, labels):
            ax.plot(*values[:, :3].T, color=color, label=label, linewidth=1.8)
            ax.scatter(*values[0, :3], color=color, marker="o", s=35)
            ax.scatter(*values[-1, :3], color=color, marker="*", s=90)
        boundaries = np.cumsum(segment_lengths)[:-1]
        for index in boundaries:
            if 0 <= index < len(original):
                ax.scatter(*original[index, :3], color=colors[0], marker="x", s=45)
        ax.set(xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)", title="TCP paths (world frame)")
        span = np.ptp(np.vstack((original[:, :3], generated[:, :3])), axis=0)
        ax.set_box_aspect(np.maximum(span, max(span.max(), .01) * .15))
        ax.legend(fontsize=8)

    figures = []
    try:
        fig = plt.figure(figsize=(20, 9), constrained_layout=True)
        figures.append(fig)
        fig.suptitle("Recorded trajectory vs generated DMP commands", fontsize=16)
        draw_3d(fig.add_subplot(2, 4, 1, projection="3d"))
        dimensions = ("X (m)", "Y (m)", "Z (m)", "Roll (rad)",
                      "Pitch (rad)", "Yaw (rad)", "Gripper command")
        for dim, title in enumerate(dimensions):
            ax = fig.add_subplot(2, 4, dim + 2)
            for values, color, label in zip((original, generated), colors, labels):
                y = values[:, dim]
                if dim in (3, 4, 5):
                    # Align equivalent angles for a readable comparison around ±pi.
                    y = np.unwrap(y)
                    y -= 2*np.pi * np.round((y[0] - original[0, dim]) / (2*np.pi))
                if dim == 6:
                    ax.step(np.arange(len(y)), y, where="post", color=color, label=label)
                else:
                    ax.plot(y, color=color, label=label, linewidth=1.5)
            for index in np.cumsum(segment_lengths)[:-1]:
                ax.axvline(index, color=colors[0], alpha=.2, linestyle=":")
            ax.set(title=title, xlabel="Sample index", ylabel=title)
            ax.grid(alpha=.2)
            ax.legend(fontsize=8)
        if output is not None:
            fig.savefig(output, dpi=150)
        if output_3d is not None:
            # Save the optional 3D export without creating another plot window.
            from matplotlib.figure import Figure
            fig_3d = Figure(figsize=(10, 8), constrained_layout=True)
            draw_3d(fig_3d.add_subplot(111, projection="3d"))
            fig_3d.savefig(output_3d, dpi=150)
        if show:
            plt.show(block=True)
    finally:
        for figure in figures:
            plt.close(figure)
    return (output, output_3d) if output is not None else ()
