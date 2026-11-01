#!/usr/bin/env python3
"""Learn segmented DMP weights from a recorded trajectory."""
import argparse
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from openclaw_control.dmp_core import TRAJECTORY_DIR, WEIGHTS_DIR, learn_weights, load_recording, save_npy, positive_float


def main():
    parser = argparse.ArgumentParser(description="从轨迹学习 DMP，权重保存到 skill/weights")
    parser.add_argument("--input", required=True, type=Path, help="采集名称（如 pick），或 .npz/.npy 文件名、路径；名称从 openclaw_control/trajectory 读取")
    parser.add_argument("--n-bfs", type=int, default=100, help="高斯基函数数量，至少 2")
    parser.add_argument("--alpha", type=positive_float, default=25.0)
    args = parser.parse_args()
    try:
        stem = args.input.stem.removesuffix("_trajectory")
        name = f"{stem}_weights.npy"
        input_path = args.input
        if input_path.name == str(input_path) and not input_path.suffix:
            input_path = TRAJECTORY_DIR / f"{stem}_trajectory.npz"
        segments, frequency = load_recording(input_path)
        data = learn_weights(segments, frequency, args.n_bfs, args.alpha)
        output = save_npy(WEIGHTS_DIR / name, data)
    except (ValueError, OSError, KeyError) as exc:
        parser.exit(1, f"错误: {exc}\n")
    print(f"已学习 {len(segments)} 段轨迹；权重保存到 {output}")


if __name__ == "__main__":
    main()
