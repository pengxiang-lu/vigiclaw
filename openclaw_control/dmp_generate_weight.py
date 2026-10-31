#!/usr/bin/env python3
"""Learn segmented DMP weights from a recorded trajectory."""
import argparse
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from openclaw_control.dmp_core import WEIGHTS_DIR, learn_weights, load_recording, save_npy, positive_float


def main():
    parser = argparse.ArgumentParser(description="从轨迹学习 DMP，权重保存到 skill/weights")
    parser.add_argument("--input", required=True, type=Path, help="采集的 .npz 轨迹（兼容旧 .npy）")
    parser.add_argument("--output", help="权重文件名，默认 <名称>_weights.npy（去掉 _trajectory）；保存在 skill/weights")
    parser.add_argument("--n-bfs", type=int, default=100, help="高斯基函数数量，至少 2")
    parser.add_argument("--alpha", type=positive_float, default=25.0)
    args = parser.parse_args()
    try:
        stem = args.input.stem.removesuffix("_trajectory")
        name = args.output or f"{stem}_weights.npy"
        if Path(name).name != name:
            raise ValueError("--output 只接受文件名；权重统一保存到 skill/weights")
        segments, frequency = load_recording(args.input)
        data = learn_weights(segments, frequency, args.n_bfs, args.alpha)
        output = save_npy(WEIGHTS_DIR / name, data)
    except (ValueError, OSError, KeyError) as exc:
        parser.exit(1, f"错误: {exc}\n")
    print(f"已学习 {len(segments)} 段轨迹；权重保存到 {output}")


if __name__ == "__main__":
    main()
