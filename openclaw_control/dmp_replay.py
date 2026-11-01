#!/usr/bin/env python3
"""Generate a DMP trajectory and reproduce it in a standalone simulation."""
import argparse
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from openclaw_control.dmp_core import (
    add_generation_arguments, generate_trajectory, load_weights,
    midpoint_arguments, positive_float, resolve_weights, save_npy,
)
from openclaw_control.dmp_plot import original_trajectory, plot_comparison
from openclaw_control.dmp_simulation import Simulation


def main():
    parser = argparse.ArgumentParser(description="DMP 仿真回放，可调整分段中间点及终点")
    add_generation_arguments(parser)
    parser.add_argument("--task", default="OpenclawPickCube")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--headless", action="store_true", help="无窗口运行")
    parser.add_argument("--gripper-hold", type=positive_float, default=0.5,
                        help="开合夹爪后的保持时间（秒）")
    parser.add_argument("--generate-only", action="store_true", help="仅生成轨迹，不启动仿真")
    parser.add_argument("--trajectory", type=Path, help="可选：openclaw_control/trajectory 下的原始采集文件名或 .npz/.npy 路径")
    parser.add_argument("--plot-output", type=Path, help="可选：额外保存对比图的 .png 路径；默认只显示、不保存")
    parser.add_argument("--show-plot", action="store_true", help="显示对比图窗口（默认行为），关闭所有图窗后继续仿真")
    parser.add_argument("--no-plot", action="store_true", help="跳过原始轨迹和生成轨迹对比图")
    args = parser.parse_args()
    try:
        data = load_weights(args.weights)
        trajectory = generate_trajectory(data, args.start, args.goal,
                                         midpoint_arguments(args.midpoint), args.time_scale)
        if args.save_trajectory:
            print(f"轨迹已保存: {save_npy(args.save_trajectory, trajectory)}")
        if not args.no_plot:
            weights_path = resolve_weights(args.weights)
            original, lengths = original_trajectory(data, weights_path, args.trajectory)
            print("请查看轨迹对比图，关闭所有图窗后继续。")
            for path in plot_comparison(original, trajectory, args.plot_output, lengths, show=True):
                print(f"对比图已保存: {path}")
    except (OSError, ValueError, KeyError) as exc:
        parser.exit(1, f"错误: {exc}\n")
    print(f"已生成 {len(trajectory)} 个轨迹点，{len(data['segment_weights'])} 段")
    if args.generate_only:
        return
    sim = Simulation(args.task, args.seed, args.headless, data["control_freq"])
    try:
        # Reach the demonstrated start before playing the timed trajectory.
        sim.hold(trajectory[0], 1.0)
        previous_gripper = trajectory[0, 6]
        for action in trajectory:
            sim.step(action)
            if action[6] != previous_gripper:
                sim.hold(action, args.gripper_hold)
            previous_gripper = action[6]
        sim.hold(trajectory[-1], 1.0)
    except KeyboardInterrupt:
        print("回放已中断")
    finally:
        sim.close()


if __name__ == "__main__":
    main()
