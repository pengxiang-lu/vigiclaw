#!/usr/bin/env python3
"""Keyboard demonstration collection in a standalone ManiSkill simulation."""
import argparse
from pathlib import Path
import queue
import sys

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from openclaw_control.dmp_core import WEIGHTS_DIR
from openclaw_control.dmp_simulation import Simulation, CONTROL_FREQ

STEP_SIZE = 0.01
ROTATION_STEP = 0.05


def recording_output(name):
    """Resolve a recording name inside the shared weights directory."""
    if not name.strip() or name in (".", "..") or "/" in name or "\\" in name:
        raise argparse.ArgumentTypeError("--output 只接受名称，不能包含目录路径")
    path = Path(name)
    if path.suffix not in ("", ".npz"):
        raise argparse.ArgumentTypeError("--output 请传名称或以 .npz 结尾的文件名")
    stem = path.stem if path.suffix else name
    if not stem.endswith("_trajectory"):
        stem += "_trajectory"
    return WEIGHTS_DIR / f"{stem}.npz"


class TrajectoryRecorder:
    def __init__(self):
        self.recording = False
        self.segments = []
        self.current = []

    def start(self):
        self.segments, self.current = [], []
        self.recording = True

    def split(self):
        if self.current:
            self.segments.append(np.asarray(self.current))
            self.current = []

    def append(self, action):
        if not self.recording:
            return
        if self.current and abs(action[6] - self.current[-1][6]) > 0.5:
            self.split()
        self.current.append(np.array(action, dtype=float, copy=True))

    def save(self, path, task):
        self.split()
        if not self.segments:
            raise ValueError("尚未采集到轨迹，请先按 Z 开始录制")
        path = Path(path)
        if path.suffix != ".npz":
            raise ValueError("采集文件扩展名必须为 .npz")
        path.parent.mkdir(parents=True, exist_ok=True)
        # Store ordinary arrays so recordings can be loaded without pickle.
        np.savez_compressed(
            path, trajectory=np.vstack(self.segments),
            num_segments=len(self.segments),
            segment_n_points=np.array([len(s) for s in self.segments], dtype=np.int64),
            control_freq=CONTROL_FREQ, task=task, coordinate_frame="world",
        )
        self.recording = False
        return path


def main():
    parser = argparse.ArgumentParser(description="键盘控制 Panda，采集实际 TCP 轨迹和夹爪指令")
    parser.add_argument("--task", default="OpenclawPickCube")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output", required=True, type=recording_output, metavar="NAME",
                        help="采集名称，保存到 skill/weights/<名称>_trajectory.npz")
    args = parser.parse_args()
    # Import only after parsing, so --help works without a desktop/X display.
    from pynput import keyboard

    events = queue.Queue()

    def on_press(key):
        if key == keyboard.Key.esc:
            events.put("escape")
        elif key == keyboard.Key.space:
            events.put("space")
        elif getattr(key, "char", None):
            events.put(key.char.lower())

    print("移动: 2/8=X±, 6/4=Y±, 7/1=Z±；旋转: 9/3=Roll±, U/J=Pitch±, I/K=Yaw±")
    print("空格=开合夹爪；Z=开始/重新录制；X=手动分段；C=保存并结束采集；Esc=退出")
    print("按键作用于仿真目标；以 50 Hz 记录实际 TCP 位姿，夹爪变化自动分段。")
    sim = Simulation(args.task, args.seed)
    recorder = TrajectoryRecorder()
    action = sim.current_action()
    listener = keyboard.Listener(on_press=on_press)
    moves = {"2": (0, 1), "8": (0, -1), "6": (1, 1), "4": (1, -1),
             "7": (2, 1), "1": (2, -1), "9": (3, 1), "3": (3, -1),
             "u": (4, 1), "j": (4, -1), "i": (5, 1), "k": (5, -1)}
    try:
        listener.start()
        while True:
            while not events.empty():
                key = events.get_nowait()
                if key == "escape":
                    return
                if key == "z":
                    recorder.start()
                    print("开始录制")
                elif key == "x" and recorder.recording:
                    recorder.split()
                    print(f"手动分段，已完成 {len(recorder.segments)} 段")
                elif key == "c":
                    recorder.append(sim.current_action(action[6]))
                    if recorder.recording:
                        print(f"已保存: {recorder.save(args.output, args.task)}")
                        return
                    print("请先按 Z 开始录制")
                elif key == "space":
                    action[6] = -1.0 if action[6] > 0 else 1.0
                    # Keep the discrete transition sample even if two keys arrive in one tick.
                    recorder.append(sim.current_action(action[6]))
                elif key in moves:
                    dim, sign = moves[key]
                    action[dim] += sign * (STEP_SIZE if dim < 3 else ROTATION_STEP)
            sim.step(action)
            recorder.append(sim.current_action(action[6]))
    except KeyboardInterrupt:
        print("采集已停止；未保存的数据不会写入文件")
    finally:
        listener.stop()
        sim.close()


if __name__ == "__main__":
    main()
