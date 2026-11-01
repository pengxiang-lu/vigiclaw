#!/usr/bin/env python3
"""Generate and send a DMP trajectory through the existing HTTP action API."""
import argparse
from pathlib import Path
import sys
import time

import numpy as np
import requests

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from openclaw_control.dmp_core import (
    add_generation_arguments, generate_trajectory, load_weights,
    midpoint_arguments, positive_float, pose, save_npy,
)


class RobotClient:
    def __init__(self, server_url="http://127.0.0.1:8000", session=None):
        self.url = server_url.rstrip("/")
        self.session = session if session is not None else requests.Session()

    def request(self, method, endpoint, **kwargs):
        response = self.session.request(method, self.url + endpoint, timeout=30, **kwargs)
        response.raise_for_status()
        data = response.json()
        if data.get("success") is False or data.get("status") in ("error", "failed"):
            raise RuntimeError(f"服务端拒绝动作: {data}")
        return data

    def current_pose(self):
        status = self.request("GET", "/arrival_status")
        if "tcp_action" not in status:
            raise RuntimeError("服务端未返回实际 TCP 位姿 tcp_action，请重启更新后的 server.py")
        value = np.asarray(status["tcp_action"], dtype=float)
        if value.shape != (7,) or not np.isfinite(value).all():
            raise ValueError("服务端 tcp_action 必须包含 7 个有限数值")
        return value

    def wait_for_arrival(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status = self.request("GET", "/arrival_status")
            if status.get("action_arrived"):
                return
            time.sleep(0.02)
        raise TimeoutError(f"等待夹爪到达目标超时（{timeout:g} 秒）")

    def execute(self, trajectory, frequency, gripper_hold=0.5, arrival_timeout=10):
        previous_gripper = trajectory[0, 6]
        for index, action in enumerate(trajectory):
            start = time.monotonic()
            self.request("POST", "/action", json={"action": action.tolist()})
            # A boundary grip change must finish before continuing to the next motion.
            if action[6] != previous_gripper:
                time.sleep(gripper_hold)
                self.wait_for_arrival(arrival_timeout)
            previous_gripper = action[6]
            time.sleep(max(0, 1 / frequency - (time.monotonic() - start)))
            if (index + 1) % 50 == 0:
                print(f"已发送 {index+1}/{len(trajectory)} 点")
        self.wait_for_arrival(arrival_timeout)


def main():
    parser = argparse.ArgumentParser(description="发送 DMP 轨迹；默认从服务端实际 TCP 位姿开始")
    add_generation_arguments(parser)
    parser.add_argument("--server-url", default="http://127.0.0.1:8000")
    parser.add_argument("--gripper-hold", type=positive_float, default=0.5)
    parser.add_argument("--arrival-timeout", type=positive_float, default=10)
    parser.add_argument("--dry-run", action="store_true", help="只生成、不发送；显式 --start 时无需服务端")
    args = parser.parse_args()
    client = RobotClient(args.server_url)
    try:
        data = load_weights(args.weights)
        midpoints = midpoint_arguments(args.midpoint)
        # Validate all overrides before contacting the server.
        generate_trajectory(data, args.start, args.goal, midpoints, args.time_scale)
        if args.start is None:
            start = client.current_pose()
        elif args.dry_run:
            start = args.start
        else:
            # XYZ-only overrides retain the current orientation and gripper state.
            start = pose(args.start, client.current_pose())
        trajectory = generate_trajectory(data, start, args.goal, midpoints, args.time_scale)
        if args.save_trajectory:
            print(f"轨迹已保存: {save_npy(args.save_trajectory, trajectory)}")
        print(f"已生成 {len(trajectory)} 点；起点: {trajectory[0].tolist()}")
        if not args.dry_run:
            # An explicitly chosen start may differ from the actual current pose.
            if args.start is not None:
                client.request("POST", "/action", json={"action": trajectory[0].tolist()})
                client.wait_for_arrival(args.arrival_timeout)
            client.execute(trajectory, data["control_freq"], args.gripper_hold, args.arrival_timeout)
            print("DMP 轨迹发送完成，末端已到达终点")
    except KeyboardInterrupt:
        parser.exit(130, "已中断，停止发送后续轨迹点\n")
    except (OSError, ValueError, KeyError, RuntimeError, requests.RequestException) as exc:
        parser.exit(1, f"错误: {exc}\n")
    finally:
        client.session.close()


if __name__ == "__main__":
    main()
