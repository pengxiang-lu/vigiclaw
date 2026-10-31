"""Small ManiSkill adapter shared by collection and local replay."""
import os
from pathlib import Path
import time

import numpy as np
from openclaw_control.dmp_core import control_frequency

PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONTROL_FREQ = 50
PANDA_BASE_X_OFFSET = 0.615


def tcp_action_from_observation(obs, gripper=1.0):
    """Actual world TCP pose, with XYZ Euler convention used by the controller.

    The seventh value is the commanded gripper state, not finger joint positions.
    """
    raw = obs["extra"]["tcp_pose"]
    if hasattr(raw, "detach"):
        raw = raw.detach().cpu().numpy()
    raw = np.asarray(raw, dtype=float).reshape(-1, 7)[0]
    w, x, y, z = raw[3:] / np.linalg.norm(raw[3:])
    roll = np.arctan2(2 * (w*x - y*z), 1 - 2 * (x*x + y*y))
    pitch = np.arcsin(np.clip(2 * (x*z + w*y), -1, 1))
    yaw = np.arctan2(2 * (w*z - x*y), 1 - 2 * (y*y + z*z))
    return np.array([*raw[:3], roll, pitch, yaw, gripper])


class Simulation:
    def __init__(self, task="OpenclawPickCube", seed=0, headless=False, control_freq=50):
        os.environ.setdefault("MS_ASSET_DIR", str(PROJECT_ROOT / "assets"))
        import mani_skill.envs  # noqa: F401
        import openclaw_tasks  # noqa: F401
        import gymnasium as gym

        self.control_freq = control_frequency(control_freq)
        self.headless = headless
        self.env = gym.make(
            task, num_envs=1, robot_uids="panda", obs_mode="state_dict",
            control_mode="pd_ee_pose", render_mode=None if headless else "human",
            sim_config={"control_freq": self.control_freq}, max_episode_steps=None,
        )
        try:
            self.obs, _ = self.env.reset(seed=seed)
        except BaseException:
            self.env.close()
            raise

    def current_action(self, gripper=1.0):
        return tcp_action_from_observation(self.obs, gripper)

    def step(self, world_action):
        start = time.monotonic()
        action = np.array(world_action, dtype=np.float32, copy=True)
        # pd_ee_pose accepts robot-root coordinates; recordings/API use world coordinates.
        action[0] += PANDA_BASE_X_OFFSET
        self.obs, _, _, _, _ = self.env.step(action)
        if not self.headless:
            self.env.render()
            time.sleep(max(0, 1 / self.control_freq - (time.monotonic() - start)))

    def hold(self, action, seconds):
        for _ in range(max(1, round(seconds * self.control_freq))):
            self.step(action)

    def close(self):
        self.env.close()
