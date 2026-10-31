"""Run with: python -m unittest discover -s tests -v (no simulator required)."""
import tempfile
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

import numpy as np

from openclaw_control.dmp_collect_trajectory import TrajectoryRecorder, recording_output
from openclaw_control.dmp_core import (
    DMP, generate_trajectory, learn_weights, load_recording, load_weights,
    midpoint_arguments, save_npy,
)
from openclaw_control.dmp_simulation import tcp_action_from_observation
from skill.scripts.dmp_action import RobotClient


def demo(gripper=1, x=0):
    t = np.linspace(0, 1, 101)
    return np.column_stack((x + .1*t, .02*np.sin(np.pi*t), .3 + .05*t,
                            np.full_like(t, -3.14), np.zeros((len(t), 2)),
                            np.full_like(t, gripper)))


class DMPWorkflowTests(unittest.TestCase):
    def setUp(self):
        self.segments = [demo(), demo(-1, .1), demo(1, .2)]
        self.weights = learn_weights(self.segments)

    def test_reproduction_accuracy_and_gripper(self):
        model = DMP().learn(self.segments[0])
        result = model.generate(101, model.start, model.goal)
        self.assertLess(np.max(np.abs(result - self.segments[0])), .004)
        np.testing.assert_allclose(result[[0, -1]], self.segments[0][[0, -1]])
        np.testing.assert_array_equal(result[:, 6], 1)

    def test_zero_displacement_retains_loop(self):
        original = demo()
        original[:, 0] = .1*np.sin(np.linspace(0, np.pi, len(original)))
        model = DMP().learn(original)
        result = model.generate(len(original), model.start, model.goal)
        self.assertGreater(result[:, 0].max(), .09)
        self.assertLess(np.max(np.abs(result-original)), .01)

    def test_midpoint_goal_and_live_start(self):
        start = [-.2, .03, .4, -3.14, 0, 0, -1]
        result = generate_trajectory(self.weights, start, [.4, -.1, .5], {0: [.05, .08, .45]})
        np.testing.assert_allclose(result[0], start)
        # First live sample, then the first 101-point demonstration segment.
        np.testing.assert_allclose(result[101, :3], [.05, .08, .45])
        np.testing.assert_allclose(result[102, :3], result[101, :3])
        self.assertEqual(result[101, 6], 1)
        self.assertEqual(result[102, 6], -1)
        np.testing.assert_allclose(result[-1, :3], [.4, -.1, .5])
        self.assertEqual(result[-1, 6], 1)

    def test_short_constant_segment_and_time_scaling(self):
        weights = learn_weights([self.segments[0][:1]])
        result = generate_trajectory(weights, goal=[.2, .1, .4], time_scale=.01)
        self.assertTrue(np.isfinite(result).all())
        np.testing.assert_allclose(result[-1, :3], [.2, .1, .4])
        slow = generate_trajectory(self.weights, time_scale=2)
        self.assertEqual(len(slow), 603)

    def test_invalid_overrides(self):
        for midpoint in ({-1: [0, 0, 0]}, {2: [0, 0, 0]}):
            with self.assertRaises(ValueError):
                generate_trajectory(self.weights, midpoints=midpoint)
        for point in ([0, 1], [float("nan"), 0, 0]):
            with self.assertRaises(ValueError):
                generate_trajectory(self.weights, goal=point)
        for values in ([[.5, 0, 0, 0]], [[0, 0, 0, 0], [0, 1, 1, 1]]):
            with self.assertRaises(ValueError):
                midpoint_arguments(values)

    def test_recorder_save_and_weight_roundtrip(self):
        recorder = TrajectoryRecorder()
        recorder.start()
        for s in self.segments:
            for p in s:
                recorder.append(p)
        with tempfile.TemporaryDirectory() as directory:
            path = recorder.save(Path(directory)/"record.npz", "OpenclawPickCube")
            with np.load(path, allow_pickle=False) as archive:
                np.testing.assert_array_equal(archive["segment_n_points"], [101, 101, 101])
            segments, frequency = load_recording(path)
            self.assertIsInstance(frequency, int)
            self.assertEqual(len(segments), 3)
            self.assertEqual(sum(map(len, segments)), 303)
            output = save_npy(Path(directory)/"weights.npy", learn_weights(segments, frequency))
            loaded = load_weights(output)
            self.assertIsInstance(loaded["control_freq"], int)
            result = generate_trajectory(loaded)
            np.testing.assert_allclose(result, generate_trajectory(self.weights))

    def test_recording_name_and_manual_boundaries(self):
        import argparse
        self.assertEqual(recording_output("pick_obj").name, "pick_obj_trajectory.npz")
        self.assertEqual(recording_output("pick_obj.npz"), recording_output("pick_obj"))
        self.assertEqual(recording_output("pick_obj_trajectory"), recording_output("pick_obj"))
        self.assertEqual(recording_output("pick_obj_trajectory.npz"), recording_output("pick_obj"))
        self.assertEqual(recording_output("pick_obj").parent.name, "weights")
        for name in ("", " ", ".", "..", "../pick", "/tmp/pick", "dir\\pick", "pick.npy"):
            with self.assertRaises(argparse.ArgumentTypeError):
                recording_output(name)
        recorder = TrajectoryRecorder()
        recorder.start()
        for point in demo()[:3]:
            recorder.append(point)
        recorder.split()
        recorder.append(demo()[3])
        with tempfile.TemporaryDirectory() as directory:
            path = recorder.save(Path(directory)/"manual.npz", "OpenclawPickCube")
            segments, _ = load_recording(path)
            self.assertEqual([len(s) for s in segments], [3, 1])

    def test_invalid_npz_segment_lengths(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"invalid.npz"
            np.savez(path, trajectory=demo(), segment_n_points=[100])
            with self.assertRaises(ValueError):
                load_recording(path)

    def test_old_recordings_and_weights(self):
        with tempfile.TemporaryDirectory() as directory:
            raw = save_npy(Path(directory)/"raw.npy", np.vstack(self.segments))
            segments, _ = load_recording(raw)
            self.assertEqual(len(segments), 3)
            old = save_npy(Path(directory)/"old.npy", {"segments": self.segments, "n_bfs": 50})
            result = generate_trajectory(load_weights(old))
            self.assertTrue(np.isfinite(result).all())

    def test_tcp_pose_is_measured_and_uses_xyz_euler(self):
        # q = qx(roll) * qy(pitch) * qz(yaw), matching euler_angles_to_matrix(..., 'XYZ').
        roll, pitch, yaw = -.6, .3, .8
        cr, cp, cy = np.cos(np.array([roll, pitch, yaw])/2)
        sr, sp, sy = np.sin(np.array([roll, pitch, yaw])/2)
        q = [cr*cp*cy-sr*sp*sy, sr*cp*cy+cr*sp*sy,
             cr*sp*cy-sr*cp*sy, cr*cp*sy+sr*sp*cy]
        obs = {"extra": {"tcp_pose": np.array([[.1, -.2, .3, *q]])}}
        actual = tcp_action_from_observation(obs, -1)
        np.testing.assert_allclose(actual, [.1, -.2, .3, roll, pitch, yaw, -1])


class HTTPExecutionTests(unittest.TestCase):
    def test_live_start_uses_measured_tcp(self):
        client = RobotClient()
        client.request = Mock(return_value={"current_action": [9]*7, "tcp_action": demo()[0].tolist()})
        np.testing.assert_allclose(client.current_pose(), demo()[0])

    def test_old_server_is_reported(self):
        client = RobotClient()
        client.request = Mock(return_value={"current_action": [9]*7})
        with self.assertRaisesRegex(RuntimeError, "重启"):
            client.current_pose()

    def test_server_rejection_stops_sending(self):
        session = Mock()
        session.request.return_value.json.return_value = {"status": "success", "success": False}
        client = RobotClient(session=session)
        with self.assertRaises(RuntimeError):
            client.execute(demo()[:3], 50)
        self.assertEqual(session.request.call_count, 1)

    @patch("skill.scripts.dmp_action.time.sleep")
    def test_gripper_transition_is_held_and_waited(self, sleep):
        client = RobotClient()
        client.request = Mock(return_value={"action_arrived": True})
        trajectory = demo()[:3].copy()
        trajectory[1:, 6] = -1
        client.execute(trajectory, 50, gripper_hold=.5)
        calls = client.request.call_args_list
        self.assertEqual(sum(c.args[0] == "POST" for c in calls), 3)
        self.assertEqual(sum(c.args[0] == "GET" for c in calls), 2)
        sleep.assert_any_call(.5)

    @patch("skill.scripts.dmp_action.time.sleep")
    def test_arrival_timeout(self, sleep):
        client = RobotClient()
        client.request = Mock(return_value={"action_arrived": False})
        with self.assertRaises(TimeoutError):
            client.wait_for_arrival(.001)


if __name__ == "__main__":
    unittest.main()
