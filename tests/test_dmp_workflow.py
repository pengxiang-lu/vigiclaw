"""Run with: python -m unittest discover -s tests -v (no simulator required)."""
import tempfile
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

import numpy as np

from openclaw_control.dmp_collect_trajectory import TrajectoryRecorder, recording_output
from openclaw_control.dmp_core import (
    DMP, generate_trajectory, learn_weights, load_recording, load_weights,
    midpoint_arguments, save_npy, add_generation_arguments, TRAJECTORY_DIR,
)
from openclaw_control.dmp_simulation import tcp_action_from_observation
from openclaw_control.dmp_plot import original_trajectory, plot_comparison
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
        self.assertEqual(recording_output("pick_obj").parent,
                         Path(__file__).resolve().parents[1] / "openclaw_control" / "trajectory")
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

    def test_generation_output_and_recording_lookup(self):
        import argparse
        parser = argparse.ArgumentParser()
        add_generation_arguments(parser)
        for name in ("generated", "generated.npy"):
            args = parser.parse_args(["--weights", "pick_weights", "--save-trajectory", name])
            self.assertEqual(args.save_trajectory, TRAJECTORY_DIR / "generated.npy")
        with tempfile.TemporaryDirectory() as directory:
            recorder = TrajectoryRecorder()
            recorder.start()
            recorder.append(demo()[0])
            path = recorder.save(Path(directory) / "lookup_trajectory.npz", "OpenclawPickCube")
            with patch("openclaw_control.dmp_core.TRAJECTORY_DIR", Path(directory)):
                segments, frequency = load_recording(path.name)
            np.testing.assert_array_equal(segments[0], demo()[:1])
            self.assertEqual(frequency, 50)

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

    def test_plot_uses_recorded_demonstration(self):
        original, lengths = original_trajectory(self.weights, "pick_weights.npy")
        np.testing.assert_array_equal(original, np.vstack(self.segments))
        self.assertEqual(lengths, [101, 101, 101])
        with tempfile.TemporaryDirectory() as directory:
            weights_path = Path(directory)/"workflow_fixture_weights.npy"
            recorder = TrajectoryRecorder()
            recorder.start()
            for point in self.segments[0]:
                recorder.append(point)
            recording = recorder.save(Path(directory)/"workflow_fixture_trajectory.npz", "OpenclawPickCube")
            legacy = dict(self.weights)
            legacy.pop("trajectory")
            original, lengths = original_trajectory(legacy, weights_path)
            np.testing.assert_array_equal(original, self.segments[0])
            recording_dir = Path(directory) / "trajectory"
            recording_dir.mkdir()
            recording = recording.rename(recording_dir / recording.name)
            with patch("openclaw_control.dmp_plot.TRAJECTORY_DIR", recording_dir):
                original, lengths = original_trajectory(legacy, weights_path)
            np.testing.assert_array_equal(original, self.segments[0])
            original, _ = original_trajectory(self.weights, weights_path, recording)
            np.testing.assert_array_equal(original, self.segments[0])
            with self.assertRaises(FileNotFoundError):
                original_trajectory(legacy, Path(directory)/"missing_weights.npy")

    def test_comparison_saves_both_figures(self):
        original = np.vstack(self.segments)
        generated = generate_trajectory(self.weights, goal=[.4, .1, .5])
        with tempfile.TemporaryDirectory() as directory:
            paths = plot_comparison(original, generated, Path(directory)/"comparison.png", [101]*3)
            self.assertEqual([p.name for p in paths], ["comparison.png", "comparison_3d.png"])
            for path in paths:
                self.assertTrue(path.read_bytes().startswith(b"\x89PNG\r\n\x1a\n"))
                self.assertGreater(path.stat().st_size, 10000)

    def test_comparison_show_only_does_not_save(self):
        import matplotlib.pyplot as plt
        from matplotlib.figure import Figure
        def check_single_window(*args, **kwargs):
            self.assertEqual(len(plt.get_fignums()), 1)
        with patch.object(plt, "show", side_effect=check_single_window) as show, patch.object(Figure, "savefig") as save:
            paths = plot_comparison(self.segments[0], self.segments[0], show=True)
        self.assertEqual(paths, ())
        show.assert_called_once_with(block=True)
        save.assert_not_called()
        self.assertEqual(plt.get_fignums(), [])

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
