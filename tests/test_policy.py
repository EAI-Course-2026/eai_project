"""Real framework integration and fail-closed policy regressions; no devices are opened."""
from dataclasses import replace
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch
from urllib.error import HTTPError

import numpy as np

from eai_robot.policy.artifacts import freeze_checkpoint
from eai_robot.policy.contracts import JOINTS, KEYS, Manifest, UNITS, sha256, vector
from eai_robot.policy.datasets import export_dataset, training_recipe
from eai_robot.policy.execution import PortLease, execute_chunk
from eai_robot.policy.observations import fake_observation, Observation
from eai_robot.policy.recording import Journal, validate_episode
from eai_robot.policy.runtime import FakeRuntime, LeRobotRuntime
from eai_robot.policy.safety import Budgets, Gate
from eai_robot.policy.transport import decode_observation, encode_observation, policy_server, RemoteRuntime

ROOT = Path(__file__).resolve().parents[1]


def fixture():
    return Manifest.load(ROOT / "configs/policy/fake.json")


class FakeClock:
    def __init__(self):
        self.now = 10.0

    def __call__(self):
        return self.now


class ContractTests(unittest.TestCase):
    def test_units_and_joint_order_are_not_inferred(self):
        original = fixture()
        for change in ({"state_order": list(reversed(JOINTS))}, {"action_order": list(reversed(JOINTS))}, {"action_units": "degrees"}, {"framework_sha": "other"}, {"action_hz": True}, {"max_chunk_steps": 0}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                Manifest(original.path, {**original.data, **change}).validate()

    def test_unknown_semantics_and_fake_cannot_enable_motion(self):
        manifest = fixture()
        path = ROOT / "calibration/scs215_so101.json"
        data = {**manifest.data, "calibration_sha256": sha256(path)}
        Manifest(manifest.path, data).require_robot_semantics(path)
        for change in ({"state_units": "unknown"}, {"action_units": "unknown"}, {"action_mode": "unknown"}, {"calibration_sha256": "bad"}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                Manifest(manifest.path, {**data, **change}).require_robot_semantics(path)
        with self.assertRaisesRegex(ValueError, "real, matched"):
            Manifest(manifest.path, {**data, "motion_verified": True}).require_robot_semantics(path, motion=True)

    def test_duplicate_camera_source_rejected(self):
        manifest = fixture()
        data = {**manifest.data, "cameras": {**manifest.data["cameras"], "observation.images.wrist": {"source": "front", "shape": [3, 32, 32]}}}
        with self.assertRaisesRegex(ValueError, "distinct"):
            Manifest(manifest.path, data).validate()

    def test_values_are_finite_in_range_not_boolean(self):
        for value in ([0] * 5, [False] * 6, [0, 0, 0, 0, 0, -1], [101, 0, 0, 0, 0, 50], [float("nan"), 0, 0, 0, 0, 50]):
            with self.subTest(value=value), self.assertRaises(ValueError):
                vector(value)

    def test_artifacts_cannot_escape_inventory(self):
        manifest = fixture()
        for name in ("../model.safetensors", "/model.safetensors"):
            with self.assertRaises(ValueError):
                manifest.artifact_path(name)

    def test_image_dtype_layout_timing_and_missing_view(self):
        manifest = fixture()
        for change in ({"images": {}}, {"images": {"front": np.zeros((3, 32, 32), dtype=np.uint8)}}, {"images": {"front": np.zeros((32, 32, 3), dtype=float)}}, {"camera_times": {"front": float("nan")}}, {"completed_at": -1.0}):
            obs = fake_observation(manifest)
            for key, value in change.items():
                setattr(obs, key, value)
            with self.subTest(change=list(change)), self.assertRaises(ValueError):
                obs.validate(manifest)

    def test_batch_rgb_values_and_explicit_order(self):
        manifest = fixture()
        obs = fake_observation(manifest)
        obs.images["front"][..., 0] = 255
        batch = obs.model_batch(manifest)
        self.assertEqual(tuple(batch["observation.images.front"].shape), (3, 32, 32))
        self.assertEqual(float(batch["observation.images.front"][0].mean()), 1)
        self.assertEqual(float(batch["observation.images.front"][2].mean()), 0)
        self.assertEqual(batch["observation.state"].tolist(), [0, 0, 0, 0, 0, 50])


class GateTests(unittest.TestCase):
    def setUp(self):
        self.manifest = fixture()
        self.clock = FakeClock()
        self.gate = Gate(self.manifest, clock=self.clock)
        self.obs = fake_observation(self.manifest)
        self.obs.captured_at = self.obs.completed_at = self.clock.now
        self.obs.camera_times = {"front": self.clock.now}
        self.gate.arm(self.obs.state)
        self.actions = np.repeat(self.obs.state[None], 3, axis=0)

    def test_valid_chunk_is_scheduled_without_bursting(self):
        ticket = self.gate.request(self.obs)
        self.gate.accept(ticket, self.actions)
        action = self.gate.next_action(self.obs.state, torque_ok=True)
        np.testing.assert_array_equal(action, self.obs.state)
        self.assertIsNone(self.gate.next_action(self.obs.state, torque_ok=True))
        self.clock.now += 1 / 30
        self.assertIsNotNone(self.gate.next_action(self.obs.state, torque_ok=True))

    def test_missing_schedule_stops(self):
        ticket = self.gate.request(self.obs)
        self.gate.accept(ticket, self.actions)
        self.clock.now += 0.1
        with self.assertRaisesRegex(RuntimeError, "schedule"):
            self.gate.next_action(self.obs.state, torque_ok=True)
        self.assertFalse(self.gate.active)

    def test_old_reply_cannot_restore_stopped_or_new_session(self):
        ticket = self.gate.request(self.obs)
        self.gate.stop()
        self.gate.arm(self.obs.state)
        self.assertNotEqual(ticket.session, self.gate.session)
        with self.assertRaises(RuntimeError):
            self.gate.accept(ticket, self.actions)
        self.assertEqual(self.gate.queue, [])

    def test_late_duplicate_wrong_identity_fail_closed(self):
        for mode in ("late", "sequence", "session", "manifest", "duplicate"):
            self.setUp()
            ticket = self.gate.request(self.obs)
            if mode == "late":
                self.clock.now += 0.25
            elif mode in {"sequence", "session", "manifest"}:
                ticket = replace(ticket, **{mode: 2 if mode == "sequence" else "other"})
            elif mode == "duplicate":
                self.gate.accept(ticket, self.actions)
            with self.subTest(mode=mode), self.assertRaises(RuntimeError):
                self.gate.accept(ticket, self.actions)
            self.assertFalse(self.gate.active)

    def test_stale_camera_old_state_and_skew_rejected(self):
        for mode in ("camera", "state", "skew", "future"):
            self.setUp()
            if mode == "camera":
                self.obs.camera_times["front"] -= 1
            elif mode == "state":
                self.obs.captured_at -= 1
            elif mode == "skew":
                self.obs.captured_at -= 0.15
            else:
                self.obs.completed_at += 1
            with self.subTest(mode=mode), self.assertRaises(RuntimeError):
                self.gate.request(self.obs)
            self.assertFalse(self.gate.active)

    def test_nan_out_of_range_bad_shape_oversized_and_first_jump(self):
        for mode in ("nan", "range", "shape", "oversized", "jump", "later_jump"):
            self.setUp()
            ticket = self.gate.request(self.obs)
            actions = self.actions.copy()
            if mode == "nan":
                actions[0, 0] = np.nan
            elif mode == "range":
                actions[0, 5] = -1
            elif mode == "shape":
                actions = actions[:, :5]
            elif mode == "oversized":
                actions = np.repeat(actions, 2, axis=0)
            elif mode == "jump":
                actions[0, 0] = 2
            else:
                actions[1, 0] = 2
            with self.subTest(mode=mode), self.assertRaises(RuntimeError):
                self.gate.accept(ticket, actions)
            self.assertFalse(self.gate.active)

    def test_oversized_time_horizon_rejected(self):
        self.gate = Gate(self.manifest, Budgets(action_ttl_s=0.05), clock=self.clock)
        self.gate.arm(self.obs.state)
        ticket = self.gate.request(self.obs)
        with self.assertRaisesRegex(RuntimeError, "deadline"):
            self.gate.accept(ticket, self.actions)

    def test_torque_alarm_tracking_step_and_expired_lease(self):
        for mode in ("torque", "alarm", "tracking", "feedback_step", "expired"):
            self.setUp()
            ticket = self.gate.request(self.obs)
            self.gate.accept(ticket, self.actions)
            feedback = self.obs.state.copy()
            if mode == "tracking":
                feedback[0] = 5
            elif mode == "feedback_step":
                feedback[0] = 2
            elif mode == "expired":
                self.clock.now += 0.6
            with self.subTest(mode=mode), self.assertRaises(RuntimeError):
                self.gate.next_action(feedback, torque_ok=mode != "torque", alarm=mode == "alarm")

    def test_disconnect_invalidates_and_records_rejection(self):
        class Disconnected:
            def predict(self, observation):
                raise ConnectionError("disconnected")
        with self.assertRaises(ConnectionError):
            execute_chunk(None, self.gate, self.obs, Disconnected())
        self.assertFalse(self.gate.active)
        self.assertIsNone(self.gate.pending)

    def test_budget_validation(self):
        for value in (0, -1, float("nan"), True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                Budgets(max_step=value)


class TransportTests(unittest.TestCase):
    def test_roundtrip_preserves_rgb_without_pickle(self):
        manifest = fixture()
        obs = fake_observation(manifest)
        obs.images["front"][0, 0] = [1, 2, 3]
        decoded = decode_observation(encode_observation(obs), manifest)
        np.testing.assert_array_equal(obs.images["front"], decoded.images["front"])
        payload = encode_observation(obs)
        payload["images"]["front"]["shape"] = [4097, 32, 3]
        with self.assertRaises(ValueError):
            decode_observation(payload, manifest)

    def test_remote_endpoint_restrictions(self):
        for url in ("http://example.com", "https://127.0.0.1", "http://secret@127.0.0.1", "http://127.0.0.1/?token=secret"):
            with self.subTest(url=url), self.assertRaises(ValueError):
                RemoteRuntime(fixture(), url, token="x" * 32)

    def test_authenticated_real_http_and_manifest_mismatch(self):
        manifest = fixture()
        server = policy_server(FakeRuntime(manifest), 0, token="x" * 32)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        try:
            url = f"http://127.0.0.1:{server.server_port}"
            runtime = RemoteRuntime(manifest, url, token="x" * 32)
            actions, elapsed = runtime.predict(fake_observation(manifest))
            self.assertEqual(actions.shape, (3, 6))
            with self.assertRaises(HTTPError) as wrong_token:
                RemoteRuntime(manifest, url, token="y" * 32).predict(fake_observation(manifest))
            self.assertEqual(wrong_token.exception.code, 401)
            altered = Manifest(manifest.path, {**manifest.data, "action_hz": 20})
            with self.assertRaises(HTTPError) as mismatch:
                RemoteRuntime(altered, url, token="x" * 32).predict(fake_observation(altered))
            self.assertEqual(mismatch.exception.code, 400)
        finally:
            server.shutdown()
            worker.join()
            server.server_close()


class RecordingTests(unittest.TestCase):
    def test_native_dataset_export_uses_sent_labels_and_episode_split(self):
        manifest = fixture()
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            episodes = []
            for i in range(2):
                path = directory / f"episode-{i}"
                journal = Journal(path, manifest, mode="demonstration", scene_id=f"scene-{i}")
                obs = fake_observation(manifest)
                journal.append(obs, requested=[1, 0, 0, 0, 0, 50], sent=dict(zip(KEYS, obs.state)), feedback=obs.state, feedback_at=obs.completed_at + .001)
                journal.close(complete=True, outcome="success")
                episodes.append(path)
            split = directory / "split.json"
            split.write_text(json.dumps({"train": [0], "evaluation": [1]}))
            destination = directory / "dataset"
            result = export_dataset(episodes, manifest, root=destination, repo_id="local/policy_test", split_file=split)
            self.assertFalse(result["uploaded"])
            from lerobot.datasets.lerobot_dataset import LeRobotDataset
            dataset = LeRobotDataset("local/policy_test", root=destination)
            self.assertEqual(float(dataset[0]["action"][0]), 0)
            recipe = training_recipe("act", destination, "local/policy_test", output_dir="outputs/train/test")
            self.assertIn("--dataset.episodes=[0]", recipe["argv"])
            self.assertEqual(recipe["evaluation_episodes"], [1])
            split.write_text(json.dumps({"train": [0], "evaluation": [0]}))
            with self.assertRaisesRegex(ValueError, "disjoint"):
                export_dataset(episodes, manifest, root=directory / "bad", repo_id="local/policy_test", split_file=split)

    def test_operator_collection_records_actual_send_and_stop(self):
        from eai_robot.policy.teleoperation import record_demonstration
        manifest = fixture()
        clock = FakeClock()

        class Operator:
            stop_event = threading.Event()

            def action(self, previous):
                target = previous.copy()
                target[0] += 0.25
                return target

        class Owner:
            state = np.array([0, 0, 0, 0, 0, 50.])

            def feedback(self):
                return self.state.copy()

            def health(self):
                return {"torque_ok": True, "alarm": False}

            def observe(self, task):
                return Observation(self.state.copy(), {"front": np.zeros((32, 32, 3), dtype=np.uint8)}, task, clock.now, clock.now, {"front": clock.now})

            def send(self, action, *, before_write):
                before_write()
                self.state = action.copy()
                return dict(zip(KEYS, action))

        def sleep(delay):
            clock.now += delay

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "operator"
            journal = Journal(path, manifest, mode="demonstration")
            frames = record_demonstration(Owner(), manifest, Operator(), journal, Budgets(), "simulated operator", .08, clock=clock, sleep=sleep)
            journal.close(complete=True, outcome="success")
            report = validate_episode(path, manifest, demonstrations=True)
            self.assertEqual(frames, report["frames"])
            self.assertEqual(report["rows"][0]["sent"][0], .25)

    def test_shadow_cannot_be_exported_as_demonstrations(self):
        manifest = fixture()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "episode"
            journal = Journal(path, manifest, mode="shadow")
            journal.append(fake_observation(manifest), requested=[0, 0, 0, 0, 0, 50])
            journal.close(complete=True)
            self.assertEqual(validate_episode(path, manifest)["frames"], 1)
            with self.assertRaisesRegex(ValueError, "successful demonstrations"):
                validate_episode(path, manifest, demonstrations=True)

    def test_sent_and_requested_differ_and_feedback_required(self):
        manifest = fixture()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "episode"
            obs = fake_observation(manifest)
            journal = Journal(path, manifest, mode="demonstration")
            journal.append(obs, requested=[1, 0, 0, 0, 0, 50], sent=dict(zip(KEYS, [0, 0, 0, 0, 0, 50])), feedback=obs.state, feedback_at=obs.completed_at + .001)
            journal.close(complete=True, outcome="success")
            report = validate_episode(path, manifest, demonstrations=True)
            self.assertEqual(report["rows"][0]["requested"][0], 1)
            self.assertEqual(report["rows"][0]["sent"][0], 0)
            row = report["rows"][0]
            row["feedback"] = None
            (path / "frames.jsonl").write_text(json.dumps(row) + "\n")
            with self.assertRaisesRegex(ValueError, "subsequent feedback"):
                validate_episode(path, manifest, demonstrations=True)

    def test_port_lease_exclusion_and_release(self):
        first = PortLease("policy-test-nonexistent-device")
        second = PortLease("policy-test-nonexistent-device")
        first.acquire()
        try:
            with self.assertRaises(RuntimeError):
                second.acquire()
        finally:
            first.release()
        second.acquire()
        second.release()


class RealACTTests(unittest.TestCase):
    def test_pi05_load_errors_cannot_return_random_model(self):
        from eai_robot.policy.runtime import load_pi05_strict
        policy_class = Mock()
        config = Mock(device="cpu")
        with patch("safetensors.torch.load_file", side_effect=ValueError("corrupt weight file")):
            with self.assertRaisesRegex(ValueError, "corrupt weight"):
                load_pi05_strict(policy_class, config, Path("unused"))
        policy_class.assert_not_called()
        policy = policy_class.return_value
        policy._fix_pytorch_state_dict_keys.return_value = {"a": 1}
        policy._prepare_pretrained_state_dict.return_value = {"model.a": 1}
        policy.load_state_dict.side_effect = RuntimeError("missing keys")
        with patch("safetensors.torch.load_file", return_value={"a": 1}):
            with self.assertRaisesRegex(RuntimeError, "missing keys"):
                load_pi05_strict(policy_class, config, Path("unused"))
        policy.load_state_dict.assert_called_once_with({"model.a": 1}, strict=True)
        policy.eval.assert_not_called()

    def test_saved_checkpoint_offline_chunk_and_checksum_tamper(self):
        import torch
        from lerobot.configs import FeatureType, PolicyFeature, NormalizationMode
        from lerobot.policies.act.configuration_act import ACTConfig
        from lerobot.policies.act.modeling_act import ACTPolicy
        from lerobot.policies.factory import make_pre_post_processors
        from safetensors.torch import save_model
        config = ACTConfig(device="cpu", input_features={"observation.state": PolicyFeature(FeatureType.STATE, (6,)), "observation.images.front": PolicyFeature(FeatureType.VISUAL, (3, 32, 32))}, output_features={"action": PolicyFeature(FeatureType.ACTION, (6,))}, chunk_size=3, n_action_steps=3, dim_model=32, n_heads=4, dim_feedforward=64, n_encoder_layers=1, n_decoder_layers=1, n_vae_encoder_layers=1, pretrained_backbone_weights=None, normalization_mapping={"VISUAL": NormalizationMode.IDENTITY, "STATE": NormalizationMode.MEAN_STD, "ACTION": NormalizationMode.MEAN_STD})
        stats = {"observation.state": {"mean": torch.ones(6) * 10, "std": torch.ones(6) * 2}, "action": {"mean": torch.ones(6) * 5, "std": torch.ones(6) * 3}}
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "checkpoint"
            root.mkdir()
            config.save_pretrained(root)
            model = ACTPolicy(config)
            save_model(model, root / "model.safetensors")
            pre, post = make_pre_post_processors(config, dataset_stats=stats)
            pre.save_pretrained(root)
            post.save_pretrained(root)
            manifest = freeze_checkpoint(root, Path(directory) / "manifest.json", revision="a" * 40, action_hz=30)
            runtime = LeRobotRuntime(manifest)
            obs = fake_observation(manifest)
            batch = runtime.pre(obs.model_batch(manifest))
            np.testing.assert_allclose(batch["observation.state"].numpy()[0], [-5, -5, -5, -5, -5, 20], atol=1e-5)
            actions, elapsed = runtime.predict(obs)
            self.assertEqual(actions.shape, (3, 6))
            self.assertTrue(np.isfinite(actions).all())
            # Verify official unnormalization exactly once, using the same model output.
            with torch.inference_mode():
                raw = runtime.policy.predict_action_chunk(runtime.pre(obs.model_batch(manifest)))
            np.testing.assert_allclose(actions, raw.numpy()[0] * 3 + 5, atol=1e-5)
            config_path = root / "config.json"
            config_path.write_text(config_path.read_text() + " ")
            with self.assertRaisesRegex(ValueError, "checksum"):
                manifest.verify_files()


if __name__ == "__main__":
    unittest.main()
