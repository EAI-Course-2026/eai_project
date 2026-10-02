"""The acceptance gate must distinguish software installation from GPU execution."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

spec = importlib.util.spec_from_file_location(
    "training_check", Path(__file__).resolve().parents[1] / "scripts/check_training_env.py"
)
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)


class TrainingEnvironmentTests(unittest.TestCase):
    def run_check(self, *, require_cuda=False, available=False, version="2.11.0+cu128",
                  runtime="12.8", commit=checker.FORK_COMMIT, url=checker.FORK_URL,
                  kernel_error=None):
        cuda = SimpleNamespace(
            is_available=lambda: available,
            get_device_name=lambda _: "Fake GPU",
            get_device_capability=lambda _: (8, 6),
            get_arch_list=lambda: ["sm_86"],
            synchronize=Mock(),
        )
        ones = Mock(side_effect=kernel_error) if kernel_error else Mock(return_value=np.ones((32, 32)))
        torch = SimpleNamespace(__version__=version, version=SimpleNamespace(cuda=runtime),
                                cuda=cuda, ones=ones, all=np.all)
        dist = SimpleNamespace(version="0.6.2", read_text=lambda _: json.dumps({
            "url": url, "vcs_info": {"commit_id": commit},
        }))
        out, err = io.StringIO(), io.StringIO()
        with patch.object(checker.platform, "system", return_value="Windows"), \
             patch.object(checker.sys, "version_info", (3, 12, 13)), \
             patch.object(checker.importlib.metadata, "distribution", return_value=dist), \
             patch.dict("sys.modules", {"torch": torch}), \
             contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = checker.check(require_cuda=require_cuda)
        return code, out.getvalue() + err.getvalue(), ones, cuda

    def test_no_gpu_can_pass_software_installation_without_claiming_gpu_acceptance(self):
        code, text, ones, _ = self.run_check()
        self.assertEqual(code, 0)
        self.assertIn("GPU execution and training are NOT verified", text)
        ones.assert_not_called()

    def test_required_gpu_unavailable_fails_instead_of_falling_back_to_cpu(self):
        code, text, ones, _ = self.run_check(require_cuda=True)
        self.assertEqual(code, 1)
        self.assertIn("CUDA is unavailable", text)
        ones.assert_not_called()

    def test_cpu_control_wheel_is_rejected_in_windows_training_environment(self):
        code, text, _, _ = self.run_check(version="2.11.0+cpu", runtime=None)
        self.assertEqual(code, 1)
        self.assertIn("Expected the locked PyTorch", text)

    def test_wrong_framework_commit_is_rejected(self):
        self.assertEqual(self.run_check(commit="wrong")[0], 1)

    def test_same_package_version_from_other_repo_is_rejected(self):
        self.assertEqual(self.run_check(url="https://github.com/huggingface/lerobot.git")[0], 1)

    def test_available_gpu_with_kernel_failure_is_not_accepted(self):
        code, text, _, _ = self.run_check(require_cuda=True, available=True,
                                         kernel_error=RuntimeError("unsupported architecture"))
        self.assertEqual(code, 1)
        self.assertIn("unsupported architecture", text)

    def test_gpu_acceptance_requires_compute_and_synchronization(self):
        code, text, ones, cuda = self.run_check(require_cuda=True, available=True)
        self.assertEqual(code, 0)
        ones.assert_called_once_with((32, 32), device="cuda:0")
        cuda.synchronize.assert_called_once()
        self.assertIn("CUDA matrix multiplication: OK", text)


if __name__ == "__main__":
    unittest.main()
