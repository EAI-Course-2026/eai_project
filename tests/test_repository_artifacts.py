"""Exercise the shared Git boundary, including force-added private payloads."""
from pathlib import Path
import runpy
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(shutil.which("git"), "Git is required for repository boundary checks")
class ArtifactBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.git("init", "--quiet")
        for name in (
            "README.md", "CONTRIBUTING.md", "AGENTS.md", "deliverables/README.md",
            "deliverables/assignment-03/README.md", "outputs/README.md",
            "environments/training/README.md",
        ):
            self.put(name, "# Synthetic guide\n")
        for name in (".github/repository-policy.json", ".github/CODEOWNERS", ".gitignore"):
            self.put(name, (ROOT / name).read_text(encoding="utf-8"))
        self.put(".git/info/exclude", "/deliverables/\n")
        namespace = runpy.run_path(str(ROOT / "scripts/check_repository.py"))
        self.check = namespace["check"]
        self.check.__globals__["ROOT"] = self.root

    def git(self, *args, check=True):
        return subprocess.run(
            ["git", *args], cwd=self.root, check=check,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        )

    def put(self, name, text):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")

    def test_shared_guides_override_machine_exclusion(self):
        guides = ["deliverables/README.md", "deliverables/assignment-03/README.md", "outputs/README.md"]
        for name in guides:
            self.assertEqual(self.git("check-ignore", "-q", name, check=False).returncode, 1, name)
        self.git("add", "--", *guides)
        self.assertEqual(self.check(), [])

    def test_payloads_remain_ignored_including_legacy_paths(self):
        for name in (
            "outputs/policy/run.json", "deliverables/assignment-03/2026-10-24-r01/report.pdf",
            "deliverables/assignment-03/2026-10-24-r01/README.md",
            "assignment2_handin/old.zip", "scripts/package_assignment2.py",
        ):
            with self.subTest(name=name):
                self.put(name, "synthetic local payload\n")
                self.assertEqual(self.git("check-ignore", "-q", name, check=False).returncode, 0)

    def test_force_added_payloads_are_rejected_even_when_named_readme(self):
        names = [
            "outputs/policy/run.json", "outputs/policy/README.md",
            "deliverables/assignment-03/2026-10-24-r01/report.pdf",
            "deliverables/assignment-03/2026-10-24-r01/README.md",
            "deliverables/misc/README.md",
        ]
        for name in names:
            self.put(name, "synthetic local payload\n")
        self.git("add", "--force", "--", *names)
        failures = self.check()
        for name in names:
            self.assertTrue(any(name in failure for failure in failures), name)
        self.assertEqual(len(failures), len(names))


if __name__ == "__main__":
    unittest.main()
