"""Check local documentation links and the versioned repository policy offline."""

from __future__ import annotations

import json
from pathlib import Path
import re
import subprocess
from urllib.parse import unquote, urlsplit


ROOT = Path(__file__).resolve().parents[1]


def check() -> list[str]:
    errors: list[str] = []
    if (ROOT / ".git").exists():
        tracked = subprocess.check_output(
            ["git", "ls-files", "-z", "--", "deliverables"], cwd=ROOT
        ).decode("utf-8").split("\0")
        for name in filter(None, tracked):
            parts = Path(name).parts
            if not (parts[-1] == "README.md" and len(parts) in (2, 3)):
                errors.append(f"Hand-in payload must remain local, but is tracked: {name}")
    documents = [ROOT / name for name in ("README.md", "CONTRIBUTING.md", "AGENTS.md")]
    documents += [ROOT / "deliverables/README.md"]
    documents += list((ROOT / "deliverables").glob("*/README.md"))
    documents += list((ROOT / "docs").rglob("*.md"))
    documents += [ROOT / "calibration/README.md", ROOT / "environments/training/README.md"]
    for document in documents:
        # Original migrated documents intentionally keep their historical paths.
        if (ROOT / "docs/course/source") in document.parents:
            continue
        source = re.sub(r"```.*?```", "", document.read_text(encoding="utf-8"), flags=re.S)
        for match in re.finditer(r"!?\[[^\]\n]*\]\(([^\s)]+)(?:\s+[^)]*)?\)", source):
            target = match.group(1).strip("<>")
            parsed = urlsplit(target)
            if parsed.scheme or parsed.netloc or not parsed.path:
                continue
            destination = (document.parent / unquote(parsed.path)).resolve()
            if not destination.is_relative_to(ROOT) or not destination.exists():
                errors.append(f"{document.relative_to(ROOT)}: missing local link {target}")

    policy = json.loads((ROOT / ".github/repository-policy.json").read_text(encoding="utf-8"))
    branches = policy["permanent_branches"]
    if len(branches) != len(set(branches)) or policy["settings"]["default_branch"] not in branches:
        errors.append("Policy must include its default branch once among permanent branches")
    ownership = (ROOT / ".github/CODEOWNERS").read_text(encoding="utf-8")
    actual = [line.split()[1:] for line in ownership.splitlines() if line.startswith("* ")]
    if actual != [[f"@{owner}" for owner in policy["codeowners"]]]:
        errors.append("Global CODEOWNERS differ from repository policy")
    checks = policy["protection"]["required_status_checks"]["checks"]
    if not checks or len({item["context"] for item in checks}) != len(checks):
        errors.append("Required checks must have unique names")
    if any(not isinstance(item["app_id"], int) or item["app_id"] <= 0 for item in checks):
        errors.append("Each required check must identify its source GitHub App")
    return errors


if __name__ == "__main__":
    failures = check()
    if failures:
        print("\n".join(failures))
        raise SystemExit(1)
    print("Active documentation links and repository policy are consistent.")
