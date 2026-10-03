"""Read-only comparison of GitHub settings with the committed repository policy.

Requires the GitHub CLI authenticated with repository administration read access.
Never changes repository settings, branches, files or hardware.
"""

from __future__ import annotations

import base64
import json
from pathlib import Path
import subprocess
from urllib.parse import quote


def api(endpoint: str) -> dict:
    result = subprocess.run(
        ["gh", "api", endpoint], capture_output=True, text=True, encoding="utf-8", timeout=45
    )
    if result.returncode:
        raise RuntimeError(f"GitHub read failed for {endpoint}: {result.stderr.strip()}")
    return json.loads(result.stdout)


def compare(policy: dict) -> list[str]:
    problems: list[str] = []
    repository = policy["repository"]
    remote = api(f"repos/{repository}")
    for key, expected in policy["settings"].items():
        if remote.get(key) != expected:
            problems.append(f"Repository {key}: expected {expected!r}, found {remote.get(key)!r}")

    expected = policy["protection"]
    for branch in policy["permanent_branches"]:
        encoded = quote(branch, safe="")
        state = api(f"repos/{repository}/branches/{encoded}")
        if not state.get("protected"):
            problems.append(f"{branch}: branch is unprotected")
            continue
        actual = api(f"repos/{repository}/branches/{encoded}/protection")
        required = actual.get("required_status_checks", {})
        if required.get("strict") != expected["required_status_checks"]["strict"]:
            problems.append(f"{branch}: status checks do not require an up-to-date branch")
        wanted_checks = {(c["context"], c["app_id"]) for c in expected["required_status_checks"]["checks"]}
        found_checks = {(c["context"], c["app_id"]) for c in required.get("checks", [])}
        if wanted_checks != found_checks:
            problems.append(f"{branch}: required check names/source apps differ")
        reviews = actual.get("required_pull_request_reviews", {})
        for key, value in expected["required_pull_request_reviews"].items():
            if reviews.get(key) != value:
                problems.append(f"{branch}: review policy {key} differs")
        for key in (
            "enforce_admins", "required_linear_history", "allow_force_pushes",
            "allow_deletions", "required_conversation_resolution",
        ):
            if actual.get(key, {}).get("enabled") != expected[key]:
                problems.append(f"{branch}: protection {key} differs")
        if actual.get("restrictions") != expected["restrictions"]:
            problems.append(f"{branch}: push restrictions differ")
        if reviews.get("bypass_pull_request_allowances") and any(reviews["bypass_pull_request_allowances"].values()):
            problems.append(f"{branch}: review bypass allowances are present")

        content = api(f"repos/{repository}/contents/.github/CODEOWNERS?ref={encoded}")
        ownership = base64.b64decode(content["content"]).decode("utf-8")
        global_owners = [line.split()[1:] for line in ownership.splitlines() if line.startswith("* ")]
        if global_owners != [[f"@{owner}" for owner in policy["codeowners"]]]:
            problems.append(f"{branch}: remote CODEOWNERS differ")
        print(f"Checked {branch} at {state['commit']['sha']}")
    return problems


if __name__ == "__main__":
    root = Path(__file__).resolve().parents[1]
    policy = json.loads((root / ".github/repository-policy.json").read_text(encoding="utf-8"))
    try:
        failures = compare(policy)
    except (RuntimeError, FileNotFoundError, subprocess.TimeoutExpired) as error:
        print(error)
        raise SystemExit(1) from error
    if failures:
        print("\n".join(failures))
        raise SystemExit(1)
    print("Remote settings, permanent branches, protection and CODEOWNERS match policy.")
