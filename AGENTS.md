# Agent contract

Read README.md and CONTRIBUTING.md before changing this repository.

- This is the canonical application repository. LeRobot is a pinned dependency; do not edit installed framework files or mutate its upstream repository as part of application maintenance.
- Permanent branches are main and develop. Never delete, reset or force-push them. Use codex/* task branches from develop; normal PRs target develop. Main hotfixes must be merged back into develop.
- Use reviewed PRs and the protection policy in .github/repository-policy.json. Do not bypass another maintainer's required review or weaken checks to finish a task. The initial, user-authorized unprotected bootstrap is documented in CONTRIBUTING.md.
- Check the exact PR head and the target branch's post-merge SHA. Report local, PR and post-merge CI separately. A green PR is not proof that main/develop's push checks passed.
- Keep root control and environments/training dependency locks separate and consistent. Use uv 0.11.7, Python 3.12 and locked synchronization. Do not create permanent OS/CUDA branches without a concrete maintenance need.
- Run relevant tests and the repository checks described in CONTRIBUTING.md. Complete required CI before merging; retain regression tests for motion leases, stale input, feedback and torque faults.
- Repository maintenance must not open serial ports, enable torque or move hardware. Hardware tests require an authorized hardware task, one process per serial port and explicit motion enable.
- Distinguish software, simulated devices, GPU and real-arm acceptance in documentation. Never expand calibration or motion limits to work around an entry check.
- Do not commit machine-local ports, cameras, paths, credentials, datasets, models or videos. Preserve local artifacts and worktrees until local cleanup is separately authorized.
- Delete temporary branches only after merge, no unique commits, task completion and successful post-merge CI; retain active release branches and branches still in use.
- Update user-facing documentation when behavior or collaboration policy changes. Preserve historical evidence and add corrections when a previous report overstated validation.
