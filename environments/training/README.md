# Training environment

This project has its own `pyproject.toml`, `uv.lock` and `.venv`. It installs
the exact LeRobot course fork at `6a077907c7989635218969ee78f5436f8faec92b`,
including training tools. Windows/Linux use locked PyTorch `2.11.0+cu128`;
Mac uses native `2.11.0` and cannot run NVIDIA CUDA.

The same isolated environment now explicitly includes the fork's `smolvla` and
`pi` extras. ACT uses the core policy dependencies. The root hardware environment
keeps its existing lock. Run `python scripts/policy.py doctor` with this project
to import all three policy families; add `--require-cuda` on the GPU host.
See the [policy deployment guide](../../docs/policy/README.md) for frozen artifacts,
offline inference, data checks and training recipes. Imports do not verify model
weights, GPU performance or task success.

From the application repository root:

```sh
uv sync --locked --project environments/training
uv run --locked --project environments/training python scripts/check_training_env.py
```

On the NVIDIA GPU machine:

```sh
uv run --locked --project environments/training python scripts/check_training_env.py --require-cuda
```

On Windows, `.\scripts\setup_training_windows.cmd` installs this environment
and runs the required GPU check. A successful software-only check does not
validate a GPU or training job. See [the Windows/CUDA/Conda guide](../../docs/environments.md)
for installation, diagnostics, example training commands and acceptance scope.

Use `uv run --locked --project environments/training ...` for daily training
commands. The Windows installer and CI retain `--no-sync` only after explicit
locked synchronization. See the [shared command policy](../../docs/environments.md#command-policy-on-windows-and-macos).
