# Shared calibration contract — Windows and macOS

`scs215_so101.json` is the single active calibration for the team's physical
SCS215/SO-101 arm. Every collaborator uses the same file and the same pinned
LeRobot 0.6.2 fork (`6a07790`). A port name is not a robot identity.

| Location | Meaning | Committed? |
| --- | --- | --- |
| `calibration/scs215_so101.json` | Joint IDs, direction, offsets and command ranges in raw encoder counts | Yes |
| `calibration/scs215_so101.meta.json` | Hardware asset label, capture method/date and verification evidence | Yes |
| `calibration/history/` | Superseded snapshots, inactive candidates, paired old poses and write receipts | Yes; never selected by default |
| `configs/hardware.local.toml` | This computer's serial port and camera index | No; ignored by Git |
| `outputs/` | Machine-specific captures and diagnostic logs | No; ignored by Git |

The metadata `hardware_id` is the team's asset label `scs215_so101`, not a USB
adapter name, serial port or OS-specific LeRobot cache ID. Metadata must not
contain host ports, host usernames or absolute paths. Generic device examples
in setup documentation are examples, not shared runtime values.

## Limits and verification

EEPROM limits are stored command restrictions. The active JSON matches the
recorded EEPROM readback. Manually observed safe travel is evidence of the
user-selected range, not a measurement of mechanical hard stops. Control may
apply an additional inward margin. The 3-count feedback allowance never expands
command limits. Shared files preserve the fact that powered motion and physical
FK alignment have not yet been verified.

A new computer needs local device configuration, not recalibration of an
unchanged arm. Changed joints, IDs, assembly or EEPROM require a fresh hardware
comparison and, where appropriate, a reviewed recalibration. Connection is
read-only; it does not automatically overwrite EEPROM or copy a stale fork
snapshot into a per-user HF cache.

Daily commands use `uv run --locked`. Before hardware work, synchronize
with `uv sync --locked`; use `--no-sync` only in an already verified environment.
See the [shared command policy](../docs/environments.md#command-policy-on-windows-and-macos).

## Local setup

Copy `configs/hardware.example.toml` to `configs/hardware.local.toml`. Windows
users can use `Copy-Item` in PowerShell; macOS users can use `cp`. Set
`[serial].port` to the port shown by that OS, and `[camera].index_or_path` to the
integer index verified on that computer. Local files can contain only the
sections being overridden; missing values come from the portable example.

The commands below are identical on both platforms once local configuration is
set:

```sh
uv sync --locked
uv run --locked python scripts/arm_serial.py inspect
uv run --locked eai-course mapping
uv run --locked eai-course fk --hardware
uv run --locked eai-course vision --preview
```

Vision refuses to select a camera when no local index or explicit
`--camera-index` is provided. Changing the camera index cannot change arm
calibration. Control and CUDA training environments retain separate locks.

Historical candidate tests require explicit `--calibration` and `--home-file`
paths to the matching archived pair. `scripts/arm_verify_candidate.py` now
selects the active calibration/home by default and remains read-only unless
`--enable` is supplied. The old filename is retained for CLI compatibility.
