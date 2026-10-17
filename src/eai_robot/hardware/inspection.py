"""Read-only device checks and explicit best-effort torque release."""
from eai_robot.calibration.results import MODEL_NUMBER


def inspect_hardware(backend):
    check_identity(backend)
    limits = {}
    for sid in range(1, 7):
        low = backend.read(sid, "Min_Position_Limit")
        high = backend.read(sid, "Max_Position_Limit")
        if not 0 <= low < high <= 1023:
            raise RuntimeError(f"ID {sid} 硬件限位 {low}..{high} 不是位置模式；不自动修改 EEPROM")
        limits[sid] = low, high
    return limits


def check_identity(backend):
    for sid in range(1, 7):
        model = backend.read(sid, "Model_Number")
        if model != MODEL_NUMBER or backend.read(sid, "ID") != sid:
            raise RuntimeError(f"ID {sid} 型号/编号不匹配：期望 SCS215 型号码 {MODEL_NUMBER}，实际 {model}")


def release(backend):
    """Try every joint even if an earlier torque-disable fails."""
    failures = []
    for sid in range(1, 7):
        try:
            backend.write(sid, "Torque_Enable", 0)
        except Exception as exc:
            failures.append(f"ID {sid}: {exc}")
    return failures
