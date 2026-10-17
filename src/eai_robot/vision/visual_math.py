"""Compatibility alias; new code uses ``eai_robot.control.visual_math``."""
import importlib as _importlib
import sys as _sys

_impl = _importlib.import_module("eai_robot.control.visual_math")
if __name__ == "__main__" and hasattr(_impl, "main"):
    raise SystemExit(_impl.main())
_sys.modules[__name__] = _impl
