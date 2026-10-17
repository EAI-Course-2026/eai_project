"""Load desktop keyboard hooks only when interactive control starts."""
import importlib
import sys


class LazyKeyboard:
    def __getattr__(self, name):
        try:
            module = importlib.import_module("pynput.keyboard")
        except ImportError as exc:
            raise RuntimeError("Keyboard control needs a desktop session and keyboard access permission") from exc
        return getattr(module, name)


keyboard = LazyKeyboard()


def read_key():
    if sys.platform == "win32":
        import msvcrt
        return msvcrt.getwch().lower()
    if not sys.stdin.isatty():
        value = sys.stdin.read(1)
        return value.lower() if value else "q"
    import termios
    import tty
    fd = sys.stdin.fileno()
    previous = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1).lower()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, previous)


def parse_ratios(text):
    import math
    values = [float(part) for part in text.replace(",", " ").split()]
    if len(values) != 6 or not all(math.isfinite(v) and 0 <= v <= 1 for v in values):
        raise ValueError("请输入六个 0..1 的有限数值，以空格或逗号分隔")
    return values
