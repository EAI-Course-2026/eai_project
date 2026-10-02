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
