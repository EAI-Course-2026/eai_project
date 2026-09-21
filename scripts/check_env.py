"""Read-only software/serial check: does not open cameras or command motors."""
import glob
import importlib
import importlib.metadata
import platform
import shutil
import sys

print(f"Python: {sys.version.split()[0]} ({sys.executable})")
print(f"Platform: {platform.system()} {platform.machine()}")
failed = sys.version_info < (3, 12)
for package, module in [("lerobot", "lerobot"), ("torch", "torch"), ("opencv-python-headless", "cv2"), ("pyserial", "serial"), ("feetech-servo-sdk", "scservo_sdk"), ("av", "av")]:
    try:
        loaded = importlib.import_module(module)
        print(f"OK {package}: {importlib.metadata.version(package)}")
        if module == "torch":
            print(f"  MPS available: {loaded.backends.mps.is_available()}")
            print(f"  CUDA available: {loaded.cuda.is_available()}")
            if loaded.backends.mps.is_available():
                x = loaded.ones(4, device="mps")
                assert (x + x).sum().item() == 8
                print("  MPS tensor operation: OK")
    except Exception as exc:
        failed = True
        print(f"FAIL {package}: {type(exc).__name__}: {exc}")
ffmpeg = shutil.which("ffmpeg")
print(f"FFmpeg: {ffmpeg or 'MISSING'}")
failed = failed or ffmpeg is None
ports = sorted(glob.glob("/dev/cu.usb*") + glob.glob("/dev/cu.wch*"))
print(f"Candidate robot serial ports: {ports or 'none; hardware not verified'}")
print("Camera capture and robot calibration/motion have NOT been tested.")
sys.exit(1 if failed else 0)
