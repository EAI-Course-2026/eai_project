"""Application-side input lifecycle. Speech providers only supply final text."""
from dataclasses import asdict
from threading import Event, Lock, Thread

from eai_robot.applications.teleoperation.intent import STOP, parse


def is_stop(text: str) -> bool:
    result = parse(text)
    return result.intent is not None and result.intent.command == STOP


def run_stdin(stream, session, emit):
    """Read while inference runs, so local stop never waits for the cloud.

    One ordinary turn at a time, no pending transcript queue or automatic replay.
    EOF joins the current bounded turn; a provider error closes this input session.
    """
    worker = None
    failed = []
    output_lock = Lock()
    def output(value):
        with output_lock:
            emit(value)
    def run(text, ready):
        result = session.run_turn(text, ready=ready)
        if result.status == "failed":
            failed.append(True)
        output(asdict(result))
    try:
        for line in stream:
            text = line.strip()
            if not text:
                continue
            if failed:
                break
            if is_stop(text):
                result = session.stop()
                output({"status": "cancelled", "stop": asdict(result)})
            elif worker is not None and worker.is_alive():
                output({"status": "rejected", "error_code": "busy"})
            else:
                ready = Event()
                worker = Thread(target=run, args=(text, ready), name="agent-turn", daemon=True)
                worker.start()
                # Install the turn cancellation token before reading the next line.
                ready.wait()
    except BaseException:
        # Ctrl-C/input failures cancel before joining, rather than waiting for HTTP.
        session.stop()
        raise
    finally:
        if worker is not None:
            worker.join()
    return 1 if failed else 0
