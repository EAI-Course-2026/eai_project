"""Application-side input lifecycle. Speech providers only supply final text."""
from dataclasses import asdict
from importlib.resources import files
from threading import Event, Lock, Thread

from eai_robot.applications.teleoperation.intent import STOP, parse
from eai_robot.policy.agents.loop import AgentSession

from .capabilities import RobotCapabilities


def create_robot_session(provider, skills, *, capabilities=None, **options):
    profile = capabilities or RobotCapabilities()
    prompt = files(__package__).joinpath("prompts/system.txt").read_text(encoding="utf-8")
    return AgentSession(provider, skills.registry(profile.allowed_tools), system_prompt=prompt,
                        context_provider=profile.context, before_tool=profile.before_tool,
                        on_cancel=lambda: skills.stop({}), **options)


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
                result = session.cancel()
                if not result.ok:
                    failed.append(True)
                output({"status": "cancelled" if result.ok else "failed", "stop": asdict(result)})
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
        session.cancel()
        raise
    finally:
        if worker is not None:
            worker.join()
    return 1 if failed else 0
