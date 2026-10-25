"""Application-side input lifecycle. Speech providers only supply final text."""
from dataclasses import asdict
from importlib.resources import files
from threading import Event, Lock, Thread

from eai_robot.applications.teleoperation.intent import normalize
from eai_robot.policy.agents.loop import AgentSession

from .capabilities import RobotCapabilities


def create_robot_session(provider, skills, *, capabilities=None, hardware_visual=False, **options):
    profile = capabilities or RobotCapabilities()
    prompt_name = "hardware_visual.txt" if hardware_visual else "system.txt"
    prompt = files(__package__).joinpath("prompts", prompt_name).read_text(encoding="utf-8")
    return AgentSession(provider, skills.registry(profile.allowed_tools), system_prompt=prompt,
                        context_provider=profile.context, before_tool=profile.before_tool,
                        on_cancel=lambda: skills.stop({}), **options)


_LOCAL_STOP_UTTERANCES = frozenset({
    "停", "停止", "停下", "停住", "别动", "不要动", "急停", "stop", "halt",
    "立即停止", "马上停止", "现在停止", "请停止", "请停下", "请停住",
    "停止当前任务", "请停止当前任务", "停止本轮", "请停止本轮",
    "停止所有运动", "请停止所有运动", "停止运动", "请停止运动",
})


def is_stop(text: str) -> bool:
    """Recognize a standalone local stop, without stealing compound agent tasks.

    The teleoperation parser deliberately treats a stop word anywhere in a sentence as
    an emergency stop.  Agent prompts are different: a user may ask for a sequence such
    as "detect, track, then stop tracking".  Only explicit standalone stop utterances
    bypass the model; scoped requests such as ``stop tracking`` remain tool-call tasks.
    """
    return normalize(text) in _LOCAL_STOP_UTTERANCES


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
