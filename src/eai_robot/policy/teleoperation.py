"""Follower-only operator collection: bounded joint jogging with a keyboard input lease."""
import threading
import time
import numpy as np

from .contracts import KEYS, vector
from .safety import Gate

JOG_KEYS = (("q", "a"), ("w", "s"), ("e", "d"), ("r", "f"), ("t", "g"), ("y", "h"))


class KeyboardJog:
    def __init__(self, *, step, input_lease_s=0.3):
        self.step = step
        self.input_lease_s = input_lease_s
        self.keys = set()
        self.updated_at = 0.0
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.listener = None

    def start(self):
        from pynput import keyboard

        def press(key):
            if key == keyboard.Key.esc:
                self.stop_event.set()
                return False
            char = getattr(key, "char", None)
            if char and char.lower() in {c for pair in JOG_KEYS for c in pair}:
                with self.lock:
                    self.keys.add(char.lower())
                    self.updated_at = time.perf_counter()

        def release(key):
            char = getattr(key, "char", None)
            if char:
                with self.lock:
                    self.keys.discard(char.lower())
                    self.updated_at = time.perf_counter()

        self.listener = keyboard.Listener(on_press=press, on_release=release)
        self.listener.start()

    def action(self, previous):
        with self.lock:
            if time.perf_counter() - self.updated_at > self.input_lease_s:
                self.keys.clear()
            keys = self.keys.copy()
        action = vector(previous).copy()
        for i, (positive, negative) in enumerate(JOG_KEYS):
            action[i] += self.step * ((positive in keys) - (negative in keys))
        # Reject range violations through the gate; never clip labels to hide rejection.
        return action

    def close(self):
        if self.listener:
            self.listener.stop()
            self.listener.join(timeout=1)


def record_demonstration(owner, manifest, source, journal, budgets, task, duration_s, *, clock=time.perf_counter, sleep=time.sleep):
    """Runs only on explicit collection; source never owns or touches the hardware."""
    gate = Gate(manifest, budgets, clock=clock)
    gate.arm(owner.feedback())
    started = clock()
    index = 0
    try:
        while clock() - started < duration_s and not source.stop_event.is_set():
            due = started + index / manifest.data["action_hz"]
            if due - started >= duration_s:
                break
            if clock() < due:
                sleep(due - clock())
            if clock() - due >= 1 / manifest.data["action_hz"]:
                raise RuntimeError("Operator recording missed cadence; discard incomplete episode")
            obs = owner.observe(task)
            ticket = gate.request(obs)
            action = source.action(gate.previous)
            gate.accept(ticket, np.asarray(action)[None])
            state = owner.feedback()
            target = gate.next_action(state, **owner.health())
            if target is None:
                raise RuntimeError("Unexpected unscheduled operator action")

            def before_write():
                if source.stop_event.is_set() or not gate.active or clock() >= ticket.expires_at or clock() - started >= duration_s:
                    raise RuntimeError("Operator stop or expired collection action")

            sent = owner.send(target, before_write=before_write)
            feedback = owner.feedback()
            journal.append(obs, requested=target, sent=sent, feedback=feedback, feedback_at=clock())
            index += 1
        return index
    except BaseException as exc:
        gate.stop(str(exc))
        raise
    finally:
        gate.stop("operator collection ended")
