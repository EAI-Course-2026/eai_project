"""A local, fail-closed session gate. Remote clocks never grant a motion lease."""
from dataclasses import dataclass
import math
import time
import uuid
import numpy as np

from .contracts import finite_positive, vector


@dataclass(frozen=True)
class Budgets:
    max_observation_age_s: float = 0.25
    max_acquisition_skew_s: float = 0.1
    response_timeout_s: float = 0.25
    action_ttl_s: float = 0.5
    max_step: float = 1.0
    max_tracking_error: float = 3.0

    def __post_init__(self):
        for name, value in vars(self).items():
            finite_positive(value, name)


@dataclass(frozen=True)
class Ticket:
    session: str
    sequence: int
    manifest: str
    issued_at: float
    observation_at: float
    reply_by: float
    expires_at: float


class Gate:
    def __init__(self, manifest, budgets=Budgets(), clock=time.perf_counter):
        self.manifest, self.budgets, self.clock = manifest, budgets, clock
        self.active = False
        self.session = ""
        self.sequence = 0
        self.pending = None
        self.queue = []
        self.ticket = None
        self.previous = None
        self.started_at = None
        self.reason = "not armed"

    def arm(self, state):
        self.previous = vector(state)
        self.session = str(uuid.uuid4())
        self.sequence = 0
        self.pending = self.ticket = None
        self.queue = []
        self.started_at = None
        self.active = True
        self.reason = ""

    def stop(self, reason="operator stop"):
        self.active = False
        self.pending = self.ticket = None
        self.queue = []
        self.reason = reason

    def _reject(self, reason):
        self.stop(reason)
        raise RuntimeError(reason)

    def request(self, observation):
        if not self.active or self.pending or self.queue:
            self._reject("Request requires an active empty queue and no in-flight request")
        now = self.clock()
        try:
            oldest = observation.validate(self.manifest, now=now, max_age_s=self.budgets.max_observation_age_s, max_skew_s=self.budgets.max_acquisition_skew_s)
        except ValueError as exc:
            self._reject(str(exc))
        self.sequence += 1
        self.pending = Ticket(self.session, self.sequence, self.manifest.fingerprint, now, oldest, now + self.budgets.response_timeout_s, oldest + self.budgets.action_ttl_s)
        return self.pending

    def accept(self, ticket, actions):
        now = self.clock()
        if not self.active or ticket != self.pending or now >= ticket.reply_by or now >= ticket.expires_at:
            self._reject("Unsolicited, out-of-order or expired response")
        try:
            raw = np.asarray(actions)
            if raw.dtype.kind not in "fiu" or raw.ndim != 2 or raw.shape[1] != 6 or not 1 <= len(raw) <= self.manifest.data["max_chunk_steps"]:
                raise ValueError("Invalid or oversized action chunk")
            array = np.stack([vector(row) for row in raw])
            deltas = np.diff(np.vstack([self.previous, array]), axis=0)
            if np.any(np.abs(deltas) > self.budgets.max_step):
                raise ValueError("First-step/chunk discontinuity exceeds step budget")
            hz = self.manifest.data["action_hz"]
            if now + (len(array) - 1) / hz >= ticket.expires_at:
                raise ValueError("Action chunk extends beyond its local deadline")
        except ValueError as exc:
            self._reject(str(exc))
        self.queue = list(array)
        self.ticket = ticket
        self.pending = None
        self.started_at = now
        self.index = 0

    def next_action(self, feedback, *, torque_ok, alarm=False):
        now = self.clock()
        if not self.active or not self.queue or self.ticket is None or now >= self.ticket.expires_at:
            self._reject("No live action lease")
        if type(torque_ok) is not bool or not torque_ok or type(alarm) is not bool or alarm:
            self._reject("Torque loss or motor alarm")
        try:
            feedback = vector(feedback)
        except ValueError as exc:
            self._reject(str(exc))
        if np.any(np.abs(feedback - self.previous) > self.budgets.max_tracking_error):
            self._reject("Feedback tracking error exceeds budget")
        due = self.started_at + self.index / self.manifest.data["action_hz"]
        if now < due:
            return None
        # A stalled local loop must not burst-send catch-up actions.
        if now - due >= 1 / self.manifest.data["action_hz"]:
            self._reject("Missed action schedule; discard chunk")
        action = self.queue.pop(0)
        if np.any(np.abs(action - feedback) > self.budgets.max_step):
            self._reject("Requested step from current feedback exceeds budget")
        self.previous = action.copy()
        self.index += 1
        return action
