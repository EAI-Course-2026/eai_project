"""Bounded JSON/RGB transport over authenticated loopback HTTP (use an SSH tunnel)."""
import base64
from dataclasses import asdict
import hmac
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import math
import os
import time
from urllib.parse import urlsplit
from urllib.request import Request, urlopen

import numpy as np

from .observations import Observation

MAX_BODY = 32 * 1024 * 1024


def token_from_env():
    token = os.environ.get("EAI_POLICY_TOKEN", "")
    if len(token) < 32:
        raise ValueError("Set EAI_POLICY_TOKEN to a shared secret of at least 32 characters")
    return token


def encode_observation(obs):
    obs_images = {}
    for name, array in obs.images.items():
        obs_images[name] = {"shape": list(array.shape), "rgb": base64.b64encode(array.tobytes()).decode("ascii")}
    return {"state": obs.state.tolist(), "images": obs_images, "task": obs.task}


def decode_observation(data, manifest):
    images = {}
    if set(data["images"]) != {s["source"] for s in manifest.data["cameras"].values()}:
        raise ValueError("Camera source mismatch")
    for name, item in data["images"].items():
        shape = item["shape"]
        if not isinstance(shape, list) or len(shape) != 3 or shape[-1] != 3 or any(type(v) is not int or not 1 <= v <= 4096 for v in shape):
            raise ValueError("Invalid RGB shape")
        rgb = base64.b64decode(item["rgb"], validate=True)
        if len(rgb) != math.prod(shape):
            raise ValueError("RGB byte count mismatch")
        images[name] = np.frombuffer(rgb, dtype=np.uint8).reshape(shape).copy()
    now = time.perf_counter()
    obs = Observation(data["state"], images, data["task"], now, now, {name: now for name in images})
    obs.validate(manifest)
    return obs


class RemoteRuntime:
    def __init__(self, manifest, url, *, timeout=1.0, token=None):
        parsed = urlsplit(url)
        if parsed.scheme != "http" or parsed.hostname not in {"localhost", "127.0.0.1", "::1"} or parsed.username or parsed.password or parsed.query or parsed.fragment or parsed.path not in {"", "/"}:
            raise ValueError("Only loopback HTTP endpoints are supported; use an SSH tunnel")
        self.manifest, self.url, self.timeout = manifest, url.rstrip("/"), timeout
        self.token = token or token_from_env()
        self.session = None
        self.sequence = 0

    def predict(self, observation, ticket=None):
        import uuid
        observation.validate(self.manifest)
        if ticket is None:
            # Standalone dry-run session, separate from any motion lease.
            self.session = self.session or str(uuid.uuid4())
            self.sequence += 1
            identity = {"session": self.session, "sequence": self.sequence, "manifest": self.manifest.fingerprint}
        else:
            identity = {key: getattr(ticket, key) for key in ("session", "sequence", "manifest")}
        body = json.dumps({**identity, "observation": encode_observation(observation)}, allow_nan=False).encode()
        if len(body) > MAX_BODY:
            raise ValueError("Request exceeds bounded transport size")
        request = Request(self.url + "/predict", data=body, headers={"Content-Type": "application/json", "Authorization": "Bearer " + self.token}, method="POST")
        started = time.perf_counter()
        # Never forward clocks/deadlines. The local gate checks the original ticket.
        with urlopen(request, timeout=self.timeout) as response:
            raw = response.read(MAX_BODY + 1)
        if len(raw) > MAX_BODY:
            raise ValueError("Response too large")
        data = json.loads(raw)
        if any(data.get(key) != value for key, value in identity.items()):
            raise ValueError("Response session/sequence/manifest mismatch")
        actions = np.asarray(data["actions"])
        if actions.dtype.kind not in "fiu" or actions.ndim != 2 or actions.shape[1] != 6 or not np.isfinite(actions).all() or not 1 <= len(actions) <= self.manifest.data["max_chunk_steps"]:
            raise ValueError("Invalid remote action chunk")
        return actions, time.perf_counter() - started


def policy_server(runtime, port, token=None):
    token = token or token_from_env()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass  # Avoid logging authorization/request content.

        def reply(self, status, data):
            body = json.dumps(data, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            self.connection.settimeout(5)
            if not hmac.compare_digest(self.headers.get("Authorization", ""), "Bearer " + token):
                self.reply(401, {"error": "unauthorized"})
                return
            if self.path != "/predict":
                self.reply(404, {"error": "unknown endpoint"})
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= MAX_BODY or self.headers.get("Transfer-Encoding"):
                    raise ValueError("Invalid request size")
                data = json.loads(self.rfile.read(length))
                if data.get("manifest") != runtime.manifest.fingerprint:
                    raise ValueError("Manifest fingerprint mismatch")
                session, sequence = data.get("session"), data.get("sequence")
                if not isinstance(session, str) or not 1 <= len(session) <= 64 or type(sequence) is not int or sequence < 1:
                    raise ValueError("Invalid request identity")
                # A single synchronous server serializes inference. No persistent policy queue
                # or robot object exists here, so clients cannot take over hardware ownership.
                actions, elapsed = runtime.predict(decode_observation(data["observation"], runtime.manifest))
                self.reply(200, {"session": session, "sequence": sequence, "manifest": data["manifest"], "actions": actions.tolist(), "inference_s": elapsed})
            except (ValueError, KeyError, TypeError) as exc:
                self.reply(400, {"error": str(exc)})
            except Exception:
                self.reply(500, {"error": "inference failed; inspect server terminal"})
                raise

    return HTTPServer(("127.0.0.1", port), Handler)
