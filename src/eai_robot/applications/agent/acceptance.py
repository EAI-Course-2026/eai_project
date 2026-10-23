"""Opt-in, bounded chat API acceptance through the production SDK and harness.

Only generated text and a read-only lookup tool are sent. No devices, user
conversations or robot skills are loaded. Reports contain metrics, not payloads.
"""
from datetime import datetime, timezone
import hashlib
from pathlib import Path
import time
import uuid

from eai_robot.policy.agents.contracts import ToolResult
from eai_robot.policy.agents.loop import AgentSession
from eai_robot.policy.agents.tools import Tool, ToolRegistry
from eai_robot.policy.model_io.contracts import ProviderError

from .configuration import load_provider

CASES = ("text", "conversation", "tools", "cancel")
DEFAULT_CASES = CASES[:3]


def run_case(provider, case, *, strict_tools=None):
    """Own and close one provider/session. Exercise, never fake, the selected API."""
    if case not in CASES:
        raise ValueError("Unknown acceptance case")
    if case == "cancel" and not provider.config.streaming:
        provider.close()
        return {"case": case, "status": "skipped", "error_code": "requires_sse"}
    key, proof = uuid.uuid4().hex, "EAI_RESULT_" + uuid.uuid4().hex
    started, first_text, cancelled_at = time.monotonic(), None, None
    requests, deltas, tool_attempts, invoked, usage = 0, 0, 0, [], {}
    session = None

    def emit(event):
        nonlocal first_text, cancelled_at, requests, deltas, tool_attempts
        if event["type"] == "model_start":
            requests += 1
        elif event["type"] == "model_end":
            # Flat numeric usage is comparable; vendor nested breakdowns vary.
            for name, value in event["data"]["usage"].items():
                if type(value) is int:
                    usage[name] = usage.get(name, 0) + value
        elif event["type"] == "model_delta":
            deltas += 1
            if first_text is None:
                first_text = time.monotonic()
                if case == "cancel":
                    cancelled_at = time.monotonic()
                    session.cancel()
        elif event["type"] == "tool_start":
            tool_attempts += 1

    def lookup(arguments, context):
        context.check_active()
        if arguments["key"] != key or invoked:
            return ToolResult(False, "rejected", error_code="unexpected_probe_call")
        invoked.append(arguments["key"])
        # This proof is never in the prompt: the model must consume tool output.
        return ToolResult(True, "completed", {"value": proof})

    function = {"name": "lookup_probe", "description": "Read-only acceptance lookup; no robot or hardware access.",
                "parameters": {"type": "object", "properties": {"key": {"type": "string"}},
                               "required": ["key"], "additionalProperties": False}}
    if strict_tools is not None:
        function["strict"] = strict_tools
    tools = ToolRegistry([Tool({"type": "function", "function": function}, lookup)] if case == "tools" else [])
    session = AgentSession(provider, tools, emit=emit, max_rounds=3, max_calls=1,
                           timeout_s=min(300., provider.config.timeout_s * 3),
                           system_prompt="Follow the acceptance instructions exactly. Use a tool only when requested. Do not add explanations or formatting.")
    status, code = "failed", None
    try:
        if case == "cancel":
            result = session.run_turn("Write a long numbered list of 100 common household objects, one per line.")
            code = result.error_code
            if result.status == "cancelled" and cancelled_at is not None:
                status = "passed"
            else:
                code = code or "no_stream_cancellation"
        else:
            prompt = (f"Call lookup_probe exactly once with key {key}. Then reply with exactly the data.value from its result."
                      if case == "tools" else f"Reply with exactly this text: {proof}")
            result = session.run_turn(prompt)
            code = result.error_code
            if result.status != "completed":
                code = code or result.status
            elif case == "tools" and invoked != [key]:
                code = "tool_not_used"
            elif result.text.strip() != proof:
                code = "unexpected_answer"
            else:
                status = "passed"
                if case in {"conversation", "tools"}:
                    result = session.run_turn("Repeat exactly your previous final answer. Do not call any tool.")
                    if result.status != "completed" or result.text.strip() != proof:
                        status, code = "failed", result.error_code or "conversation_mismatch"
            if status == "passed" and case == "tools" and tool_attempts != 1:
                status, code = "failed", "unexpected_tool_call"
        elapsed = time.monotonic()
        return {"case": case, "status": status, "error_code": code,
                "duration_ms": round((elapsed - started) * 1000, 2),
                "first_text_ms": round((first_text - started) * 1000, 2) if first_text is not None else None,
                "cancel_wait_ms": round((elapsed - cancelled_at) * 1000, 2) if cancelled_at is not None else None,
                "model_requests": requests, "text_deltas": deltas, "tool_calls": tool_attempts,
                "successful_lookups": len(invoked), "usage": usage}
    finally:
        session.close()


def probe(paths, *, env_file=None, transport="both", cases=DEFAULT_CASES, strict_tools=None):
    """Run each selected case in isolation; failures never suppress other rows.

    No retries, automatic discovery, model selection, endpoint fallback or writes.
    The caller explicitly opts in to network calls via the `probe` command.
    """
    if (transport not in {"configured", "json", "sse", "both"} or not cases or set(cases) - set(CASES)
            or (strict_tools is not None and type(strict_tools) is not bool)):
        raise ValueError("Invalid acceptance selection")
    modes = {"configured": (None,), "json": (False,), "sse": (True,), "both": (False, True)}[transport]
    rows = []
    for index, path in enumerate(paths, 1):
        for streaming in modes:
            for case in dict.fromkeys(cases):
                provider = None
                row = {"profile": index, "transport": "configured" if streaming is None else "sse" if streaming else "json", "case": case}
                try:
                    row["profile_sha256"] = hashlib.sha256(Path(path).read_bytes()).hexdigest()
                    provider = load_provider(path, env_file, streaming=streaming)
                    row["provider"] = provider.describe()
                    row.update(run_case(provider, case, strict_tools=strict_tools))
                except ProviderError as exc:
                    row.update(status="failed", error_code=exc.code)
                except (OSError, ValueError):
                    row.update(status="failed", error_code="configuration_error")
                finally:
                    if provider is not None:
                        provider.close()
                rows.append(row)
    passed = sum(r["status"] == "passed" for r in rows)
    failed = sum(r["status"] == "failed" for r in rows)
    return {"schema_version": 1, "suite": "chat_api_acceptance", "at": datetime.now(timezone.utc).isoformat(),
            "status": "passed" if passed and not failed else "failed", "passed": passed, "failed": failed,
            "skipped": len(rows) - passed - failed, "hardware_connected": False,
            "strict_tools": strict_tools, "scope": "selected_profiles_and_cases_only", "results": rows}
