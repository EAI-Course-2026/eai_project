"""Standard OpenAI function definitions bound to local execution policy."""
from copy import deepcopy
from dataclasses import asdict, dataclass
import json
import re
from typing import Callable

from jsonschema import Draft202012Validator
from referencing import Registry
from referencing.exceptions import NoSuchResource

from eai_robot.policy.model_io.contracts import ToolCall, ToolDefinition
from .contracts import ToolContext, ToolResult


def _no_remote_ref(uri):
    # JSON Schema references may use local $defs; no implicit network retrieval.
    raise NoSuchResource(ref=uri)


@dataclass(frozen=True)
class Tool:
    definition: ToolDefinition
    handler: Callable[[dict, ToolContext], ToolResult]
    # Execution metadata stays local; it is never a vendor schema extension.
    interrupt: bool = False

    @property
    def name(self):
        return self.definition["function"]["name"]


class ToolRegistry:
    def __init__(self, tools: list[Tool]):
        self._tools, self._validators = {}, {}
        for original in tools:
            tool = Tool(deepcopy(original.definition), original.handler, original.interrupt)
            spec = tool.definition
            function = spec.get("function", {})
            name = function.get("name")
            if (set(spec) != {"type", "function"} or spec.get("type") != "function"
                    or not isinstance(name, str) or not re.fullmatch(r"[a-zA-Z0-9_-]{1,64}", name)
                    or set(function) - {"name", "description", "parameters", "strict"}
                    or not isinstance(function.get("description", ""), str)
                    or ("strict" in function and type(function["strict"]) is not bool)):
                raise ValueError("Expected a standard OpenAI function tool definition")
            if name in self._tools:
                raise ValueError("Unique tools are required")
            schema = function.get("parameters", {"type": "object"})
            json.dumps(schema, allow_nan=False)
            Draft202012Validator.check_schema(schema)
            if schema.get("type") != "object":
                raise ValueError("Function parameters must be an object schema")
            self._tools[name] = tool
            self._validators[name] = Draft202012Validator(schema, registry=Registry(retrieve=_no_remote_ref))

    @property
    def definitions(self) -> tuple[ToolDefinition, ...]:
        return tuple(deepcopy(t.definition) for t in self._tools.values())

    def prepare(self, call: ToolCall) -> Tool | ToolResult:
        tool = self._tools.get(call.name)
        if tool is None:
            return ToolResult(False, "rejected", error_code="unknown_tool")
        try:
            json.dumps(call.arguments, allow_nan=False)
            valid = isinstance(call.arguments, dict) and self._validators[call.name].is_valid(call.arguments)
        except Exception:
            valid = False
        if not valid:
            return ToolResult(False, "rejected", error_code="invalid_arguments")
        return tool

    def invoke(self, call: ToolCall, context: ToolContext, *, before=None, after=None) -> ToolResult:
        context.check_active()
        tool = self.prepare(call)
        if isinstance(tool, ToolResult):
            return tool
        arguments = deepcopy(call.arguments)
        result = before(tool, arguments, context) if before else None
        context.check_active()
        if result is None:
            result = tool.handler(deepcopy(call.arguments), context)
        context.check_active()
        if not isinstance(result, ToolResult):
            raise TypeError("Tool handlers/hooks must return ToolResult")
        json.dumps(asdict(result), allow_nan=False)
        if after:
            after(tool, result, context)
        return result
