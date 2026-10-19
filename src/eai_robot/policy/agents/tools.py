"""Injected tool definitions and hooks; no provider or robot-specific conventions."""
from dataclasses import asdict, dataclass
import json
from typing import Callable

from eai_robot.policy.model_io.contracts import ToolCall, ToolDefinition
from .contracts import ToolContext, ToolResult


@dataclass(frozen=True)
class Tool:
    name: str
    description: str
    parameters: dict
    handler: Callable[[dict, ToolContext], ToolResult]
    # Valid interrupt calls delegate to the session cancellation callback.
    interrupt: bool = False

    def validate(self, arguments):
        props = self.parameters["properties"]
        if (not isinstance(arguments, dict) or set(arguments) - set(props)
                or not set(self.parameters.get("required", ())) <= set(arguments)):
            return False
        return all(isinstance(value, str) and 0 < len(value) <= 128
                   and ("enum" not in props[key] or value in props[key]["enum"])
                   for key, value in arguments.items())


class ToolRegistry:
    def __init__(self, tools: list[Tool]):
        self._tools = {tool.name: tool for tool in tools}
        if len(self._tools) != len(tools):
            raise ValueError("Unique tools are required")
        for tool in tools:
            schema = tool.parameters
            if (schema.get("type") != "object" or schema.get("additionalProperties") is not False
                    or not isinstance(schema.get("properties"), dict)
                    or not set(schema.get("required", ())) <= set(schema["properties"])
                    or any(spec.get("type") != "string" for spec in schema["properties"].values())):
                raise ValueError("Only closed object schemas with string properties are supported")

    @property
    def definitions(self) -> tuple[ToolDefinition, ...]:
        return tuple(ToolDefinition(t.name, t.description, json.loads(json.dumps(t.parameters)))
                     for t in self._tools.values())

    def prepare(self, call: ToolCall) -> Tool | ToolResult:
        tool = self._tools.get(call.name)
        if tool is None:
            return ToolResult(False, "rejected", error_code="unknown_tool")
        if not tool.validate(call.arguments):
            return ToolResult(False, "rejected", error_code="invalid_arguments")
        return tool

    def invoke(self, call: ToolCall, context: ToolContext, *, before=None, after=None) -> ToolResult:
        context.check_active()
        tool = self.prepare(call)
        if isinstance(tool, ToolResult):
            return tool
        result = before(tool, dict(call.arguments), context) if before else None
        context.check_active()
        if result is None:
            result = tool.handler(dict(call.arguments), context)
        context.check_active()
        if not isinstance(result, ToolResult):
            raise TypeError("Tool handlers/hooks must return ToolResult")
        json.dumps(asdict(result), allow_nan=False)
        if after:
            after(tool, result, context)
        return result
