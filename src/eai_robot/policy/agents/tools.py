"""Allowlisted, injected tools. The first schema subset is objects of strings."""
from dataclasses import asdict, dataclass
import json
from typing import Callable

from .contracts import ToolCall, ToolResult


@dataclass(frozen=True)
class Tool:
    name: str
    description: str
    parameters: dict
    handler: Callable[[dict], ToolResult]

    def schema(self) -> dict:
        return {"type": "function", "function": {
            "name": self.name, "description": self.description,
            "parameters": self.parameters,
        }}

    def validate(self, arguments: dict) -> bool:
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
        if len(self._tools) != len(tools) or "stop_motion" not in self._tools:
            raise ValueError("Unique tools and stop_motion are required")
        for tool in tools:
            schema = tool.parameters
            if (schema.get("type") != "object" or schema.get("additionalProperties") is not False
                    or not isinstance(schema.get("properties"), dict)
                    or not set(schema.get("required", ())) <= set(schema["properties"])
                    or any(spec.get("type") != "string" for spec in schema["properties"].values())):
                raise ValueError("Only closed object schemas with string properties are supported")

    @property
    def schemas(self) -> tuple[dict, ...]:
        # Return detached data: neither provider nor caller can mutate the allowlist.
        return tuple(json.loads(json.dumps(tool.schema())) for tool in self._tools.values())

    def invoke(self, call: ToolCall) -> ToolResult:
        tool = self._tools.get(call.name)
        if tool is None:
            return ToolResult(False, "rejected", error_code="unknown_tool")
        if not tool.validate(call.arguments):
            return ToolResult(False, "rejected", error_code="invalid_arguments")
        result = tool.handler(dict(call.arguments))
        if not isinstance(result, ToolResult):
            raise TypeError("Tool handlers must return ToolResult")
        # Refuse objects, NaN and other non-JSON output before sending to a provider.
        json.dumps(asdict(result), allow_nan=False)
        return result
