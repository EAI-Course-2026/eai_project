"""Explicit credentials: process environment or selected dotenv values; no discovery."""
import os
import re

from .contracts import ProviderError


class EnvironmentCredential:
    def __init__(self, name: str | None, values=None):
        if name is not None and (not isinstance(name, str) or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", name)):
            raise ValueError("credential_env must name an environment variable")
        self.name = name
        self._values = {name: values[name]} if name and values and name in values else {}

    def resolve(self) -> str:
        value = os.environ.get(self.name, self._values.get(self.name, "")) if self.name else ""
        value = value or ""
        if self.name and not value.strip():
            raise ProviderError("missing_credential")
        if "\r" in value or "\n" in value:
            raise ProviderError("invalid_credential")
        return value
