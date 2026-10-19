"""Explicit, per-request environment credentials; no discovery or secret storage."""
import os
import re

from .contracts import ProviderError


class EnvironmentCredential:
    def __init__(self, name: str | None):
        if name is not None and (not isinstance(name, str) or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", name)):
            raise ValueError("credential_env must name an environment variable")
        self.name = name

    def resolve(self) -> str:
        value = os.environ.get(self.name, "") if self.name else ""
        if self.name and not value.strip():
            raise ProviderError("missing_credential")
        if "\r" in value or "\n" in value:
            raise ProviderError("invalid_credential")
        return value
