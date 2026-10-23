"""Explicit API profile loading, shared by the application and acceptance CLI."""
from dataclasses import replace
from pathlib import Path
import tomllib

from dotenv import dotenv_values

from eai_robot.policy.model_io.providers import ChatCompletionsProvider, ProviderConfig
from eai_robot.policy.model_io.responses import ResponsesProvider


def load_provider(path, env_file=None, *, streaming=None):
    with Path(path).open("rb") as stream:
        config = tomllib.load(stream)
    if set(config) != {"provider"} or not isinstance(config["provider"], dict):
        raise ValueError("Expected only a [provider] section")
    settings = dict(config["provider"])
    provider_class = {"chat_completions": ChatCompletionsProvider, "responses": ResponsesProvider}.get(settings.pop("adapter", None))
    if provider_class is None:
        raise ValueError("Unsupported provider adapter")
    try:
        spec = ProviderConfig(**settings)
    except TypeError:
        raise ValueError("Invalid provider fields; store credentials in the named environment variable") from None
    if streaming is not None:
        # Acceptance can test JSON and SSE independently. stream_options is a
        # stream-only SDK argument; all other model/vendor settings stay intact.
        options = dict(spec.model_options)
        if not streaming:
            options.pop("stream_options", None)
        spec = replace(spec, streaming=streaming, model_options=options)
    values = None
    if env_file is not None:
        env_path = Path(env_file)
        if not env_path.is_file():
            raise ValueError("Explicit environment file does not exist")
        values = dotenv_values(env_path, interpolate=False, encoding="utf-8")
    return provider_class(spec, credential_values=values)
