"""API profiles: offline list/init/check and explicit network acceptance probe."""
import argparse
from importlib.resources import files
import json
import os
from pathlib import Path

import tomli_w

from eai_robot.policy.model_io.contracts import ProviderError
from eai_robot.policy.model_io.providers import ProviderConfig
from .configuration import load_provider


def catalog():
    return json.loads(files(__package__).joinpath("provider_presets.json").read_text(encoding="utf-8"))


def generate(*, preset, model, output, env_file=None, base_url=None, credential_env=None,
             no_auth=False, adapter="chat_completions", execution="managed_api",
             streaming=True, max_tokens=2048, token_limit_parameter=None):
    data = catalog()
    canonical = data["aliases"].get(preset, preset)
    if canonical not in data["presets"]:
        raise ValueError("Unknown preset; use custom with an explicit base_url")
    choice = data["presets"][canonical]
    settings = {"adapter": adapter, "execution": execution, "base_url": base_url or choice.get("base_url"),
                "model": model, "timeout_s": 30., "max_tokens": max_tokens, "streaming": streaming,
                "token_limit_parameter": token_limit_parameter or choice["token_limit_parameter"]}
    if not no_auth:
        settings["credential_env"] = credential_env or choice["credential_env"]
    ProviderConfig(**{k:v for k,v in settings.items() if k != "adapter"})
    if adapter not in {"chat_completions", "responses"}:
        raise ValueError("Unsupported adapter")
    output = Path(output)
    env_file = Path(env_file) if env_file else output.parent / (".env." + output.stem)
    if output.resolve() == env_file.resolve():
        raise ValueError("Config and env file must differ")
    targets = [(output, tomli_w.dumps({"provider": settings}))]
    if not no_auth:
        targets.append((env_file, "# Local secret; fill this value. Never commit this file.\n" + settings["credential_env"] + "=\n"))
    created = []
    try:
        for path, content in targets:
            path.parent.mkdir(parents=True, exist_ok=True)
            # Exclusive create: no accidental overwrite or symlink following.
            fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            created.append(path)
            with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as stream:
                stream.write(content)
                stream.flush()
                os.fsync(stream.fileno())
    except BaseException:
        for path in created:
            path.unlink(missing_ok=True)
        raise
    return {"status": "created", "preset": canonical, "config": str(output),
            "env_file": str(env_file) if not no_auth else None,
            "credential_env": settings.get("credential_env"), "network_called": False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("list", help="List endpoint presets and sources; no model lookup")
    init = commands.add_parser("init", help="Create a TOML profile and blank local dotenv file")
    init.add_argument("--preset", required=True)
    init.add_argument("--model", required=True, help="Explicit model ID from your provider console")
    init.add_argument("--output", type=Path, required=True)
    init.add_argument("--env-file", type=Path)
    init.add_argument("--base-url", help="Override for another region, workspace, gateway or local service")
    auth = init.add_mutually_exclusive_group()
    auth.add_argument("--credential-env")
    auth.add_argument("--no-auth", action="store_true")
    init.add_argument("--adapter", choices=("chat_completions", "responses"), default="chat_completions")
    init.add_argument("--execution", choices=("managed_api", "self_hosted", "local_process"), default="managed_api")
    init.add_argument("--streaming", action=argparse.BooleanOptionalAction, default=True)
    init.add_argument("--max-tokens", type=int, default=2048)
    init.add_argument("--token-limit-parameter", choices=("max_tokens", "max_completion_tokens"))
    check = commands.add_parser("check", help="Check config and selected credential locally; no API request")
    check.add_argument("--provider-config", type=Path, required=True)
    check.add_argument("--env-file", type=Path)
    probe_parser = commands.add_parser("probe", help="Make real API requests: text, conversation, read-only tools; may incur charges")
    probe_parser.add_argument("--provider-config", type=Path, action="append", required=True)
    probe_parser.add_argument("--env-file", type=Path)
    probe_parser.add_argument("--transport", choices=("configured", "json", "sse", "both"), default="both")
    probe_parser.add_argument("--case", choices=("text", "conversation", "tools", "cancel"), action="append", dest="cases")
    probe_parser.add_argument("--strict-tools", action=argparse.BooleanOptionalAction, default=None)
    args = parser.parse_args(argv)
    try:
        if args.command == "list":
            result = catalog()
        elif args.command == "init":
            options = vars(args).copy()
            options.pop("command")
            result = generate(**options)
        elif args.command == "probe":
            from .acceptance import DEFAULT_CASES, probe
            result = probe(args.provider_config, env_file=args.env_file, transport=args.transport,
                           cases=args.cases or DEFAULT_CASES, strict_tools=args.strict_tools)
        else:
            provider = load_provider(args.provider_config, args.env_file)
            try:
                result = {"status": "valid_local_config", "provider": provider.describe(), "network_called": False}
            finally:
                provider.close()
        print(json.dumps(result, ensure_ascii=False, allow_nan=False))
        return 1 if result.get("status") == "failed" else 0
    except ProviderError as exc:
        print(json.dumps({"status": "invalid", "error_code": exc.code, "network_called": False}))
        return 1
    except (OSError, ValueError):
        # Do not print config text, raw parser errors or secret values.
        print(json.dumps({"status": "invalid", "error_code": "configuration_error", "network_called": False}))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
