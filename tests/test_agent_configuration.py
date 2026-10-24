"""Provider presets, explicit dotenv isolation and offline configuration workflows."""
from contextlib import redirect_stdout, redirect_stderr
from threading import Event
import io
import json
import os
from pathlib import Path
import stat
import subprocess
import sys
import tempfile
import tomllib
import unittest
from unittest.mock import patch

from eai_robot.applications.agent.cli import load_provider
from eai_robot.applications.agent.configure import catalog, generate, main
from eai_robot.policy.agents.loop import AgentSession
from eai_robot.policy.agents.tools import ToolRegistry
from eai_robot.applications.agent.skills import DryRunSkills
from eai_robot.policy.model_io.contracts import ToolCall, ProviderError
from eai_robot.policy.model_io.providers import ChatCompletionsProvider, ProviderConfig
from test_agent import answer, service, request
from test_agent_layers import packet, streaming_service


class ConfigurationTests(unittest.TestCase):
    def test_each_preset_generates_valid_independent_config_without_selecting_model(self):
        data = catalog()
        with tempfile.TemporaryDirectory() as folder:
            for name in [*data["presets"], *data["aliases"]]:
                with self.subTest(name=name):
                    out = Path(folder) / name / 'provider.local.toml'
                    result = generate(preset=name, model='operator-selected', output=out,
                                      **({'base_url': 'https://small.example/v1'} if name == 'custom' else {}))
                    config = tomllib.loads(out.read_text())["provider"]
                    self.assertEqual(config["model"], 'operator-selected')
                    self.assertEqual(config["adapter"], 'chat_completions')
                    self.assertNotIn('api_key', config)
                    self.assertFalse(result["network_called"])
                    self.assertIn(config["credential_env"] + '=\n', Path(result["env_file"]).read_text())
                    ProviderConfig(**{k:v for k,v in config.items() if k != "adapter"})
                    if os.name != 'nt':
                        self.assertEqual(stat.S_IMODE(Path(result["env_file"]).stat().st_mode), 0o600)

    def test_explicit_env_file_selects_only_credential_and_environment_has_priority(self):
        with tempfile.TemporaryDirectory() as folder, service(lambda _: (200, answer())) as (url, _, headers):
            out, env = Path(folder)/'custom.local.toml', Path(folder)/'.env.custom'
            generate(preset='custom', model='fixture', output=out, env_file=env, base_url=url.removesuffix('/chat/completions'),
                     execution='self_hosted', streaming=False, credential_env='CONFIG_TEST_KEY')
            env.write_text('CONFIG_TEST_KEY=file-secret\nOTHER_KEY=must-not-load\n', encoding='utf-8')
            with patch.dict(os.environ, {}, clear=True):
                provider = load_provider(out, env)
                self.addCleanup(provider.close)
                provider.complete(request(), cancel=Event())
                self.assertNotIn('OTHER_KEY', os.environ)
                self.assertNotIn('CONFIG_TEST_KEY', os.environ)
                os.environ['CONFIG_TEST_KEY'] = 'environment-secret'
                provider.complete(request(), cancel=Event())
                self.assertEqual([h['Authorization'] for h in headers], ['Bearer file-secret', 'Bearer environment-secret'])
                del os.environ['CONFIG_TEST_KEY']
                with self.assertRaisesRegex(ProviderError, 'missing_credential'):
                    load_provider(out)

    def test_dotenv_interpolation_is_disabled_and_files_are_not_discovered(self):
        with tempfile.TemporaryDirectory() as folder:
            out, env = Path(folder)/'custom.local.toml', Path(folder)/'.env.custom'
            generate(preset='custom', model='fixture', output=out, env_file=env, base_url='https://example.invalid/v1', credential_env='CONFIG_TEST_KEY')
            env.write_text('CONFIG_TEST_KEY=${UNRELATED_SECRET}\n', encoding='utf-8')
            with patch.dict(os.environ, {'UNRELATED_SECRET': 'DO_NOT_EXPAND'}, clear=True):
                provider = load_provider(out, env)
                self.addCleanup(provider.close)
                self.assertEqual(provider._credential.resolve(), '${UNRELATED_SECRET}')
                with self.assertRaisesRegex(ProviderError, 'missing_credential'):
                    load_provider(out)

    def test_init_never_overwrites_and_rolls_back_new_file_if_secret_file_exists(self):
        with tempfile.TemporaryDirectory() as folder:
            out, env = Path(folder)/'custom.local.toml', Path(folder)/'.env.custom'
            env.write_text('KEEP_SECRET=original')
            with self.assertRaises(FileExistsError):
                generate(preset='deepseek', model='fixture', output=out, env_file=env)
            self.assertEqual(env.read_text(), 'KEEP_SECRET=original')
            self.assertFalse(out.exists())
            out.write_text('KEEP_CONFIG=original')
            with self.assertRaises(FileExistsError):
                generate(preset='kimi', model='fixture', output=out, env_file=Path(folder)/'.env.other')
            self.assertEqual(out.read_text(), 'KEEP_CONFIG=original')

    def test_check_is_offline_and_redacts_parse_errors_and_missing_keys(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder)/'provider.local.toml'
            result = generate(preset='minimax', model='fixture', output=out)
            env = Path(result['env_file'])
            env.write_text('MINIMAX_API_KEY=LOCAL_TEST_SECRET\n')
            with patch.dict(os.environ, {}, clear=True), patch('httpx.Client.send', side_effect=AssertionError('Network forbidden')):
                stdout = io.StringIO()
                with redirect_stdout(stdout):
                    self.assertEqual(main(['check', '--provider-config', str(out), '--env-file', str(env)]), 0)
                self.assertFalse(json.loads(stdout.getvalue())['network_called'])
                self.assertNotIn('LOCAL_TEST_SECRET', stdout.getvalue())
                out.write_text('api_key="UNCLOSED_PRIVATE_SECRET')
                stdout, stderr = io.StringIO(), io.StringIO()
                with redirect_stdout(stdout), redirect_stderr(stderr):
                    self.assertEqual(main(['check', '--provider-config', str(out), '--env-file', str(env)]), 1)
                self.assertNotIn('UNCLOSED_PRIVATE_SECRET', stdout.getvalue()+stderr.getvalue())

    def test_custom_can_select_responses_or_local_no_auth_without_new_provider_class(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder)/'responses.local.toml'
            generate(preset='custom', model='fixture', output=out, base_url='http://127.0.0.1:1234/v1',
                     execution='local_process', adapter='responses', no_auth=True)
            provider = load_provider(out)
            self.addCleanup(provider.close)
            self.assertEqual(provider.describe()['adapter'], 'responses')
            self.assertEqual(len(list(Path(folder).iterdir())), 1)
        for field in ('tools', 'messages', 'stream', 'model', 'max_tokens', 'previous_response_id', 'store'):
            with self.subTest(field=field), self.assertRaises(ValueError):
                ProviderConfig(base_url='https://example.invalid/v1', model='fixture', extra_body={field: 'override'})

    def test_vendor_extra_body_is_forwarded_but_cannot_replace_control_fields(self):
        with service(lambda _: (200, answer())) as (url, requests, _):
            provider = ChatCompletionsProvider(ProviderConfig(url, 'fixture', 'self_hosted',
                                                             extra_body={'enable_thinking': False, 'thinking': {'type': 'disabled'}}))
            self.addCleanup(provider.close)
            provider.complete(request(), cancel=Event())
            self.assertFalse(requests[0]['enable_thinking'])
            self.assertEqual(requests[0]['thinking'], {'type': 'disabled'})
            self.assertEqual(requests[0]['model'], 'fixture')

    def test_chat_reasoning_extension_is_preserved_in_tool_continuation_json_and_sse(self):
        # Several compatible vendors require this field to be returned with tool results.
        for streaming in (False, True):
            skills = DryRunSkills({'stand': [.5]*6})
            body = answer(calls=(ToolCall('state', 'get_robot_state', {}),))
            body['choices'][0]['message']['reasoning_content'] = 'opaque-vendor-reasoning'
            if streaming:
                raw = packet({'role':'assistant', 'reasoning_content':'opaque-vendor-', 'tool_calls':[
                    {'index':0, 'id':'state', 'type':'function', 'function':{'name':'get_robot_state','arguments':'{}'}}]})
                raw += packet({'reasoning_content':'reasoning'}) + packet(reason='tool_calls') + b'data: [DONE]\n\n'
                context = streaming_service(lambda data,index: ([raw],None) if index == 1 else ([packet({'content':'Done'})+packet(reason='stop')+b'data: [DONE]\n\n'],None))
            else:
                context = service(lambda data: (200, body if not any(m['role']=='tool' for m in data['messages']) else answer()))
            with self.subTest(streaming=streaming), context as result:
                url, requests = result[:2]
                provider = ChatCompletionsProvider(ProviderConfig(url, 'fixture', 'self_hosted', streaming=streaming))
                session = AgentSession(provider, skills.registry())
                self.addCleanup(session.close)
                self.assertEqual(session.run_turn('Fixture').status, 'completed')
                assistant = next(m for m in requests[1]['messages'] if m['role']=='assistant')
                self.assertEqual(assistant['reasoning_content'], 'opaque-vendor-reasoning')

    def test_final_reasoning_metadata_survives_the_next_user_turn(self):
        body = answer('Done')
        body['choices'][0]['message']['reasoning_content'] = 'opaque-final-reasoning'
        with service(lambda _: (200, body)) as (url, requests, _):
            provider = ChatCompletionsProvider(ProviderConfig(url, 'fixture', 'self_hosted'))
            session = AgentSession(provider, ToolRegistry([]))
            self.addCleanup(session.close)
            self.assertEqual(session.run_turn('First turn').status, 'completed')
            self.assertEqual(session.run_turn('Second turn').status, 'completed')
            previous = next(m for m in requests[1]['messages'] if m['role'] == 'assistant')
            self.assertEqual(previous['reasoning_content'], 'opaque-final-reasoning')
            self.assertNotIn('tool_calls', previous)

    def test_agent_cli_uses_explicit_dotenv_without_echoing_key(self):
        def responder(data):
            return 200, answer("Fixture")
        with tempfile.TemporaryDirectory() as folder, service(responder) as (url, requests, headers):
            root = Path(__file__).resolve().parents[1]
            out = Path(folder)/'provider.local.toml'
            result = generate(preset='custom', model='fixture', output=out, base_url=url.removesuffix('/chat/completions'),
                              execution='self_hosted', streaming=False, credential_env='CLI_DOTENV_KEY')
            env = Path(result['env_file'])
            env.write_text('CLI_DOTENV_KEY=ONLY_LOCAL_TEST_VALUE\n')
            process_env = dict(os.environ, PYTHONPATH=str(root/'src'))
            process_env.pop('CLI_DOTENV_KEY', None)
            proc = subprocess.run([sys.executable, '-m', 'eai_robot.applications.agent.cli', '--provider-config', str(out),
                                   '--env-file', str(env), '--text', 'Fixture'], cwd=folder, env=process_env,
                                  text=True, capture_output=True, timeout=15)
            self.assertEqual(proc.returncode, 0, proc.stderr)
            self.assertEqual(headers[0]['Authorization'], 'Bearer ONLY_LOCAL_TEST_VALUE')
            self.assertNotIn('ONLY_LOCAL_TEST_VALUE', proc.stdout+proc.stderr)
            self.assertEqual(requests[0]['model'], 'fixture')

    def test_server_strict_is_explicit_but_local_validation_always_runs(self):
        from test_agent import tool_context
        for strict in (None, True, False):
            skills = DryRunSkills({'stand': [.5]*6}, strict=strict)
            registry = skills.registry()
            definitions = registry.definitions
            for spec in definitions:
                if strict is None:
                    self.assertNotIn('strict', spec['function'])
                else:
                    self.assertIs(spec['function']['strict'], strict)
            result = registry.invoke(ToolCall('bad', 'move_to_pose', {'pose_name':'not-allowed'}), tool_context())
            self.assertEqual(result.error_code, 'invalid_arguments')
            self.assertEqual(skills.tasks, {})

    def test_bad_generation_settings_do_not_create_files(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder)/'provider.local.toml'
            for options in ({'preset':'does-not-exist'}, {'preset':'custom'},
                            {'preset':'custom', 'base_url':'https://name:SECRET@example.invalid/v1'},
                            {'preset':'deepseek', 'credential_env':'not-a-variable-name'}):
                with self.subTest(options=options), self.assertRaises(ValueError):
                    generate(model='fixture', output=out, **options)
                self.assertFalse(out.exists())
                self.assertEqual(list(Path(folder).iterdir()), [])


if __name__ == '__main__':
    unittest.main()
