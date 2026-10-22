"""The Conda adapter only selects an interpreter and reuses the project locks."""
from contextlib import redirect_stdout
import importlib.util
import io
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('setup_conda', ROOT/'scripts/setup_conda.py')
installer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(installer)


class CondaSetupTests(unittest.TestCase):
    def fixture(self, folder, *, known=False, version=(3,12), is_conda=True, prefix_is_base=False, uv_installed=True):
        base = Path(folder)/'base'
        prefix = base if prefix_is_base else Path(folder)/'envs/eai-test'
        calls = []
        def run(args, **options):
            calls.append((args, options))
            output, code = '', 0
            if args[1:] == ['info','--base']:
                output = str(base)
            elif args[1:] == ['env','list','--json']:
                output = json.dumps({'envs': [str(prefix)] if known else []})
            elif args[-1] == installer.PROBE:
                output = json.dumps({'prefix':str(prefix),'python':str(prefix/'python'), 'version':list(version), 'conda':is_conda})
            elif args[-3:] == ['-m','uv','--version']:
                output, code = ('uv '+installer.UV_VERSION, 0) if uv_installed else ('',1)
            return subprocess.CompletedProcess(args, code, stdout=output, stderr='')
        return prefix, calls, run

    def test_control_uses_scoped_conda_prefix_and_original_lock(self):
        with tempfile.TemporaryDirectory() as folder:
            prefix, calls, run = self.fixture(folder, uv_installed=False)
            with patch.dict(os.environ, {'CONDA_EXE':'conda-test', 'UV_PROJECT_ENVIRONMENT':'wrong-target',
                                        'VIRTUAL_ENV':'other-venv', 'UV_FROZEN':'1'}), patch.object(installer.subprocess,'run',side_effect=run), redirect_stdout(io.StringIO()):
                self.assertEqual(installer.setup(name='eai-test'),0)
                self.assertEqual(os.environ['UV_PROJECT_ENVIRONMENT'],'wrong-target')
            sync, options = next((a,o) for a,o in calls if 'sync' in a)
            self.assertIn('--locked',sync)
            self.assertIn('--inexact',sync)
            self.assertNotIn('--frozen',sync)
            self.assertEqual(sync[sync.index('--project')+1],str(ROOT))
            self.assertEqual(options['env']['UV_PROJECT_ENVIRONMENT'],str(prefix.resolve()))
            self.assertNotIn('UV_FROZEN',options['env'])
            self.assertNotIn('VIRTUAL_ENV',options['env'])
            self.assertTrue(any('uv=='+installer.UV_VERSION in a for a,_ in calls))
            self.assertTrue(any('unittest' in a for a,_ in calls))
            self.assertTrue(any('--demo' in a for a,_ in calls))
            self.assertTrue(all('run' not in a or '--name' in a for a,_ in calls))
            self.assertFalse(any('--enable' in a or '--execute' in a for a,_ in calls))

    def test_training_uses_its_own_lock_and_gpu_is_opt_in(self):
        for require_cuda in (False,True):
            with self.subTest(require_cuda=require_cuda), tempfile.TemporaryDirectory() as folder:
                _, calls, run = self.fixture(folder)
                with patch.dict(os.environ, {'CONDA_EXE':'conda-test'}), patch.object(installer.subprocess,'run',side_effect=run), redirect_stdout(io.StringIO()):
                    installer.setup(name='eai-test',kind='training',require_cuda=require_cuda)
                sync = next(a for a,_ in calls if 'sync' in a)
                self.assertEqual(sync[sync.index('--project')+1],str(ROOT/'environments/training'))
                check = next(a for a,_ in calls if 'scripts/check_training_env.py' in a)
                self.assertEqual('--require-cuda' in check,require_cuda)
                self.assertFalse(any('unittest' in a or '--demo' in a for a,_ in calls))
                self.assertTrue(any('lerobot-train' in a and '--help' in a for a,_ in calls))

    def test_base_and_unknown_existing_environments_are_not_modified(self):
        for name in ('base','root','../other','two words'):
            with patch.object(installer.subprocess,'run') as run, self.assertRaises(ValueError):
                installer.setup(name=name)
            run.assert_not_called()
        with tempfile.TemporaryDirectory() as folder:
            _, calls, run = self.fixture(folder, known=True)
            with patch.dict(os.environ, {'CONDA_EXE':'conda-test'}), patch.object(installer.subprocess,'run',side_effect=run), self.assertRaisesRegex(ValueError,'exists'):
                installer.setup(name='eai-test')
            self.assertFalse(any('create' in a or 'sync' in a for a,_ in calls))

    def test_reuse_is_explicit_and_invalid_interpreter_stops_before_install(self):
        for version, is_conda, base in (((3,11),True,False), ((3,12),False,False), ((3,12),True,True)):
            with tempfile.TemporaryDirectory() as folder:
                _, calls, run = self.fixture(folder, version=version, is_conda=is_conda, prefix_is_base=base)
                with patch.dict(os.environ, {'CONDA_EXE':'conda-test'}), patch.object(installer.subprocess,'run',side_effect=run), self.assertRaises(ValueError):
                    installer.setup(name='eai-test',reuse=True)
                self.assertFalse(any('sync' in a or 'install' in a for a,_ in calls))
        with tempfile.TemporaryDirectory() as folder:
            _, calls, run = self.fixture(folder, known=True)
            with patch.dict(os.environ, {'CONDA_EXE':'conda-test'}), patch.object(installer.subprocess,'run',side_effect=run), redirect_stdout(io.StringIO()):
                installer.setup(name='eai-test',reuse=True,skip_tests=True)
            self.assertFalse(any('create' in a or 'unittest' in a for a,_ in calls))
            self.assertTrue(any('scripts/check_env.py' in a for a,_ in calls))
            self.assertTrue(any('--demo' in a for a,_ in calls))

    def test_sync_failure_stops_checks_and_pinned_uv_matches_repository(self):
        import tomllib
        required = tomllib.loads((ROOT/'pyproject.toml').read_text())['tool']['uv']['required-version']
        self.assertEqual(required,'=='+installer.UV_VERSION)
        with tempfile.TemporaryDirectory() as folder:
            _, calls, run = self.fixture(folder)
            def fail(args,**options):
                if 'sync' in args:
                    raise subprocess.CalledProcessError(1,args)
                return run(args,**options)
            with patch.dict(os.environ, {'CONDA_EXE':'conda-test'}), patch.object(installer.subprocess,'run',side_effect=fail), self.assertRaises(subprocess.CalledProcessError):
                installer.setup(name='eai-test')
            self.assertFalse(any('--demo' in a for a,_ in calls))


if __name__ == '__main__':
    unittest.main()
