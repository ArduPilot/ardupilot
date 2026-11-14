#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Native-module build/install helpers, without a running vehicle."""

import os
import subprocess
import sys
import tempfile
import unittest

from pathlib import Path
from types import MethodType
from types import SimpleNamespace
from unittest.mock import Mock
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vehicle_test_suite import Context  # noqa: E402
from vehicle_test_suite import PreconditionFailedException  # noqa: E402
from vehicle_test_suite import TestSuite as VehicleTestSuite  # noqa: E402


class ModuleLoadersTest(unittest.TestCase):
    def setUp(self):
        self.work = tempfile.TemporaryDirectory(prefix='rust-autotest-unit-')
        self.addCleanup(self.work.cleanup)
        old_directory = os.getcwd()
        os.chdir(self.work.name)
        self.addCleanup(os.chdir, old_directory)
        self.root = Path(__file__).resolve().parents[3]
        self.suite = SimpleNamespace(contexts=[Context()], rootdir=lambda: str(self.root),
                                     progress=Mock(), set_parameters=Mock(), mav=None)
        self.suite.context_get = lambda: self.suite.contexts[-1]
        self.suite.check_rust_toolchain = Mock()
        for name in ('build_rust_module', 'install_rust_file_context', 'install_rust_module_context',
                     'install_rust_applet_context', 'install_rust_test_module_context',
                     'rust_applet_source_path', 'rust_test_source_path', 'context_pop', 'context_push',
                     'context_collect', 'install_message_hook_context', 'rust_scripting_restart'):
            setattr(self.suite, name, MethodType(getattr(VehicleTestSuite, name), self.suite))
        self.source = Path(self.work.name) / 'probe.rs'
        self.source.write_text('// test fixture\n')

    def fake_build(self, command, **kwargs):
        self.assertEqual(command[0], sys.executable)
        self.assertEqual(command[1], str(self.root / 'Tools/modules/build.py'))
        self.assertEqual(kwargs['directory'], str(self.root))
        self.assertTrue(Path(kwargs['env']['TMPDIR']).is_dir())
        self.build_output = Path(command[3])
        self.build_output.write_bytes(b'module')
        self.build_output.with_suffix('.elf').write_bytes(b'elf')
        self.build_output.with_name('libmodule_sdk.rlib').write_bytes(b'sdk')

    def test_build_install_and_context_cleanup(self):
        with patch('vehicle_test_suite.util.run_cmd', side_effect=self.fake_build) as run:
            dest = self.suite.install_rust_module_context(str(self.source))
        self.assertEqual(Path(dest).read_bytes(), b'module')
        self.assertEqual(Path(dest).name, 'probe.apmod')
        self.assertFalse(self.build_output.parent.exists())
        self.assertEqual(run.call_args.args[0][-4:], ['--target', 'x86_64-unknown-linux-gnu', '--stack-size', '16384'])
        self.suite.context_pop(process_interaction_allowed=False)
        self.assertFalse(Path(dest).exists())

    def test_applet_and_test_module_forward_options(self):
        self.suite.install_rust_module_context = Mock()
        self.suite.install_rust_applet_context('aerobatics/main.rs', install_name='airshow.apmod', stack_size=8192)
        self.suite.install_rust_module_context.assert_called_once_with(
            str(self.root / 'libraries/AP_Module/applets/aerobatics/main.rs'),
            install_name='airshow.apmod', stack_size=8192)
        self.suite.install_rust_module_context.reset_mock()
        self.suite.install_rust_test_module_context('math.rs', target='thumbv7em-none-eabihf')
        self.suite.install_rust_module_context.assert_called_once_with(
            str(self.root / 'libraries/AP_Module/tests/modules/math.rs'), target='thumbv7em-none-eabihf')

    def test_failed_build_installs_nothing(self):
        def fail(command, **kwargs):
            self.fake_build(command, **kwargs)
            raise subprocess.CalledProcessError(1, command)
        with patch('vehicle_test_suite.util.run_cmd', side_effect=fail):
            with self.assertRaises(subprocess.CalledProcessError):
                self.suite.install_rust_module_context(str(self.source))
        self.assertFalse(Path('modules').exists())
        self.assertFalse(self.build_output.parent.exists())
        self.assertEqual(self.suite.context_get().installed_rust_files, [])

    def test_compiler_environment_and_scratch(self):
        with patch.dict(os.environ, {'RUSTC': '/chosen/rustc', 'TMPDIR': '/nonexistent-rust-tmp'}):
            with patch('vehicle_test_suite.util.run_cmd', side_effect=self.fake_build) as run:
                self.suite.install_rust_module_context(str(self.source))
            env = run.call_args.kwargs['env']
            self.assertEqual(env['RUSTC'], '/chosen/rustc')
            self.assertFalse(Path(env['TMPDIR']).exists())
            self.assertEqual(os.environ['TMPDIR'], '/nonexistent-rust-tmp')
        self.suite.context_pop(process_interaction_allowed=False)

    def test_existing_file_is_not_overwritten_or_removed(self):
        Path('modules').mkdir()
        dest = Path('modules/existing.apmod')
        dest.write_bytes(b'existing')
        with self.assertRaises(FileExistsError):
            self.suite.install_rust_file_context(self.source, install_name=dest.name)
        self.suite.context_pop(process_interaction_allowed=False)
        self.assertEqual(dest.read_bytes(), b'existing')

    def test_copy_failure_is_cleaned_by_context(self):
        with patch('vehicle_test_suite.shutil.copyfileobj', side_effect=OSError('copy failed')):
            with self.assertRaises(OSError):
                self.suite.install_rust_file_context(self.source, install_name='broken.apmod')
        self.suite.context_pop(process_interaction_allowed=False)
        self.assertFalse(Path('rust/broken.apmod').exists())

    def test_nested_context_and_schedule_cleanup(self):
        module = self.suite.install_rust_file_context(self.source, install_name='outer.apmod')
        self.suite.contexts.append(Context())
        schedule = self.suite.install_rust_file_context(self.source, install_name='trick72.txt', directory='scripts')
        with self.assertRaises(FileExistsError):
            self.suite.install_rust_file_context(self.source, install_name='outer.apmod')
        self.suite.context_pop(process_interaction_allowed=False)
        self.assertFalse(Path(schedule).exists())
        self.assertTrue(Path(module).exists())
        Path(module).unlink()  # cleanup tolerates a file removed by the test
        self.suite.context_pop(process_interaction_allowed=False)

    def test_invalid_destination(self):
        with self.assertRaises(ValueError):
            self.suite.install_rust_module_context(str(self.source), install_name='bad.elf')
        with self.assertRaises(ValueError):
            self.suite.install_rust_file_context(self.source, directory='../elsewhere')

    def test_restart_wraps_and_does_not_restore_request(self):
        self.suite.get_parameter = Mock(return_value=32767)
        self.suite.set_parameter = Mock()
        self.suite.drain_mav = Mock()
        self.suite.wait_text = Mock(side_effect=RuntimeError('reload timed out'))
        with self.assertRaisesRegex(RuntimeError, 'reload timed out'):
            self.suite.rust_scripting_restart(timeout=10)
        self.suite.set_parameter.assert_called_once_with('MOD_RESTART', -32768, add_to_context=False)
        self.suite.wait_text.assert_called_once_with(
            r'Module: loaded [0-9]+ modules', regex=True, check_context=True, timeout=10)
        self.assertEqual(len(self.suite.contexts), 1)

    def test_restart_success(self):
        self.suite.get_parameter = Mock(return_value=2)
        self.suite.set_parameter = Mock()
        self.suite.drain_mav = Mock()
        self.suite.wait_text = Mock()
        self.suite.rust_scripting_restart()
        self.suite.set_parameter.assert_called_once_with('MOD_RESTART', 3, add_to_context=False)
        self.assertEqual(len(self.suite.contexts), 1)
        self.assertEqual(self.suite.context_get().parameters, [])

    def test_toolchain_preconditions(self):
        with patch('vehicle_test_suite.subprocess.check_output', side_effect=FileNotFoundError):
            with self.assertRaisesRegex(PreconditionFailedException, 'Install Rust 1.98.1 or newer'):
                VehicleTestSuite.check_rust_toolchain(self.suite)
        with patch('vehicle_test_suite.subprocess.check_output', return_value='rustc 1.98.0 (example)'):
            with self.assertRaisesRegex(PreconditionFailedException, 'Install Rust 1.98.1 or newer'):
                VehicleTestSuite.check_rust_toolchain(self.suite)
        libdir = Path(self.work.name) / 'target-lib'
        libdir.mkdir()
        replies = ['rustc 1.98.1 (example)', str(libdir)]
        with patch('vehicle_test_suite.subprocess.check_output', side_effect=replies):
            with self.assertRaisesRegex(PreconditionFailedException, 'Missing Rust core'):
                VehicleTestSuite.check_rust_toolchain(self.suite)
        (libdir / 'libcore-fixture.rlib').write_bytes(b'core')
        replies = ['rustc 1.101.0-nightly (example)', str(libdir)]
        with patch('vehicle_test_suite.subprocess.check_output', side_effect=replies):
            VehicleTestSuite.check_rust_toolchain(self.suite)


if __name__ == '__main__':
    unittest.main()
