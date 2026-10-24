#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Consecutive PPP tests must not reuse an externally restored executable."""

import os
import sys
import tempfile
import unittest

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rover import AutoTestRover  # noqa: E402


class PPPWebServerTest(unittest.TestCase):
    def test_each_backend_relinks_after_restoring_the_non_ppp_binary(self):
        with tempfile.TemporaryDirectory() as directory:
            previous = os.getcwd()
            os.chdir(directory)
            try:
                binary = Path('build/sitl/bin/ardurover')
                binary.parent.mkdir(parents=True)
                binary.write_bytes(b'non-PPP')
                context = SimpleNamespace(sitl_commandline_customised=False)
                suite = SimpleNamespace(context_get=lambda: context)
                for method in ('context_push', 'context_pop', 'context_collect', 'progress', 'set_parameters',
                               'install_rust_applet_context', 'install_applet_script_context', 'reboot_sitl',
                               'rust_scripting_restart', 'scripting_restart', 'wait_text', 'wait_ready_to_arm',
                               'TestWebServer'):
                    setattr(suite, method, Mock())

                def build(target, **kwargs):
                    self.assertEqual(target, 'bin/ardurover')
                    self.assertIn('--enable-PPP', kwargs['extra_configure_args'])
                    # Waf otherwise sees the same task signature on the second
                    # call and leaves the restored non-PPP output untouched.
                    self.assertFalse(binary.exists())
                    binary.write_bytes(b'PPP')

                with patch('rover.util.build_SITL', side_effect=build) as rebuild:
                    with patch('rover.util.start_PPP_daemon', return_value=Mock()):
                        for rust in (False, True):
                            AutoTestRover.NetworkingWebServerPPPMain(suite, rust)
                            self.assertEqual(binary.read_bytes(), b'non-PPP')
                self.assertEqual(rebuild.call_count, 2)
                suite.scripting_restart.assert_called_once()
                suite.rust_scripting_restart.assert_called_once()
            finally:
                os.chdir(previous)


if __name__ == '__main__':
    unittest.main()
