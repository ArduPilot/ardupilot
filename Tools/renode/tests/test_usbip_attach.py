#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Tests for the Renode USB/IP attachment helper."""

import argparse
import importlib.util

from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest

MODULE_PATH = Path(__file__).resolve().parents[1] / 'usbip_attach.py'
SPEC = importlib.util.spec_from_file_location('usbip_attach', MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
usbip_attach = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(usbip_attach)


def test_signal_detaches_active_port(tmp_path):
    args = argparse.Namespace(host='127.0.0.1', port=3240, busid='1-0',
                              install_rules=False)
    detached = []

    def request_stop(_delay):
        usbip_attach.request_stop(None, None)

    with mock.patch.object(usbip_attach, 'parse_args', return_value=args), \
            mock.patch.object(usbip_attach.signal, 'signal'), \
            mock.patch.object(usbip_attach, 'VHCI', tmp_path), \
            mock.patch.object(usbip_attach.os, 'access', return_value=True), \
            mock.patch.object(usbip_attach, 'attach', return_value=(4, '')), \
            mock.patch.object(usbip_attach, 'port_attached', return_value=True), \
            mock.patch.object(usbip_attach, 'detach', side_effect=detached.append), \
            mock.patch.object(usbip_attach.time, 'sleep', side_effect=request_stop):
        usbip_attach.main()

    assert detached == [4]


def test_malformed_vhci_status_port_is_ignored(tmp_path):
    status = tmp_path / 'status'
    status.write_text('hub port sta spd dev socket busid\n'
                      'hs not-a-port 006 00 0 0 1-0\n')
    with mock.patch.object(usbip_attach, 'VHCI', tmp_path):
        assert not usbip_attach.port_attached(0)


def test_install_rules_requires_dialout_group(tmp_path):
    udev_rule = tmp_path / '99-vhci-user.rules'
    modules_load = tmp_path / 'vhci-hcd.conf'
    with mock.patch.object(usbip_attach.os, 'geteuid', return_value=0), \
            mock.patch.object(usbip_attach.grp, 'getgrnam',
                              side_effect=KeyError('dialout')), \
            mock.patch.object(usbip_attach, 'UDEV_RULE_PATH', udev_rule), \
            mock.patch.object(usbip_attach, 'MODULES_LOAD_PATH', modules_load):
        with pytest.raises(SystemExit, match='requires a dialout group'):
            usbip_attach.install_rules()

    assert not udev_rule.exists()
    assert not modules_load.exists()


def test_install_rules_rejects_missing_vhci_files(tmp_path):
    with mock.patch.object(usbip_attach.os, 'geteuid', return_value=0), \
            mock.patch.object(usbip_attach.grp, 'getgrnam',
                              return_value=SimpleNamespace(gr_gid=20)), \
            mock.patch.object(usbip_attach, 'UDEV_RULE_PATH',
                              tmp_path / '99-vhci-user.rules'), \
            mock.patch.object(usbip_attach, 'MODULES_LOAD_PATH',
                              tmp_path / 'vhci-hcd.conf'), \
            mock.patch.object(usbip_attach, 'VHCI', tmp_path / 'vhci'), \
            mock.patch.object(usbip_attach.subprocess, 'run'):
        with pytest.raises(SystemExit, match='did not create'):
            usbip_attach.install_rules()
