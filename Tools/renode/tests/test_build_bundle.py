#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Tests for the source-independent Renode runtime bundle."""

import importlib.util
import json
import subprocess
import sys

from pathlib import Path

import pytest

RENODE = Path(__file__).resolve().parents[1]
ROOT = RENODE.parents[1]
MODULE_PATH = RENODE / 'build_bundle.py'
SPEC = importlib.util.spec_from_file_location('renode_build_bundle', MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
build_bundle = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(build_bundle)


def test_bundle_runs_hwdef_compiler_without_source_tree(tmp_path):
    physics = tmp_path / 'renode-physics'
    physics.write_bytes(b'physics executable')
    bundle = tmp_path / 'bundle'
    manifest = build_bundle.create_bundle(ROOT, bundle, physics)

    assert manifest['format'] == 1
    assert manifest['physics'] == 'Tools/renode/bin/renode-physics'
    assert (bundle / manifest['physics']).read_bytes() == physics.read_bytes()
    recorded = json.loads((bundle / 'bundle.json').read_text())
    assert recorded == manifest
    assert (bundle / 'Tools/renode/dfu.py').is_file()
    assert (bundle / 'Tools/renode/peripherals/common/'
            'AP_UnixSocketTerminal.cs').is_file()
    assert not list(bundle.rglob('*.png'))
    assert not list(bundle.rglob('__pycache__'))

    for script in ('launch.py', 'run.py'):
        subprocess.run(
            [sys.executable, '-E', str(bundle / 'Tools/renode' / script), '--help'],
            cwd=tmp_path, check=True, capture_output=True, text=True)

    listed = subprocess.run(
        [sys.executable, str(bundle / 'Tools/renode/gen_board.py'), '--list'],
        cwd=tmp_path, check=True, capture_output=True, text=True).stdout
    assert 'CubeBlack\n' in listed
    assert 'Pixhawk6X\n' in listed

    for board in ('CubeBlack', 'Pixhawk6X', 'HolybroG4_GPS',
                  '3DR-L431-ASAUAV'):
        generated = tmp_path / board
        result = subprocess.run([
            sys.executable, str(bundle / 'Tools/renode/gen_board.py'),
            board, '--outdir', str(generated),
        ], cwd=tmp_path, check=True, capture_output=True, text=True)
        assert str(generated / ('%s.repl' % board)) in result.stdout
        assert (generated / ('%s.repl' % board)).is_file()
        assert (generated / ('%s.resc' % board)).is_file()


def test_bundle_refuses_to_replace_existing_directory(tmp_path):
    physics = tmp_path / 'renode-physics.exe'
    physics.touch()
    bundle = tmp_path / 'bundle'
    bundle.mkdir()

    with pytest.raises(FileExistsError):
        build_bundle.create_bundle(ROOT, bundle, physics)


def test_bundle_refuses_concurrent_builder(tmp_path):
    physics = tmp_path / 'renode-physics'
    physics.touch()
    bundle = tmp_path / 'bundle'
    descriptor = build_bundle._claim_output(bundle)
    try:
        with pytest.raises(FileExistsError, match='another bundle build'):
            build_bundle.create_bundle(ROOT, bundle, physics)
    finally:
        build_bundle.os.close(descriptor)
