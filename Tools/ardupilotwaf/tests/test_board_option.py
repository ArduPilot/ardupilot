# AP_FLAKE8_CLEAN
"""Regression coverage for board selection on the Waf command line."""

import os
import subprocess
import sys

from pathlib import Path

import pytest


@pytest.mark.parametrize("board_args", [
    ["board=CubeOrange"],
    ["board =CubeOrange"],
    ["--board", "sitl", "board=CubeOrange"],
])
def test_configure_rejects_bare_board_assignment(tmp_path, board_args):
    root = Path(__file__).resolve().parents[3]
    out_dir = tmp_path / "build"
    result = subprocess.run(
        [sys.executable, "./waf", "configure", *board_args, "--no-submodule-update", "--out", str(out_dir), "-j1"],
        cwd=root,
        env={**os.environ, "WAFLOCK": str(tmp_path / "waf.lock")},
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
        timeout=30,
    )

    assert result.returncode == 1, result.stdout
    assert "Invalid board option 'board='. Use --board=BOARD." in result.stdout
    assert not out_dir.exists()
