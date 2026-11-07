"""Shared fixtures for ArduPilot autotest pytest suite."""

from __future__ import annotations

import importlib.util
import pathlib
import tempfile

from typing import TYPE_CHECKING
from typing import Any

if TYPE_CHECKING:
    from collections.abc import Generator
    from importlib.machinery import ModuleSpec

import pytest

_HERE = pathlib.Path(__file__).resolve().parent
_TOOLS = _HERE.parent
_ROOT = _TOOLS.parent


def rootdir() -> pathlib.Path:
    return _ROOT


def scripts_dir() -> pathlib.Path:
    return _TOOLS / "scripts"


def autotest_dir() -> pathlib.Path:
    return _HERE


def load_module(name: str, path: pathlib.Path) -> Any:
    spec: ModuleSpec | None = importlib.util.spec_from_file_location(name, str(path))
    if spec is None or spec.loader is None:
        raise ImportError(f"Could not load module from {path}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="session")
def ardubuild_root() -> pathlib.Path:
    return _ROOT


@pytest.fixture(scope="session")
def board_list_module() -> Any:
    return load_module("board_list", _TOOLS / "scripts" / "board_list.py")


@pytest.fixture(scope="session")
def build_options_module() -> Any:
    return load_module("build_options", _TOOLS / "scripts" / "build_options.py")


@pytest.fixture(scope="session")
def extract_features_module() -> Any:
    return load_module("extract_features", _TOOLS / "scripts" / "extract_features.py")


@pytest.fixture
def tmp_workdir() -> Generator[pathlib.Path, None, None]:
    with tempfile.TemporaryDirectory(prefix="ap_autotest_") as td:
        yield pathlib.Path(td)


@pytest.fixture(scope="session")
def shared_tmpdir() -> pathlib.Path:
    return pathlib.Path(tempfile.mkdtemp(prefix="ap_autotest_shared_"))


@pytest.fixture
def build_options(build_options_module: Any) -> Any:
    return build_options_module.BUILD_OPTIONS


@pytest.fixture
def default_board() -> str:
    return "CubeOrange"


@pytest.fixture
def build_targets() -> list[str]:
    return ["copter", "plane", "rover"]
