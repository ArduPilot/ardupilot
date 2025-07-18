"""
Ensure that "python3 -m pip install " is used consistently in GitHub Actions.

Find all .yml files in the .github/workflows directory and find all lines that contain
" pip install " and ensure that they also contain "python3 -m pip install ".

If there is any line that has the first but not the second, then print the file name,
line number, and the line.  Raise an error if any non-compliant files are found.
"""

from __future__ import annotations

from pathlib import Path
from typing import Iterable
from unittest import mock

import pytest


def process_file(file_path: Path) -> Iterable[tuple[int, str]]:
    """Process a file and yield any lines that contain " pip install " but not
    "python3 -m pip install ".
    """
    with file_path.open() as in_file:
        for line_number, line in enumerate(in_file, start=1):
            if " pip install " in line and "python3 -m pip install " not in line:
                yield line_number, line.strip()


def test_pip_install_without_python3_minus_m_prefix() -> None:
    workflows_dir = Path(".github/workflows")
    if not workflows_dir.is_dir():
        msg = f"Directory {workflows_dir} does not exist."
        raise FileNotFoundError(msg)

    errors = []
    for yml_file in workflows_dir.glob("*.yml"):
        errors.extend(
            f"{yml_file}:{line_number}: {line}"
            for line_number, line in process_file(yml_file)
        )
    if errors:
        msg = (
            "Use 'python3 -m pip install' instead of 'pip install' "
            f"({len(errors)} errors). "
        )
        print(f"{msg}\n" + "\n".join(sorted(errors)))
        raise ValueError(msg)


@pytest.mark.parametrize(
    "lines,expected",
    [
        (
            [
                "This is a test line.\n",
                " pip install some_package\n",
                "python3 -m pip install another_package\n",
                " pip install yet_another_package\n",
            ],
            [(2, "pip install some_package"), (4, "pip install yet_another_package")],
        ),
        (
            ["No pip install here.\n", "Just a random line.\n"],
            [(1, "No pip install here.")],
        ),
        (
            [" pip install package1\n", "python3 -m pip install package2\n"],
            [(1, "pip install package1")],
        ),
        (["pip install package1\n", "python3 -m pip install package2\n"], []),
        ([], []),
    ],
)
def test_process_file(lines, expected) -> None:
    """Mocking the file reading to simulate the content of a file"""
    mock_file_path = mock.Mock(spec=Path)
    # Use mock_open to properly handle file context manager and iteration
    mock_file = mock.mock_open(read_data="".join(lines))
    mock_file.return_value.__iter__ = lambda self: iter(lines)
    mock_file_path.open = mock_file
    assert list(process_file(mock_file_path)) == expected
