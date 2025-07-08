#!/usr/bin/env python3

# CAUTION: The first docstring in this file will be the copyright notice that will be
# looked for in all file paths that the pre-commit hook passes in.  If all files
# contain the __doc__ text below, the commit will be allowed to proceed.

"""ArduPilot.org.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""

# --- Do not merges these two docstrings.  See CAUTION above. ---

from argparse import ArgumentParser
from pathlib import Path

r"""
A pre-commit hook that will use the content of __doc__ and then ensure all files
processed contain that same copyright information.  This is useful to ensure that all
those files consistent copyright notices, which is important for legal and compliance
reasons.  If any files does not contain the expected copyright information, the commit
will be aborted and an error message will be displayed with the file paths of all files
that do not match.

Place an entry into the local `.pre-commit-config.yaml` file to run this job.
```yaml
  - repo: local
    hooks:
    -   id: check-copyright
        name: check copyright notice in files
        entry: Tools/gittools/pre_commit_copyright.py
        language: python
        files: |
            (?x)^(
              Tools/ros2/.*\.py
            )$
        args: [
            --ignore=excluded_file.py,
            --ignore=another_excluded.py,
        ]
```

Usage:
1. Place this script in the `Tools/gittools/pre_commit_copyright.py` file.
2. Ensure the docstring of this file is set to the expected copyright notice.
3. Add the entry to your `.pre-commit-config.yaml` file as shown above.
4. Run `pre-commit install` to set up the pre-commit hooks.
5. Now, when you try to commit changes, this hook will check for the copyright notice.
6. If any files do not contain the expected copyright notice, the commit will be aborted.
7. The error message will list all files that do not match the expected copyright notice.
8. Use --ignore=file_path to exclude specific files from copyright checking.

Command line usage:
python pre_commit_copyright.py file1.py file2.py --ignore=excluded_file.py \
    --ignore=another_excluded.py
"""


def get_file_paths() -> list[str]:
    """Parse command line arguments and return a sorted list of file paths excluding
    ignored files."""
    parser = ArgumentParser(description="Check copyright notice in files")
    parser.add_argument("files", nargs="+", help="File paths to check")
    parser.add_argument(
        "--ignore",
        action="append",
        default=[],
        help="File paths to ignore (can be used multiple times)",
    )
    args = parser.parse_args()
    return sorted(set(args.files) - set(args.ignore))


def check_copyright(file_path: Path) -> bool:
    """Check if the file contains the expected copyright notice."""
    try:
        return __doc__ in file_path.read_text()
    except Exception as e:  # noqa: BLE001 Path.read_text() can raise many exceptions
        print(f"Error reading {file_path}: {e}")
        return False


def main():
    """Main function to check all file paths from command line arguments.

    pre-commit will repeatedly call this function and each time it will pass ~four file
    paths to be checked.

    Supports --ignore flags to exclude specific files from copyright checking.
    """
    if failed_files := [
        path for path in get_file_paths() if not check_copyright(Path(path))
    ]:
        raise SystemExit(f"Copyright not found in: {', '.join(failed_files)}")


if __name__ == "__main__":
    main()
