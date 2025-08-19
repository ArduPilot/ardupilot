#!/usr/bin/env python

"""Rewrite Tools/gittools/git-subsystems-split in Python.

git-subsystems-split.py [OPTIONS]

Ardupilot's git extension.

Split HEAD commit into commits separated by subsystems (vehicles, libraries and
folders in the project's root). Basically, reset and call commit-subsystems.

If neither --copy or --edit is passed, then subsystems-split will try to make
the original commit's message into a template for commit-subsystems.

Options:
    --copy
    Make all commits have exactly the same message as the HEAD commit.

    --edit
    Edit the commit message as a template for commit-subsystems.
"""

import argparse
import logging
import os
import subprocess
from collections.abc import Iterator
from pathlib import Path

COMMIT_PREFIXES = {
    s.lower(): s  # The dict key will be a lowercase version of the dict value.
    for s in {
        "AC_Fence",
        "AntennaTracker",
        "AP_Arming",
        "AP_BattMonitor",
        "AP_BoardConfig",
        "AP_Compass",
        "AP_EFI",
        "AP_ExternalAHRS",
        "AP_GPS",
        "AP_HAL_ChibiOS",
        "AP_HAL",
        "AP_InertialSensor",
        "AP_Logger",
        "AP_Mission",
        "AP_NavEKF3",
        "AP_RangeFinder",
        "AP_Scripting",
        "AP_TECS",
        "AP_VisualOdom",
        "ardupilotwaf",
        "ArduCopter",
        "ArduPlane",
        "ArduRover",
        "ArduSub",
        "autotest",
        "benchmarks",
        "Blimp",
        "docs",
        "DroneCAN",
        "EKF",
        "GPS",
        "ground_control",
        "HAL_ChibiOS",
        "hwdef",
        "IMU",
        "libraries",
        "MAVLink",
        "modules",
        "Plane",
        "RC",
        "Rover",
        "Scripting",
        "SITL",
        "tests",
        "Tools",
    }
}
# Files in the root directory and also in "Tools/ardupilotwaf" should use prefix "waf".
COMMIT_PREFIXES["ardupilotwaf"] = "waf"  # Add a special case for Tools/ardupilotwaf.

"""
SCRIPT_DIR=$(dirname $(realpath ${BASH_SOURCE[0]}))
GIT_DIR=$(git rev-parse --git-dir)
GIT_ROOT=$(git rev-parse --show-toplevel)

MSG_FILE="$GIT_DIR/SUBSYSTEMS_SPLIT_MSG"

# script_dir = Path(__file__).resolve().parent
# git_dir = Path(run_command(["git", "rev-parse", "--git-dir"]))
# git_root = Path(run_command(["git", "rev-parse", "--show-toplevel"]))
# msg_file = git_dir / "SUBSYSTEMS_SPLIT_MSG"
"""

logger = logging.getLogger(__name__)


def parse_args() -> argparse.Namespace:
    """Parse command line arguments.

    git-subsystems-split.py [OPTIONS]

    Options:
        --copy: Make all commits have exactly the same message as the HEAD commit.
        --edit: Edit the commit message as a template for commit-subsystems.

        --copy and --edit are mutually exclusive.

        If --edit is not passed, then assume that --copy is passed.
    """
    parser = argparse.ArgumentParser(
        description="Split HEAD commit into multiple commits separated by subsystem"
    )
    parser.add_argument(
        "--copy",
        action="store_true",
        help="Make all commits have exactly the same message as the HEAD commit.",
    )
    parser.add_argument(
        "--edit",
        action="store_true",
        help="Edit the commit message as a template for commit-subsystems.",
    )
    parsed_args = parser.parse_args()
    if parsed_args.copy and parsed_args.edit:
        parser.error(
            "Options --copy and --edit are mutually exclusive. Please choose only one."
        )
    if not parsed_args.edit:  # If --edit is not passed.
        parsed_args.copy = True  # Then assume that --copy is passed.
    return parsed_args


def run_command(command: list[str]) -> str:
    """Run a command and return stdout.strip()."""
    logger.debug(f"Running command: {' '.join(command)}")
    result = subprocess.run(command, check=True, text=True, capture_output=True)
    logger.debug(f"Command output: {result.stdout}")
    return result.stdout.strip()


def get_subsystems() -> Iterator[Path]:
    """Get a list of subsystems (vehicles, libraries and folders in project's root).

    Yield all paths in git_root that do not start with a dot.

    TODO: Does not yield autotest.
    """
    git_root = Path(run_command(["git", "rev-parse", "--show-toplevel"]))
    logger.debug(f"Getting subsystems from {git_root}")
    for path in sorted(git_root.iterdir()):
        if path.is_dir() and not path.name.startswith("."):
            logger.debug(f"Found subsystem: {path}")
            yield path
        else:
            logger.debug(f"Skipping non-subsystem: {path}")


def get_git_commit_author() -> str:
    """Get the author of the current commit.

    Translate this bash code to Python:
    author_name=$(git log -n 1 --format=%an)
    author_email=$(git log -n 1 --format=%ae)
    author="$author_name <$author_email>"
    """
    logger.debug("Getting author of the current commit")
    author = run_command(["git", "log", "-n", "1", "--format=%an <%ae>"])
    logger.debug(f"Author: {author}")
    return author


def get_git_commit_message() -> str:
    """Get the commit message of the current commit.

    Translate this bash code to Python:
    git log -n 1 --format=%B > "$MSG_FILE"
    """
    logger.debug("Getting commit message of the current commit")
    commit_message = run_command(["git", "log", "-n", "1", "--format=%B"])
    logger.debug(f"Commit message: {commit_message}")
    return commit_message


def edit_subsystems_split_message() -> None:
    """Edit the commit message file.

    Translate this bash code to Python:
    ${EDITOR:-vi} ${MSG_FILE:-.git/SUBSYSTEMS_SPLIT_MSG}
    """
    git_dir = Path(run_command(["git", "rev-parse", "--git-dir"]))
    msg_file = git_dir / "SUBSYSTEMS_SPLIT_MSG"
    logger.debug(f"Editing commit message file: {msg_file}")
    editor = run_command(["git", "var", "GIT_EDITOR"]) or os.getenv("EDITOR", "vi")
    subprocess.run([editor, str(msg_file)], check=True)
    logger.debug(f"Edited commit message file: {msg_file}")


def get_git_head() -> str:
    """Get the current HEAD commit hash.

    Translate this bash code to Python:
    git rev-parse HEAD
    """
    logger.debug("Getting current HEAD commit hash")
    head_commit = run_command(["git", "rev-parse", "HEAD"])
    logger.debug(f"HEAD commit: {head_commit}")
    return head_commit


def get_git_subsystems_split_message() -> str:
    git_dir = Path(run_command(["git", "rev-parse", "--git-dir"]))
    msg_file = git_dir / "SUBSYSTEMS_SPLIT_MSG"
    logger.debug(f"Getting commit message from {msg_file}")
    return msg_file.read_text()


def get_commit_prefix_from_filepath(filepath: Path) -> str:
    """Drop the filename from the filepath and then read the filepath from right to
    left and return the rightmost commit_prefix.  Regardless of the case in the
    filepath, always return the commit_prefix in the case in COMMIT_PREFIXES.

    >>> get_commit_prefix_from_filepath(Path("lib/aP_hAl_ChIbIoS/tOOl/README.md"))
    'AP_HAL_ChibiOS'
    >>> get_commit_prefix_from_filepath(Path("lib/aP_hAl_ChIbIoS/tOOls/README.md"))
    'Tools'
    """
    # Drop the filename from the filepath
    filepath = filepath.parent
    # Read the filepath from right to left and return the rightmost commit_prefix.
    for part in reversed(filepath.parts):
        if commit_prefix := COMMIT_PREFIXES.get(part.lower()):
            return commit_prefix
    return "waf"  # Default to "waf" if no commit_prefix is found.

def find_missing_prefixes() -> Iterator[Path]:
    """Find all files in the git repository that do not have a commit prefix.

    This should only yield `PosixPath('.')`. 
    """
    git_root = Path(run_command(["git", "rev-parse", "--show-toplevel"])).resolve()
    for file in git_root.rglob("*"):
        if file.is_file() and not file.name.startswith("."):
            file = file.relative_to(git_root)
            # Only files in the root directory should have a commit prefix of "waf".
            if not str(file).startswith(".") and get_commit_prefix_from_filepath(file) == "waf":
                yield file.parent

if __name__ == "__main__":
    print(sorted(set(find_missing_prefixes())))
    
