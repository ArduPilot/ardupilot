#!/usr/bin/env python3
'''
Validate that defines in hwdef files are used in the ArduPilot codebase.

This tool iterates through all boards, extracts defines from their hwdef files,
and checks if each define is referenced somewhere in the codebase or submodules.

AP_FLAKE8_CLEAN
'''

import argparse
import multiprocessing
import os
import pathlib
import subprocess
import sys
from typing import Dict, Tuple, Optional

# Add the scripts directory to the path
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

from board_list import BoardList  # noqa: E402


# Module-level cache for codebase content (built once per worker process)
_codebase_content_cache = None


def build_codebase_content() -> str:
    """
    Build a single string containing all source file content from the repository.
    Uses a module-level cache so it's only built once per process.

    Returns:
        A single string with all file contents concatenated
    """
    global _codebase_content_cache

    if _codebase_content_cache is not None:
        return _codebase_content_cache

    repo_root = pathlib.Path(os.path.dirname(__file__)) / '..' / '..'
    repo_root = repo_root.resolve()

    # File extensions to check (C/C++ source/header files and Python scripts)
    source_extensions = {'.cpp', '.h', '.c', '.py'}

    def should_check_file(filepath: str) -> bool:
        """Determine if we should check this file"""
        path = pathlib.Path(filepath)

        # Only check files with relevant extensions
        if path.suffix not in source_extensions:
            return False

        return True

    def get_git_files(repo_path: pathlib.Path) -> list:
        """Get all tracked files from git"""
        try:
            result = subprocess.run(
                ['git', 'ls-files'],
                cwd=repo_path,
                capture_output=True,
                text=True,
                check=True
            )
            return result.stdout.splitlines()
        except subprocess.CalledProcessError:
            return []

    content_parts = []

    # Read files from main repository
    for relative_path in get_git_files(repo_root):
        if not should_check_file(relative_path):
            continue

        filepath = repo_root / relative_path
        try:
            content = filepath.read_text(encoding='utf-8', errors='ignore')
            content_parts.append(content)
        except Exception:
            # Skip files that can't be read
            pass

    # Also read files from submodules
    try:
        result = subprocess.run(
            ['git', 'submodule', 'foreach', '--quiet', 'echo $sm_path'],
            cwd=repo_root,
            capture_output=True,
            text=True,
            check=True
        )
        submodule_paths = result.stdout.splitlines()

        for submodule_path in submodule_paths:
            submodule_dir = repo_root / submodule_path
            if not submodule_dir.exists():
                continue

            for relative_path in get_git_files(submodule_dir):
                if not should_check_file(relative_path):
                    continue

                filepath = submodule_dir / relative_path
                try:
                    content = filepath.read_text(encoding='utf-8', errors='ignore')
                    content_parts.append(content)
                except Exception:
                    # Skip files that can't be read
                    pass
    except subprocess.CalledProcessError:
        pass

    # Join all content with newlines and cache it
    _codebase_content_cache = '\n'.join(content_parts)
    return _codebase_content_cache


def validate_board_worker(args: Tuple) -> Tuple[str, Optional[Dict[str, str]], Optional[str]]:
    """
    Worker function for parallel board validation.

    Args:
        args: Tuple of (board, board_num, total_boards, verbose)

    Returns:
        Tuple of (board_name, unused_defines_dict, error_message)
    """
    board, board_num, total_boards, verbose = args

    # Build codebase content once per worker process (cached)
    codebase_content = build_codebase_content()

    # Import here to avoid issues with multiprocessing
    board_list = BoardList()

    try:
        # Get the HWDef instance for this board
        hwdef_instance = board_list.get_HWDef_instance(board)

        if hwdef_instance is None:
            # SITL board - no hwdef file
            return (board.name, None, None)

        # Process the hwdef files to populate the defines dictionary
        hwdef_instance.process_hwdefs()

        # Get defines from the hwdef instance
        defines = hwdef_instance.defines

        # Check each define
        unused = {}
        for define_name, line in defines.items():
            # First check if this define is used by other defines in the same hwdef
            used_in_hwdef = False
            for other_name, other_value in defines.items():
                if other_name != define_name and other_value is not None:
                    if define_name in other_value:
                        used_in_hwdef = True
                        break

            # If not used in hwdef, check if used in codebase
            if not used_in_hwdef and not check_define_in_codebase(define_name, codebase_content):
                unused[define_name] = line

        return (board.name, unused if unused else None, None)

    except Exception as e:
        error_msg = f"Error validating board {board.name}: {e}"
        return (board.name, None, error_msg)


def check_define_in_codebase(define_name: str, codebase_content: Optional[str] = None) -> bool:
    """
    Check if a define is used in the codebase.

    Args:
        define_name: The define to search for
        codebase_content: Optional pre-built string containing all source file content.
                         If provided, search in this string. Otherwise, read files.

    Returns:
        True if the define is found, False otherwise
    """
    # Use the pre-built content string for fast searching
    return define_name in codebase_content


class HWDefValidator:
    def __init__(self, verbose: bool = False, quiet: bool = False):
        self.verbose = verbose
        self.quiet = quiet
        self.board_list = BoardList()
        self.unused_defines: Dict[str, Dict[str, str]] = {}
        self.errors: list = []

    def validate_boards(self, boards: list, num_workers: int = 1) -> Dict[str, Dict[str, str]]:
        """
        Validate boards using sequential or parallel execution.

        Args:
            boards: List of Board instances to validate
            num_workers: Number of parallel worker processes (1 = sequential)

        Returns:
            Dictionary mapping board names to lists of unused defines
        """
        total = len(boards)

        # Always use parallel execution (with num_workers processes)
        if not self.quiet:
            if num_workers > 1:
                print(f"Validating {total} boards using {num_workers} parallel workers...")
            else:
                print(f"Validating {total} boards...")

        # Prepare arguments for workers (each worker will build codebase content once)
        work_items = [
            (board, idx + 1, total, self.verbose)
            for idx, board in enumerate(boards)
        ]

        # Create a multiprocessing pool
        with multiprocessing.Pool(processes=num_workers) as pool:
            # Use imap_unordered for better progress tracking
            completed = 0

            for result in pool.imap_unordered(validate_board_worker, work_items):
                board_name, unused, error = result
                completed += 1
                remaining = total - completed

                # Print progress
                if not self.quiet:
                    print(f"Completed {completed}/{total} boards ({remaining} remaining): {board_name}")

                # Store results
                if error:
                    self.errors.append(error)
                    if self.quiet:
                        print(f"  {error}")

                if unused:
                    self.unused_defines[board_name] = unused

        return self.unused_defines

    def print_results(self) -> int:
        """Print validation results"""
        if self.errors:
            print("\nErrors encountered:")
            for error in self.errors:
                print(f"  {error}")

        if self.unused_defines:
            print(f"\nFound unused defines in {len(self.unused_defines)} boards:")
            for board_name, unused_dict in sorted(self.unused_defines.items()):
                print(f"\n{board_name}:")
                for define_name, line in unused_dict.items():
                    print(f"  {define_name}: {line}")
            return 1

        # Only print success message if not in quiet mode
        if not self.quiet:
            print("\nAll defines are used in the codebase!")

        return 0


def main():
    parser = argparse.ArgumentParser(
        description='Validate that hwdef defines are used in the codebase'
    )
    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        help='Enable verbose output'
    )
    parser.add_argument(
        '-q', '--quiet',
        action='store_true',
        help='Quiet mode - only show output on failure'
    )
    parser.add_argument(
        '-n', '--parallel',
        type=int,
        metavar='N',
        help='Number of parallel processes to use for validation'
    )
    parser.add_argument(
        '--board',
        action='append',
        dest='boards',
        help='Validate specific board(s). Can be specified multiple times.'
    )
    parser.add_argument(
        '--all-boards',
        action='store_true',
        help='Validate all boards'
    )

    args = parser.parse_args()

    # Validate arguments
    if args.boards and args.all_boards:
        print("Error: Cannot specify both --board and --all-boards")
        return 1

    if not args.boards and not args.all_boards:
        print("Error: Must specify either --board or --all-boards")
        return 1

    if args.verbose and args.quiet:
        print("Error: Cannot specify both --verbose and --quiet")
        return 1

    validator = HWDefValidator(verbose=args.verbose, quiet=args.quiet)

    # Determine which boards to validate
    if args.all_boards:
        boards_to_validate = validator.board_list.boards
    else:
        # Validate specific boards
        boards_to_validate = []
        for board_name in args.boards:
            board = None
            for b in validator.board_list.boards:
                if b.name == board_name:
                    board = b
                    break

            if board is None:
                print(f"Error: Board '{board_name}' not found")
                return 1

            boards_to_validate.append(board)

    # Validate boards (parallel or sequential based on num_workers)
    num_workers = args.parallel if args.parallel else 1
    if num_workers < 1:
        print("Error: Number of parallel processes must be at least 1")
        return 1

    validator.validate_boards(boards_to_validate, num_workers)

    return validator.print_results()


if __name__ == '__main__':
    sys.exit(main())
