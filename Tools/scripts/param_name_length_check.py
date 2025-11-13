#!/usr/bin/env python3
'''
validate that parameter names do not exceed AP_MAX_NAME_SIZE

This script parses ArduPilot C++ source to validate that all parameter names
(including prefixes from nested groups) do not exceed the 16-character limit.

It recursively builds full parameter names and validates their length.

Usage:
    python3 param_name_length_check.py <ardupilot_root_directory> [--verbose] [--warnings]

AP_FLAKE8_CLEAN
'''

import re
import sys
import argparse
from pathlib import Path
from typing import Dict, List, Tuple, Set, Optional
from dataclasses import dataclass


@dataclass
class ParamInfo:
    '''information about a parameter'''
    name: str
    file_path: str
    line_number: int
    class_name: str
    prefix_chain: List[str]
    is_vector3f: bool = False

    def full_name(self) -> str:
        '''full parameter name including all prefixes'''
        return ''.join(self.prefix_chain) + self.name

    def full_length(self) -> int:
        '''full length including Vector3f suffix if applicable'''
        base_length = len(self.full_name())
        # Vector3f parameters get _X, _Y, _Z suffixes (2 additional characters)
        if self.is_vector3f:
            return base_length + 2
        return base_length


@dataclass
class SubgroupInfo:
    '''information about a subgroup (AP_SUBGROUPINFO)'''
    prefix: str
    class_name: str
    file_path: str
    line_number: int


class ParamValidator(object):
    '''validates parameter name lengths in ArduPilot codebase'''

    # maximum parameter name length from AP_Param.h
    MAX_NAME_LENGTH = 16

    # regex patterns for parsing C++ code
    GOBJECT_PATTERN = re.compile(
        r'GOBJECT(?:PTR)?\s*\(\s*\w+\s*,\s*"([^"]+)"\s*,\s*(\w+)\s*\)',
        re.MULTILINE)

    GROUPINFO_PATTERN = re.compile(
        r'AP_GROUPINFO(?:_[A-Z_]+)?\s*\(\s*"([^"]+)"\s*,.*?\)',
        re.MULTILINE)

    SUBGROUPINFO_PATTERN = re.compile(
        r'AP_SUBGROUPINFO\s*\(\s*\w+\s*,\s*"([^"]+)"\s*,\s*\d+\s*,\s*\w+\s*,\s*(\w+)\s*\)',
        re.MULTILINE)

    GROUPINFO_TABLE_PATTERN = re.compile(
        r'const\s+AP_Param::GroupInfo\s+(\w+)::var_info\[\]',
        re.MULTILINE)

    VECTOR3F_PATTERN = re.compile(
        r'AP_GROUPINFO(?:_FLAGS)?(?:_DEFAULT_POINTER)?\s*\(\s*"([^"]+)"\s*,.*?AP_PARAM_VECTOR3F',
        re.MULTILINE | re.DOTALL)

    def __init__(self, root_dir: str, verbose: bool = False, show_warnings: bool = False):
        self.root_dir = Path(root_dir)
        self.verbose = verbose
        self.show_warnings = show_warnings
        self.errors: List[ParamInfo] = []
        self.warnings: List[ParamInfo] = []
        self.top_level_prefixes: Dict[str, str] = {}  # class_name -> prefix
        self.class_files: Dict[str, Path] = {}  # class_name -> file_path
        self.subgroups_cache: Dict[str, List[SubgroupInfo]] = {}  # class_name -> subgroups

    def log(self, message: str):
        '''print verbose logging message'''
        if self.verbose:
            print("[DEBUG] %s" % message)

    def find_cpp_files(self) -> List[Path]:
        '''find all .cpp files in the ArduPilot source tree'''
        cpp_files = []

        # Search in libraries, vehicle directories, and Tools
        search_dirs = [
            self.root_dir / 'libraries',
            self.root_dir / 'ArduCopter',
            self.root_dir / 'ArduPlane',
            self.root_dir / 'Rover',
            self.root_dir / 'ArduSub',
            self.root_dir / 'AntennaTracker',
            self.root_dir / 'Blimp',
            self.root_dir / 'Tools' / 'AP_Periph',
            self.root_dir / 'Tools' / 'Replay',
        ]

        for search_dir in search_dirs:
            if search_dir.exists():
                cpp_files.extend(search_dir.rglob('*.cpp'))

        return cpp_files

    def parse_top_level_prefixes(self):
        '''parse Parameters.cpp files to find GOBJECT/GOBJECTPTR declarations'''
        self.log("Parsing top-level parameter prefixes...")

        # Look for Parameters.cpp in various vehicle directories
        param_files = [
            self.root_dir / 'Tools' / 'AP_Periph' / 'Parameters.cpp',
            self.root_dir / 'ArduCopter' / 'Parameters.cpp',
            self.root_dir / 'ArduPlane' / 'Parameters.cpp',
            self.root_dir / 'Rover' / 'Parameters.cpp',
            self.root_dir / 'ArduSub' / 'Parameters.cpp',
            self.root_dir / 'AntennaTracker' / 'Parameters.cpp',
            self.root_dir / 'Blimp' / 'Parameters.cpp',
        ]

        for param_file in param_files:
            if not param_file.exists():
                continue

            self.log(f"  Parsing {param_file}")
            content = param_file.read_text(errors='ignore')

            for match in self.GOBJECT_PATTERN.finditer(content):
                prefix = match.group(1)
                class_name = match.group(2)
                self.top_level_prefixes[class_name] = prefix
                self.log(f"    Found prefix: {class_name} -> '{prefix}'")

    def find_class_file(self, class_name: str) -> Optional[Path]:
        '''find the .cpp file that defines var_info for a given class'''
        if class_name in self.class_files:
            return self.class_files[class_name]

        # Search for the file containing this class's var_info definition
        cpp_files = self.find_cpp_files()

        for cpp_file in cpp_files:
            try:
                content = cpp_file.read_text(errors='ignore')
                pattern = re.compile(
                    rf'const\s+AP_Param::GroupInfo\s+{re.escape(class_name)}::var_info\[\]',
                    re.MULTILINE
                )
                if pattern.search(content):
                    self.class_files[class_name] = cpp_file
                    self.log(f"  Found var_info for {class_name} in {cpp_file}")
                    return cpp_file
            except Exception as e:
                self.log(f"  Error reading {cpp_file}: {e}")
                continue

        return None

    def parse_subgroups(self, class_name: str, file_path: Path) -> List[SubgroupInfo]:
        '''parse AP_SUBGROUPINFO declarations in a var_info table'''
        if class_name in self.subgroups_cache:
            return self.subgroups_cache[class_name]

        try:
            content = file_path.read_text(errors='ignore')
        except Exception as e:
            self.log(f"  Error reading {file_path}: {e}")
            return []

        subgroups = []
        lines = content.split('\n')

        for line_num, line in enumerate(lines, 1):
            # Skip commented-out lines (both // and /* */ style)
            stripped = line.strip()
            if stripped.startswith('//') or stripped.startswith('/*') or stripped.startswith('*'):
                continue

            match = self.SUBGROUPINFO_PATTERN.search(line)
            if match:
                prefix = match.group(1)
                subclass_name = match.group(2)
                subgroups.append(SubgroupInfo(
                    prefix=prefix,
                    class_name=subclass_name,
                    file_path=str(file_path),
                    line_number=line_num
                ))
                self.log(f"    Found subgroup: {subclass_name} with prefix '{prefix}'")

        self.subgroups_cache[class_name] = subgroups
        return subgroups

    def parse_parameters(self, class_name: str, file_path: Path, prefix_chain: List[str]) -> List[ParamInfo]:
        '''parse AP_GROUPINFO declarations in a var_info table'''
        params = []

        try:
            content = file_path.read_text(errors='ignore')
        except Exception as e:
            self.log(f"  Error reading {file_path}: {e}")
            return params

        lines = content.split('\n')

        # Find Vector3f parameters
        vector3f_params = set()
        for match in self.VECTOR3F_PATTERN.finditer(content):
            vector3f_params.add(match.group(1))

        # Parse all GROUPINFO entries
        for line_num, line in enumerate(lines, 1):
            # Skip commented-out lines (both // and /* */ style)
            stripped = line.strip()
            if stripped.startswith('//') or stripped.startswith('/*') or stripped.startswith('*'):
                continue

            # Skip AP_SUBGROUPINFO (handled separately)
            if 'AP_SUBGROUPINFO' in line:
                continue

            match = self.GROUPINFO_PATTERN.search(line)
            if match:
                param_name = match.group(1)
                is_vector3f = param_name in vector3f_params

                params.append(ParamInfo(
                    name=param_name,
                    file_path=str(file_path),
                    line_number=line_num,
                    class_name=class_name,
                    prefix_chain=prefix_chain.copy(),
                    is_vector3f=is_vector3f
                ))

        return params

    def validate_class_recursive(self, class_name: str, prefix_chain: List[str],
                                 visited: Set[str] = None) -> List[ParamInfo]:
        '''recursively validate all parameters in a class and its subgroups'''
        if visited is None:
            visited = set()

        # Prevent infinite recursion
        chain_key = f"{class_name}:{''.join(prefix_chain)}"
        if chain_key in visited:
            return []
        visited.add(chain_key)

        self.log(f"Validating class: {class_name} with prefix chain: {''.join(prefix_chain)}")

        all_params = []

        # Find the file containing this class's var_info
        file_path = self.find_class_file(class_name)
        if not file_path:
            self.log(f"  Warning: Could not find var_info for {class_name}")
            return all_params

        # Parse direct parameters in this class
        params = self.parse_parameters(class_name, file_path, prefix_chain)
        all_params.extend(params)

        # Parse and recurse into subgroups
        subgroups = self.parse_subgroups(class_name, file_path)
        for subgroup in subgroups:
            new_prefix_chain = prefix_chain + [subgroup.prefix]
            sub_params = self.validate_class_recursive(
                subgroup.class_name,
                new_prefix_chain,
                visited
            )
            all_params.extend(sub_params)

        return all_params

    def validate_all(self) -> Tuple[List[ParamInfo], List[ParamInfo]]:
        '''validate all parameter names in the codebase'''
        if self.verbose:
            print("ArduPilot Parameter Name Length Validator")
            print("=" * 60)
            print(f"Maximum parameter name length: {self.MAX_NAME_LENGTH} characters\n")

        # Step 1: Parse top-level prefixes
        self.parse_top_level_prefixes()
        if self.verbose:
            print(f"Found {len(self.top_level_prefixes)} top-level parameter groups\n")

        # Step 2: Validate each top-level class
        all_params = []
        for class_name, prefix in self.top_level_prefixes.items():
            params = self.validate_class_recursive(class_name, [prefix])
            all_params.extend(params)

        # Step 3: Check lengths and categorize
        for param in all_params:
            length = param.full_length()

            if length > self.MAX_NAME_LENGTH:
                self.errors.append(param)
            elif length == self.MAX_NAME_LENGTH and self.show_warnings:
                self.warnings.append(param)

        return self.errors, self.warnings

    def print_results(self):
        """Print validation results"""
        print("\nValidation Results:")
        print("=" * 60)

        if self.warnings and self.show_warnings:
            print(f"\nWARNINGS: {len(self.warnings)} parameter(s) at exactly {self.MAX_NAME_LENGTH} characters:")
            for param in sorted(self.warnings, key=lambda p: p.full_name()):
                print(f"  {param.full_name()} ({param.full_length()} chars)")
                print(f"    File: {param.file_path}:{param.line_number}")
                print(f"    Prefix: {''.join(param.prefix_chain[:-1]) if len(param.prefix_chain) > 1 else '(none)'}")
                print(f"    Param: {param.name}")
                if param.is_vector3f:
                    print("    Note: Vector3f parameter (adds 2 chars for _X/_Y/_Z suffix)")
                print()

        if self.errors:
            print(f"\nERROR: {len(self.errors)} parameter(s) exceed {self.MAX_NAME_LENGTH} character limit:")
            for param in sorted(self.errors, key=lambda p: p.full_length(), reverse=True):
                full_name = param.full_name()
                print(f"  {full_name} ({param.full_length()} chars) - EXCEEDS LIMIT!")
                print(f"    File: {param.file_path}:{param.line_number}")
                print(f"    Prefix: {''.join(param.prefix_chain)} ({sum(len(p) for p in param.prefix_chain)} chars)")
                print(f"    Param: {param.name} ({len(param.name)} chars)")
                if param.is_vector3f:
                    print("    Note: Vector3f parameter (adds 2 chars for _X/_Y/_Z suffix)")

            print(f"\nParameter validation failed: {len(self.errors)} parameter(s) exceed limit")
            return False
        else:
            if self.verbose or self.warnings:
                print(f"\nParameter validation passed: all names within {self.MAX_NAME_LENGTH} character limit")
                if self.warnings and not self.show_warnings:
                    print(f"  ({len(self.warnings)} at exactly {self.MAX_NAME_LENGTH} chars - use --warnings to see details)")
            return True


def main():
    parser = argparse.ArgumentParser(
        description='Validate ArduPilot parameter name lengths',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument(
        'root_dir',
        nargs='?',
        default='.',
        help='ArduPilot root directory (default: current directory)'
    )
    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        help='Enable verbose output'
    )
    parser.add_argument(
        '-w', '--warnings',
        action='store_true',
        help='Show warnings for parameters at exactly 16 characters'
    )

    args = parser.parse_args()

    # Validate root directory
    root_path = Path(args.root_dir).resolve()
    if not root_path.exists():
        print(f"Error: Directory not found: {root_path}", file=sys.stderr)
        sys.exit(1)

    # Check if this looks like an ArduPilot directory
    if not (root_path / 'libraries' / 'AP_Param').exists():
        print(f"Error: {root_path} does not appear to be an ArduPilot root directory", file=sys.stderr)
        print("       (missing libraries/AP_Param)", file=sys.stderr)
        sys.exit(1)

    # Run validation
    validator = ParamValidator(root_path, verbose=args.verbose, show_warnings=args.warnings)
    errors, warnings = validator.validate_all()

    # Print results
    success = validator.print_results()

    # Exit with appropriate code
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
