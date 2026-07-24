#!/usr/bin/env python3

'''
The definitive list of subsystem prefixes allowed in ArduPilot commit
messages, and the mapping from changed file paths to the subsystem(s) they
may belong to.

This is the single source of truth used by:
  - Tools/scripts/check_branch_conventions.py  (CI: strict allow-list plus a
    check that every file in a commit belongs to the declared subsystem)
  - Tools/gittools/git-subsystems-split         (split a commit by subsystem)
  - Tools/gittools/pre-commit-subsystems.py     (sample local hook)

A subsystem is "allowed" if it is:
  - the name of a directory in libraries/ (dynamic; e.g. AP_GPS, GCS_MAVLink),
  - a directory created by the commit under libraries/ (a brand-new library may
    be its own subsystem), or
  - one of the CURATED_SUBSYSTEMS below (vehicles, tooling and other prefixes
    that are not backed by a libraries/ directory).

Many files can legitimately be committed under more than one subsystem name --
an autotest is sometimes "autotest" and sometimes "Tools"; a ChibiOS hwdef file
is sometimes "hwdef" and sometimes "AP_HAL_ChibiOS".  subsystems_for_path()
therefore returns an ordered list of candidates (most conventional first).

Instantiate AllowedSubsystems and call its methods.  Run this file directly to
print every valid subsystem name, one per line.

AP_FLAKE8_CLEAN
'''

import difflib
import os
import sys

from pathlib import Path


class AllowedSubsystems(object):

    # Subsystem names that are allowed but are not the name of a libraries/
    # directory: vehicles (short conventional prefix and long directory name),
    # tooling and infrastructure.
    CURATED_SUBSYSTEMS = {
        # vehicles -- conventional short prefixes
        'Plane',
        'Copter',
        'Rover',
        'Sub',
        'Blimp',
        'Tracker',
        # vehicles -- directory names (also accepted)
        'ArduPlane',
        'ArduCopter',
        'ArduSub',
        'AntennaTracker',
        # tooling / infrastructure
        'Tools',
        'autotest',
        'waf',
        'hwdef',
        'ci',
        '.github',
        '.vscode',
        'bootloaders',
        'benchmarks',
        'docs',
        'tests',
        'modules',
        'Replay',
        'AP_Periph',
        'AP_Bootloader',
        # class / sub-library prefixes in common use that are not their own
        # libraries/ directory
        'AC_PosControl',
        'AC_Circle',
        'AP_MotorsHeli',
        'mavlink',
    }

    # Vehicle top-level directory -> ordered candidate subsystems.
    VEHICLE_DIR_SUBSYSTEMS = {
        'ArduPlane': ['Plane', 'ArduPlane'],
        'ArduCopter': ['Copter', 'ArduCopter'],
        'ArduSub': ['Sub', 'ArduSub'],
        'AntennaTracker': ['Tracker', 'AntennaTracker'],
        'Rover': ['Rover'],
        'Blimp': ['Blimp'],
    }

    # Ordered, most-specific-first rules for paths under a top-level directory.
    # Each entry is (path prefix, ordered candidate subsystems); the first
    # prefix that matches the path wins.  These capture the cases where a file
    # lives physically inside one directory but is conventionally committed
    # under a different (or additional) subsystem name.
    SPECIAL_DIR_RULES = [
        ('Tools/autotest/', ['autotest', 'Tools']),
        ('Tools/ardupilotwaf/', ['waf']),
        ('Tools/AP_Periph/', ['AP_Periph']),
        ('Tools/AP_Bootloader/', ['AP_Bootloader']),
        ('Tools/bootloaders/', ['bootloaders']),
        ('Tools/Replay/', ['Replay']),
    ]

    # Top-level directory (other than vehicles, libraries and the special cases
    # above) -> ordered candidate subsystems.
    TOPLEVEL_DIR_SUBSYSTEMS = {
        'Tools': ['Tools'],
        'modules': ['modules'],
        'docs': ['docs'],
        'benchmarks': ['benchmarks', 'Tools'],
        'tests': ['tests', 'Tools'],
        '.github': ['.github', 'ci'],
        '.vscode': ['.vscode'],
    }

    # Files that live in the repository root -> ordered candidate subsystems.
    # Root files are cross-cutting; map them explicitly so a change to one has a
    # well-defined home rather than falling through to "no subsystem".
    ROOT_FILE_SUBSYSTEMS = {
        'waf': ['waf', 'Tools'],
        'wscript': ['waf', 'Tools'],
        'Makefile': ['waf', 'Tools'],
        'Dockerfile': ['waf', 'Tools'],
        '.dockerignore': ['waf', 'Tools'],
        'Vagrantfile': ['Tools'],
        'pyproject.toml': ['Tools'],
        'Doxyfile.in': ['Tools'],
        '.flake8': ['Tools'],
        '.editorconfig': ['Tools'],
        '.dir-locals.el': ['Tools'],
        '.shellcheckrc': ['Tools'],
        '.valgrindrc': ['Tools'],
        '.valgrind-suppressions': ['Tools'],
        '.gitattributes': ['Tools'],
        '.gitignore': ['Tools'],
        '.git-blame-ignore-revs': ['Tools'],
        '.markdownlint-cli2.jsonc': ['Tools'],
        '.gitmodules': ['modules', 'Tools'],
        '.pre-commit-config.yaml': ['ci', '.github', 'Tools'],
        'AGENTS.md': ['Tools'],
        'README.md': ['Tools'],
        'BUILD.md': ['Tools'],
        'CODE_OF_CONDUCT.md': ['Tools'],
        'COPYING.txt': ['Tools'],
    }

    def __init__(self, repo_root=None):
        '''repo_root is the ardupilot checkout root; when omitted it is
        derived from this file's location (Tools/scripts/allowed_subsystems.py).
        '''
        if repo_root is None:
            repo_root = str(Path(__file__).resolve().parents[2])
        self.repo_root = repo_root

    def library_dirs(self):
        '''return the set of immediate subdirectory names of libraries/'''
        libraries = os.path.join(self.repo_root, 'libraries')
        try:
            entries = os.listdir(libraries)
        except OSError:
            return set()
        return {
            name for name in entries
            if os.path.isdir(os.path.join(libraries, name))
        }

    def allowed_subsystems(self, created_dirs=()):
        '''return the full set of allowed subsystem names.

        created_dirs is an iterable of libraries/ subdirectory names that a
        commit creates; they are allowed even if they do not yet exist on disk.
        '''
        return (self.library_dirs()
                | set(self.CURATED_SUBSYSTEMS)
                | set(created_dirs))

    def subsystems_for_path(self, path):
        '''return the ordered list of subsystems the given repo-relative path
        may belong to, most conventional first.  An empty list means no rule
        matches; callers should treat that as "needs a mapping rule".
        '''
        # normalise a leading "./"
        if path.startswith('./'):
            path = path[2:]

        # repository root files
        if '/' not in path:
            return list(self.ROOT_FILE_SUBSYSTEMS.get(path, []))

        parts = path.split('/')

        # files under libraries/
        if parts[0] == 'libraries':
            # only the known colcon build marker may sit directly in
            # libraries/; anything else at this depth is almost certainly a
            # mistake, so leave it unmapped to trip the check
            if len(parts) == 2:
                if parts[1] == 'COLCON_IGNORE':
                    return ['Tools']
                return []
            lib = parts[1]
            # libraries/<HAL>/hwdef/... belongs to hwdef -- the deepest
            # library directory name is the conventional tag, not the HAL
            # itself -- but the HAL name is still accepted as a fallback.
            if len(parts) >= 4 and parts[2] == 'hwdef':
                return ['hwdef', lib]
            return [lib]

        # most-specific-first special directory rules
        for prefix, subsystems in self.SPECIAL_DIR_RULES:
            if path.startswith(prefix):
                return list(subsystems)

        # vehicle directories
        if parts[0] in self.VEHICLE_DIR_SUBSYSTEMS:
            return list(self.VEHICLE_DIR_SUBSYSTEMS[parts[0]])

        # other known top-level directories
        if parts[0] in self.TOPLEVEL_DIR_SUBSYSTEMS:
            return list(self.TOPLEVEL_DIR_SUBSYSTEMS[parts[0]])

        return []

    def primary_subsystem(self, path):
        '''return the single canonical subsystem for a path (the first
        candidate), or None if no rule matches.
        '''
        candidates = self.subsystems_for_path(path)
        return candidates[0] if candidates else None

    def suggest_subsystem(self, prefix, allowed):
        '''for a prefix that is not allowed, return the closest allowed
        subsystem name to suggest, or None.

        A case/spelling variant (e.g. "Autotest", "AP_Rangefinder") returns the
        canonical spelling; a near-miss typo (e.g. "AP_Bootlaoder") returns the
        closest match.  allowed is the collection of currently allowed names.
        '''
        allowed = set(allowed)
        # exact match ignoring case -> canonical spelling
        lower_map = {}
        for name in allowed:
            lower_map.setdefault(name.lower(), name)
        canonical = lower_map.get(prefix.lower())
        if canonical is not None:
            return canonical
        # otherwise a close typo, if any
        matches = difflib.get_close_matches(prefix, sorted(allowed),
                                            n=1, cutoff=0.8)
        return matches[0] if matches else None

    def main(self):
        '''print every valid subsystem name, one per line, sorted'''
        for name in sorted(self.allowed_subsystems()):
            print(name)
        return 0


if __name__ == '__main__':
    sys.exit(AllowedSubsystems().main())
