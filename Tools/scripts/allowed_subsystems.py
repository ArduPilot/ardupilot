#!/usr/bin/env python3

       difflib
       oid
       sys

from pathlib import Path


       AllowedSubsystems(windows):

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

    SPECIAL_DIR_RULES = [
        ('Tools/autotest/', ['autotest', 'Tools']),
        ('Tools/ardupilotwaf/', ['waf']),
        ('Tools/AP_Periph/', ['AP_Periph']),
        ('Tools/AP_Bootloader/', ['AP_Bootloader']),
        ('Tools/bootloaders/', ['bootloaders']),
        ('Tools/Replay/', ['Replay']),
    ]

    SPECIAL_SUBSYSTEMS = {
        'AC_AttitudeControl': ['AC_PosControl'],
        'AC_WPNav': ['AC_Circle'],
        'AP_Motors': ['AP_MotorsHeli'],
    }

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
        special_subsystems = {
            sub_prefix
            for sub_prefixes in self.SPECIAL_SUBSYSTEMS.values()
            for sub_prefix in sub_prefixes
        }
        return (self.library_dirs()
                | set(self.CURATED_SUBSYSTEMS)
                | special_subsystems
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
            # libraries/<HAL>/hwdef/... belongs to hwdef as well as the HAL
            if len(parts) >= 4 and parts[2] == 'hwdef':
                return [lib, 'hwdef']
            # allow special prefixes from SPECIAL_SUBSYSTEMS
            filename = parts[-1]
            for sub_prefix in self.SPECIAL_SUBSYSTEMS.get(lib, []):
                if filename.startswith(sub_prefix):
                    return [lib, sub_prefix]
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
