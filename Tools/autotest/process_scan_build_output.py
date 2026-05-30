#!/usr/bin/env python3

'''
Tool to process the output of clang-scan-build.

It parses the .plist reports emitted by "scan-build -plist-html", applies
the suppressions and directory excludes from scan_build_suppressions.py,
and fails if either:

  - any finding is not covered by a suppression (new problem introduced), or
  - any suppression did not match a finding (suppression is now stale and
    must be removed, e.g. because the underlying issue was fixed).

Both conditions must be clean for the check to pass.  There is no separate
expected-count to maintain: the suppression list IS the complete record of
accepted findings.

Findings are matched on the pair
(repository-relative file, issue_hash_content_of_line_in_context).  The
hash is independent of absolute line numbers, so unrelated edits do not
disturb existing suppressions.

AP_FLAKE8_CLEAN
'''

import os
import pathlib
import plistlib
import re
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scan_build_suppressions import EXCLUDE_DIRS  # noqa: E402
from scan_build_suppressions import SUPPORTED_VERSIONS  # noqa: E402
from scan_build_suppressions import SUPPRESSIONS  # noqa: E402


class ProcessScanBuildOutput():
    def __init__(self, stdout_filepath, suppression_stubs=False):
        self.stdout_filepath = stdout_filepath
        self.suppression_stubs = suppression_stubs
        self.analyser_version = None
        self.repo_root = os.path.realpath(
            os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))

    def progress(self, msg):
        print(f"psbo: {msg}")

    def stdout_text(self):
        return pathlib.Path(self.stdout_filepath).read_text()

    def find_scan_build_dir(self):
        '''report directory named in captured stdout, or None.

        A run which finds nothing does not write a report directory and so
        names none; that is a legitimate result, not an error, and is told
        apart from a broken run by analyser_version_from_stdout().
        '''
        scan_build_dir = None
        for line in self.stdout_text().split("\n"):
            m = re.match(r".*Run 'scan-view (.+)'", line)
            if m is not None:
                scan_build_dir = m.group(1)
        return scan_build_dir

    def analyser_version_from_stdout(self):
        '''clang major version scan-build reported using, or None.

        scan-build logs the analyser it selected, which tells us the
        version even when the run produced no reports at all.
        '''
        for line in self.stdout_text().split("\n"):
            m = re.match(r".*Using '/usr/lib/llvm-(\d+)/bin/clang' for static analysis", line)
            if m is not None:
                return int(m.group(1))
        return None

    def to_repo_relative(self, path):
        '''reduce a path as stored in a plist to a repository-relative path'''
        p = os.path.normpath(path)
        if os.path.isabs(p):
            if p.startswith(self.repo_root + os.sep):
                return os.path.relpath(p, self.repo_root)
            return p
        # relative path (e.g. build/sitl/../../libraries/X or ../../libraries/X)
        parts = p.split(os.sep)
        while parts and parts[0] in ('.', '..'):
            parts.pop(0)
        return os.sep.join(parts)

    def findings_from_plists(self, scan_build_dir):
        '''return the dict of unique (file, issue_hash) -> bug_type findings.

        scan-build writes one .plist per translation unit, so a finding in
        a header or inline function appears in several plists; de-duping on
        (file, issue_hash) matches scan-build's own report count.
        '''
        findings = {}
        for plist_path in sorted(pathlib.Path(scan_build_dir).glob('*.plist')):
            with open(plist_path, 'rb') as f:
                data = plistlib.load(f)
            if self.analyser_version is None:
                self.analyser_version = self.version_from_plist(data)
            files = data.get('files', [])
            for diag in data.get('diagnostics', []):
                file_index = diag['location']['file']
                file_rel = self.to_repo_relative(files[file_index])
                issue_hash = diag.get('issue_hash_content_of_line_in_context', '')
                findings[(file_rel, issue_hash)] = diag.get('type', '')
        return findings

    def version_from_plist(self, data):
        '''return the clang major version a plist was produced by, or None.

        The issue hashes are not stable across clang versions for every
        finding, so a suppression carries one hash per version and we need
        to know which one this run should be keyed on.
        '''
        m = re.search(r'clang version (\d+)', data.get('clang_version', ''))
        if m is None:
            return None
        return int(m.group(1))

    def is_excluded(self, file_rel):
        return any(file_rel.startswith(d) for d in EXCLUDE_DIRS)

    def archive_rename(self, scan_build_dir):
        '''in CI, move the report dir to a fixed path so it can be archived.

        Done before the pass/fail check so the artifacts are available even
        when the check fails.  A no-op outside CI.
        '''
        dest = "/__w/ardupilot/ardupilot/tmp/scan-build"
        if not os.path.isdir(os.path.dirname(dest)):
            return scan_build_dir
        self.progress(f"Renaming {scan_build_dir} to {dest}")
        shutil.move(scan_build_dir, dest)
        new_stdout_filepath = os.path.join(dest, os.path.basename(self.stdout_filepath))
        self.progress(f"Copying {self.stdout_filepath} to {new_stdout_filepath}")
        shutil.move(self.stdout_filepath, new_stdout_filepath)
        self.stdout_filepath = new_stdout_filepath
        return dest

    def run(self):
        # the version is taken from stdout first so that it is known even
        # for a run which produced no reports
        self.analyser_version = self.analyser_version_from_stdout()

        scan_build_dir = self.find_scan_build_dir()
        if scan_build_dir is None:
            # no report directory: either nothing was found, or scan-build
            # never ran.  The version line proves the analyser ran.
            if self.analyser_version is None:
                self.progress("FAIL: scan-build does not appear to have run: "
                              "no report directory and no analyser named in its output")
                sys.exit(1)
            self.progress("No report directory: the analyser found nothing")
            findings = {}
        else:
            if not os.path.isdir(scan_build_dir):
                self.progress(f"FAIL: report directory {scan_build_dir} does not exist")
                sys.exit(1)
            scan_build_dir = self.archive_rename(scan_build_dir)
            findings = self.findings_from_plists(scan_build_dir)
            if not list(pathlib.Path(scan_build_dir).glob('*.plist')):
                self.progress(f"FAIL: report directory {scan_build_dir} contains no .plist reports; "
                              "the analysis did not produce output")
                sys.exit(1)

        if self.analyser_version is None:
            self.progress("FAIL: could not determine the clang version used for the analysis")
            sys.exit(1)
        if self.analyser_version not in SUPPORTED_VERSIONS:
            self.progress(f"FAIL: clang {self.analyser_version} is not one of the versions this "
                          f"suppression list records hashes for ({', '.join(str(v) for v in SUPPORTED_VERSIONS)}).  "
                          "Harvest its hashes before using it, or the ratchet silently checks nothing.")
            sys.exit(1)

        # --exclude keeps these out of the HTML reports but not the plists,
        # so drop the excluded directories here too.
        excluded = {k for k in findings if self.is_excluded(k[0])}
        in_scope = {k: t for k, t in findings.items() if k not in excluded}

        # a duplicate is the same (file, hash) claimed by two different
        # entries; that is a list-authoring error whichever version it is
        # recorded under, so every recorded version is examined here.
        seen = {}
        duplicates = []
        for index, (f, hashes, reason) in enumerate(SUPPRESSIONS):
            for _version, h in sorted(hashes.items()):
                key = (f, h)
                if seen.get(key, index) != index:
                    duplicates.append(key)
                    continue
                seen[key] = index

        # a finding is suppressed only by the hash recorded for the version
        # actually run.  Accepting another version's hash would silently
        # cover a finding this list has never recorded for this analyser,
        # and would leave the entry looking recorded when it is not.
        suppress_keys = {}
        for index, (f, hashes, reason) in enumerate(SUPPRESSIONS):
            h = hashes.get(self.analyser_version)
            if h is not None:
                suppress_keys[(f, h)] = index
        matched = set()
        remaining = []
        for (file_rel, issue_hash), bug_type in in_scope.items():
            key = (file_rel, issue_hash)
            if key in suppress_keys:
                matched.add(suppress_keys[key])
                continue
            remaining.append((file_rel, issue_hash, bug_type))

        self.progress(f"Analyser version: clang {self.analyser_version}")
        self.progress(f"Unique findings: {len(findings)}")
        self.progress(f"Excluded directories: {len(excluded)}")
        self.progress(f"In scope: {len(in_scope)}")
        self.progress(f"Suppressed: {len(matched)}")
        self.progress(f"Remaining: {len(remaining)}")

        # suppressions that matched no finding are stale: the underlying issue
        # was fixed (or the entry was mis-keyed).  Remove them from
        # scan_build_suppressions.py so the list stays accurate.
        # An entry is stale only when it carries a hash for the version we
        # ran and that hash matched nothing.  Not every finding exists in
        # every clang release - the checkers change - so an entry with no
        # hash for this version simply does not apply to this run and must
        # not be reported as stale.
        stale = []
        not_applicable = 0
        for index, entry in enumerate(SUPPRESSIONS):
            if index in matched:
                continue
            if self.analyser_version not in entry[1]:
                not_applicable += 1
                continue
            stale.append(entry)
        self.progress(f"Not recorded for clang {self.analyser_version}: {not_applicable}")
        for (f, hashes, r) in stale:
            self.progress(f"STALE suppression (finding no longer exists): {f} {sorted(hashes.values())} ({r})")

        if self.suppression_stubs:
            # emit each remaining finding as a paste-ready SUPPRESSIONS entry
            # (replace 'REASON HERE' with the reviewed justification).  Printed
            # without the psbo: prefix so the lines can be copied directly.
            version = self.analyser_version
            for (file_rel, issue_hash, bug_type) in sorted(remaining):
                print(f"    ('{file_rel}', {{{version}: '{issue_hash}'}}, 'REASON HERE'),  # {bug_type}")
        else:
            for (file_rel, issue_hash, bug_type) in sorted(remaining):
                self.progress(f"  {file_rel}: {bug_type} [{issue_hash}]")

        failed = False
        if remaining:
            self.progress(f"FAIL: {len(remaining)} finding(s) not in suppression list; "
                          "fix them or add to scan_build_suppressions.py")
            failed = True
        if stale:
            self.progress(f"FAIL: {len(stale)} stale suppression(s); "
                          "remove them from scan_build_suppressions.py")
            failed = True
        if duplicates:
            for (f, h) in duplicates:
                self.progress(f"DUPLICATE suppression: {f} {h}")
            self.progress(f"FAIL: {len(duplicates)} duplicate suppression(s); "
                          "remove them from scan_build_suppressions.py")
            failed = True
        if failed:
            sys.exit(1)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='tool to post-process output from clang-scan-build in CI')
    parser.add_argument('clang_scan_build_stdout', default=None, help='file containing stdout from clang-scan-build process')
    parser.add_argument('--suppression-stubs', action='store_true', help='print each remaining finding as a paste-ready scan_build_suppressions.py entry')  # noqa: E501

    args = parser.parse_args()

    p = ProcessScanBuildOutput(args.clang_scan_build_stdout, suppression_stubs=args.suppression_stubs)
    p.run()
