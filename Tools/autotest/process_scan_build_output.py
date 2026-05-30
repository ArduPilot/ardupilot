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
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scan_build_suppressions import EXCLUDE_DIRS, SUPPRESSIONS  # noqa: E402


class ProcessScanBuildOutput():
    def __init__(self, stdout_filepath, suppression_stubs=False):
        self.stdout_filepath = stdout_filepath
        self.suppression_stubs = suppression_stubs
        self.repo_root = os.path.realpath(
            os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))

    def progress(self, msg):
        print(f"psbo: {msg}")

    def find_scan_build_dir(self):
        '''extract the scan-build output directory from captured stdout'''
        t = pathlib.Path(self.stdout_filepath).read_text()
        scan_build_dir = None
        for line in t.split("\n"):
            m = re.match(r".*Run 'scan-view (.+)'", line)
            if m is not None:
                scan_build_dir = m.group(1)
        if scan_build_dir is None:
            raise ValueError("Did not find scan-build output directory in output")
        return scan_build_dir

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
            files = data.get('files', [])
            for diag in data.get('diagnostics', []):
                file_index = diag['location']['file']
                file_rel = self.to_repo_relative(files[file_index])
                issue_hash = diag.get('issue_hash_content_of_line_in_context', '')
                findings[(file_rel, issue_hash)] = diag.get('type', '')
        return findings

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
        os.rename(scan_build_dir, dest)
        new_stdout_filepath = os.path.join(dest, os.path.basename(self.stdout_filepath))
        self.progress(f"Copying {self.stdout_filepath} to {new_stdout_filepath}")
        os.rename(self.stdout_filepath, new_stdout_filepath)
        self.stdout_filepath = new_stdout_filepath
        return dest

    def run(self):
        scan_build_dir = self.find_scan_build_dir()
        scan_build_dir = self.archive_rename(scan_build_dir)
        findings = self.findings_from_plists(scan_build_dir)

        # --exclude keeps these out of the HTML reports but not the plists,
        # so drop the excluded directories here too.
        excluded = {k for k in findings if self.is_excluded(k[0])}
        in_scope = {k: t for k, t in findings.items() if k not in excluded}

        suppress_keys = {(f, h): reason for (f, h, reason) in SUPPRESSIONS}
        matched = set()
        remaining = []
        for (file_rel, issue_hash), bug_type in in_scope.items():
            key = (file_rel, issue_hash)
            if key in suppress_keys:
                matched.add(key)
                continue
            remaining.append((file_rel, issue_hash, bug_type))

        self.progress(f"Unique findings: {len(findings)}")
        self.progress(f"Excluded directories: {len(excluded)}")
        self.progress(f"In scope: {len(in_scope)}")
        self.progress(f"Suppressed: {len(matched)}")
        self.progress(f"Remaining: {len(remaining)}")

        # suppressions that matched no finding are stale: the underlying issue
        # was fixed (or the entry was mis-keyed).  Remove them from
        # scan_build_suppressions.py so the list stays accurate.
        stale = [(f, h, r) for (f, h, r) in SUPPRESSIONS if (f, h) not in matched]
        for (f, h, r) in stale:
            self.progress(f"STALE suppression (finding no longer exists): {f} {h} ({r})")

        if self.suppression_stubs:
            # emit each remaining finding as a paste-ready SUPPRESSIONS entry
            # (replace 'REASON HERE' with the reviewed justification).  Printed
            # without the psbo: prefix so the lines can be copied directly.
            for (file_rel, issue_hash, bug_type) in sorted(remaining):
                print(f"    ('{file_rel}', '{issue_hash}', 'REASON HERE'),  # {bug_type}")
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
