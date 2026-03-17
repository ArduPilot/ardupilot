#!/usr/bin/env python3

from __future__ import annotations

'''
Check PR branch commit conventions and markdown linting.

Validates:
  - no merge commits
  - no fixup! commits
  - commit messages have a well-formed subsystem prefix before ':'
  - commit subject lines are <= 160 characters
  - changed markdown files pass markdownlint-cli2

AP_FLAKE8_CLEAN
'''

import argparse
import os
import re
import subprocess
import sys

import build_script_base

DOCS_URL = "https://ardupilot.org/dev/docs/submitting-patches-back-to-master.html"
MAX_SUBJECT_LEN = 160
BLACKLISTED_PREFIXES = {
    "DEBUG",
    "DRAFT",
    "TEMP",
    "TMP",
    "WIP",
}
# spaces and quotes allowed to support Revert commits e.g. 'Revert "AP_Periph: ...'
PREFIX_RE = re.compile(r'^[-A-Za-z0-9._/" ]+$')

# Enable colour when attached to a terminal or running under GitHub Actions
_colour = sys.stdout.isatty() or os.environ.get('GITHUB_ACTIONS') == 'true'
_GREEN = '\033[32m' if _colour else ''
_RED = '\033[31m' if _colour else ''
_YELLOW = '\033[33m' if _colour else ''
_RESET = '\033[0m' if _colour else ''

PASS = f"{_GREEN}✓{_RESET}"
FAIL = f"{_RED}✗{_RESET}"
SKIP = f"{_YELLOW}~{_RESET}"


class CheckBranchConventions(build_script_base.BuildScriptBase):

    DEFAULT_UPSTREAM = "origin/master"

    def __init__(self, base_branch: str | None = None) -> None:
        super().__init__()
        self.base_branch = base_branch

    def progress_prefix(self) -> str:
        return "CBC"

    def run_git(self, args, show_output=True, source_dir=None):
        cmd_list = ["git"] + list(args)
        return self.run_program(
            "SCB-GIT", cmd_list,
            show_output=show_output, show_command=False, cwd=source_dir,
        )

    def check_merge_commits(self) -> bool:
        merge_commits = self.run_git(
            ["log", f"{self.base_branch}..HEAD", "--merges", "--oneline"],
            show_output=False,
        ).strip()
        if merge_commits:
            print(f"{FAIL} Merge commits are not allowed:")
            for line in merge_commits.splitlines():
                print(f"         {line}")
            print(f"       See: {DOCS_URL}")
            return False
        print(f"{PASS} No merge commits.")
        return True

    def check_fixup_commits(self, commits: str) -> bool:
        bad = [line for line in commits.splitlines() if "fixup!" in line]
        if bad:
            print(f"{FAIL} fixup! commits are not allowed:")
            for line in bad:
                print(f"         {line}")
            print(f"       See: {DOCS_URL}")
            return False
        print(f"{PASS} No fixup! commits.")
        return True

    def check_commit_messages(self, commits: str) -> bool:
        ok = True
        for line in commits.splitlines():
            if not line.strip():
                continue
            # strip leading hash from --oneline format
            subject = line.split(" ", 1)[1] if " " in line else line
            if ":" not in subject:
                print(f"{FAIL} Missing subsystem prefix: {line}")
                print(f"       Reword to e.g. 'AP_Compass: {subject}'")
                print(f"       See: {DOCS_URL}")
                ok = False
                continue
            prefix = subject.split(":")[0]
            if prefix.strip().upper() in BLACKLISTED_PREFIXES:
                print(f"{FAIL} Bad subsystem prefix '{prefix}': {line}")
                print(f"       See: {DOCS_URL}")
                ok = False
            if not PREFIX_RE.match(prefix):
                print(f"{FAIL} Malformed subsystem prefix '{prefix}': {line}")
                print("       Prefix must contain only letters, digits, '.', '_', '/', '-', spaces, quotes.")
                print(f"       See: {DOCS_URL}")
                ok = False
        if ok:
            print(f"{PASS} All commit messages have well-formed subsystem tags.")
        return ok

    def check_commit_lengths(self, commits: str) -> bool:
        ok = True
        for line in commits.splitlines():
            if not line.strip():
                continue
            if len(line) > MAX_SUBJECT_LEN:
                print(f"{FAIL} Subject too long ({len(line)} chars, limit {MAX_SUBJECT_LEN}): {line}")
                ok = False
        if ok:
            print(f"{PASS} All commit subject lines within {MAX_SUBJECT_LEN} characters.")
        return ok

    def check_markdown(self) -> bool:
        changed_md = self.run_git(
            ["diff", "--name-only", "--diff-filter=AM",
             f"{self.base_branch}...HEAD", "--", "*.md"],
            show_output=False,
        ).strip()
        if not changed_md:
            print(f"{PASS} No markdown files changed.")
            return True

        try:
            result = subprocess.run(["markdownlint-cli2"] + changed_md.splitlines())
        except FileNotFoundError:
            print(f"{SKIP} markdownlint-cli2 not installed.")
            return True
        if result.returncode != 0:
            print(f"{FAIL} Markdown linting errors found (see above).")
            return False

        print(f"{PASS} Markdown files pass linting.")
        return True

    def run(self) -> None:
        if self.base_branch is None:
            current = self.find_current_git_branch_or_sha1()
            self.base_branch = self.find_git_branch_merge_base(current, self.DEFAULT_UPSTREAM)
            self.progress(f"Using merge base with {self.DEFAULT_UPSTREAM}: {self.base_branch}")

        commits = self.run_git(
            ["log", f"{self.base_branch}..HEAD", "--oneline"],
            show_output=False,
        ).strip()

        n = len(commits.splitlines()) if commits else 0
        print(f"\nChecking {n} commit(s) since {self.base_branch}...\n")

        results = [
            self.check_merge_commits(),
            self.check_fixup_commits(commits),
            self.check_commit_messages(commits),
            self.check_commit_lengths(commits),
            self.check_markdown(),
        ]

        failures = results.count(False)
        print(f"\n{'All checks passed.' if not failures else f'{failures} check(s) failed.'}")
        sys.exit(0 if all(results) else 1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Check PR branch commit conventions and markdown linting",
    )
    parser.add_argument(
        "--base-branch",
        default=None,
        help="Upstream base branch or commit to compare against "
             "(default: merge base of HEAD with origin/master)",
    )
    args = parser.parse_args()
    CheckBranchConventions(args.base_branch).run()
