#!/usr/bin/env python3

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

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import urllib.error
import urllib.request

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

    def check_author_emails(self) -> bool:
        emails = self.run_git(
            ["log", f"{self.base_branch}..HEAD", "--format=%ae"],
            show_output=False,
        ).strip()
        bad = []
        for email in emails.splitlines():
            if "example.com" in email:
                bad.append(email)
        if bad:
            print(f"{FAIL} Author email(s) with example.com are not allowed:")
            for email in bad:
                print(f"         {email}")
            return False
        print(f"{PASS} No unacceptable author emails.")
        return True

    def get_submodule_paths(self) -> set:
        '''parse .gitmodules and return the set of submodule paths'''
        root = self.run_git(['rev-parse', '--show-toplevel'], show_output=False).strip()
        gitmodules = os.path.join(root, '.gitmodules')
        paths = set()
        output = self.run_git(
            ['config', '--file', gitmodules, '--get-regexp', 'path'],
            show_output=False,
        )
        for line in output.splitlines():
            # each line looks like: submodule.modules/mavlink.path modules/mavlink
            parts = line.split()
            if len(parts) == 2:
                paths.add(parts[1])
        return paths

    def get_submodule_urls(self) -> dict:
        '''parse .gitmodules and return a mapping of submodule path to canonical url'''
        root = self.run_git(['rev-parse', '--show-toplevel'], show_output=False).strip()
        gitmodules = os.path.join(root, '.gitmodules')
        path_out = self.run_git(
            ['config', '--file', gitmodules, '--get-regexp', 'path'],
            show_output=False,
        )
        name_to_path = {}
        for line in path_out.splitlines():
            # submodule.modules/mavlink.path modules/mavlink
            parts = line.split()
            if len(parts) == 2:
                name = parts[0][len('submodule.'):-len('.path')]
                name_to_path[name] = parts[1]
        url_out = self.run_git(
            ['config', '--file', gitmodules, '--get-regexp', 'url'],
            show_output=False,
        )
        result = {}
        for line in url_out.splitlines():
            parts = line.split()
            if len(parts) == 2:
                name = parts[0][len('submodule.'):-len('.url')]
                if name in name_to_path:
                    result[name_to_path[name]] = parts[1]
        return result

    def get_changed_paths_for_commit(self, commit: str) -> list:
        '''return the list of paths changed in a single commit'''
        output = self.run_git(
            ['diff-tree', '--no-commit-id', '-r', '--name-only', commit],
            show_output=False,
        )
        paths = []
        for line in output.splitlines():
            line = line.strip()
            if line:
                paths.append(line)
        return paths

    def check_submodule_isolation(self) -> bool:
        '''check that each submodule update is isolated in its own commit'''
        submodule_paths = self.get_submodule_paths()

        commits_raw = self.run_git(
            ['rev-list', '--reverse', f'{self.base_branch}..HEAD'],
            show_output=False,
        ).strip()
        commits = [c.strip() for c in commits_raw.splitlines() if c.strip()]

        ok = True
        for commit in commits:
            changed = self.get_changed_paths_for_commit(commit)
            submodule_changes = [p for p in changed if p in submodule_paths]

            if not submodule_changes:
                continue

            other_changes = [p for p in changed if p not in submodule_paths]
            commit_ok = True

            if len(submodule_changes) > 1:
                print(
                    f"{FAIL} {commit[:12]} updates multiple submodules in one commit: "
                    f"{submodule_changes}"
                )
                commit_ok = False
            if other_changes:
                print(
                    f"{FAIL} {commit[:12]} updates submodule(s) {submodule_changes} "
                    f"but also modifies: {other_changes}"
                )
                commit_ok = False

            if commit_ok:
                print(f"{PASS} {commit[:12]} is a clean submodule update of {submodule_changes[0]}")
            else:
                ok = False

        if ok:
            print(f"{PASS} All submodule updates are isolated in their own commits.")
        return ok

    def _base_branch_name(self) -> str | None:
        '''extract bare branch name from self.base_branch; returns None for bare SHAs'''
        if not self.base_branch:
            return None
        if re.match(r'^[0-9a-f]{40}$', self.base_branch):
            return None
        # "upstream/master" -> "master", "origin/stable-4.5" -> "stable-4.5"
        return self.base_branch.split('/')[-1]

    def _github_repo_from_url(self, url: str) -> str | None:
        '''extract "owner/repo" from a GitHub HTTPS URL, or None if not a GitHub URL'''
        m = re.match(r'https://github\.com/([^/]+/[^/]+?)(?:\.git)?$', url.strip())
        return m.group(1) if m else None

    def _github_api_compare(self, owner_repo: str, sha: str) -> tuple:
        '''
        Call the GitHub compare API to check whether sha is an ancestor of master
        in the given canonical repo.

        GET /repos/{owner_repo}/compare/master...{sha}

        The REST API accesses the repository's own git database and is not subject
        to GitHub's fork-network object sharing (a git-protocol-level feature), so
        a commit that exists only in a personal fork will return HTTP 404/422 here.

        Returns (ok, message):
          True,  ""      — sha is reachable from master ("behind" or "identical")
          False, reason  — sha not in master's history or not in the canonical repo
          None,  reason  — unexpected API error; caller should treat as SKIP
        '''
        api_url = f'https://api.github.com/repos/{owner_repo}/compare/master...{sha}'
        headers = {'Accept': 'application/vnd.github.v3+json'}
        token = os.environ.get('GITHUB_TOKEN', '')
        if token:
            headers['Authorization'] = f'Bearer {token}'
        req = urllib.request.Request(api_url, headers=headers)
        try:
            with urllib.request.urlopen(req, timeout=30) as resp:
                data = json.loads(resp.read())
        except urllib.error.HTTPError as e:
            if e.code in (404, 422):
                return False, f"not found in canonical repo {owner_repo}"
            return None, f"GitHub API returned HTTP {e.code} for {owner_repo}"
        except urllib.error.URLError as e:
            return None, f"GitHub API unreachable: {e.reason}"
        status = data.get('status', '')
        if status in ('behind', 'identical'):
            return True, ""
        if status == 'ahead':
            return False, (f"exists in {owner_repo} but is not yet merged to master "
                           f"(it is ahead of master)")
        if status == 'diverged':
            return False, f"not reachable from master in {owner_repo} (diverged)"
        return None, f"unexpected GitHub compare status {status!r}"

    def check_submodule_references_exist(self) -> bool:
        '''
        For PRs targeting master, verify that each submodule reference introduced
        on this branch is reachable from master in the canonical submodule repo.

        Uses the GitHub compare API (GET /repos/{owner}/compare/master...{sha})
        which is repo-specific and is not affected by GitHub's fork-network object
        sharing (a git-protocol-level behaviour that allows any SHA present in any
        public fork to be fetched via the canonical URL).

        Skips silently for non-master PRs.
        '''
        if self._base_branch_name() != 'master':
            print(f"{SKIP} Submodule reference check only applies to master-targeting PRs.")
            return True

        submodule_paths = self.get_submodule_paths()
        submodule_urls = self.get_submodule_urls()

        commits_raw = self.run_git(
            ['rev-list', '--reverse', f'{self.base_branch}..HEAD'],
            show_output=False,
        ).strip()
        commits = [c.strip() for c in commits_raw.splitlines() if c.strip()]

        ok = True
        any_checked = False
        for commit in commits:
            changed = self.get_changed_paths_for_commit(commit)
            submodule_changes = [p for p in changed if p in submodule_paths]

            for submodule_path in submodule_changes:
                ls_tree_out = self.run_git(
                    ['ls-tree', commit, submodule_path],
                    show_output=False,
                ).strip()
                # format: "160000 commit <sha> <path>"
                parts = ls_tree_out.split()
                if len(parts) < 3 or parts[1] != 'commit':
                    continue
                sha = parts[2]

                url = submodule_urls.get(submodule_path)
                owner_repo = self._github_repo_from_url(url) if url else None
                label = f"{commit[:12]} references {submodule_path}@{sha[:12]}"

                if not owner_repo:
                    print(f"{SKIP} {label}: not a GitHub HTTPS URL, skipping")
                    continue

                any_checked = True
                ok_sha, msg = self._github_api_compare(owner_repo, sha)
                if ok_sha is None:
                    print(f"{SKIP} {label}: {msg}")
                elif ok_sha:
                    print(f"{PASS} {label}: reachable from master in {owner_repo}")
                else:
                    print(f"{FAIL} {label}: {msg}")
                    ok = False

        if ok:
            if any_checked:
                print(f"{PASS} All submodule references are reachable from master "
                      f"in their canonical repos.")
            else:
                print(f"{PASS} No submodule reference changes to check.")
        return ok

    def check_markdown_rst_hyperlinks(self) -> bool:
        '''flag RST-style markup in changed markdown files:
           - external hyperlinks: `text <url>`_ or `__
           - cross-references:    :ref:`...`, :doc:`...`, etc.
        '''
        changed_md = self.run_git(
            ["diff", "--name-only", "--diff-filter=AM",
             f"{self.base_branch}...HEAD", "--", "*.md"],
            show_output=False,
        ).strip()
        if not changed_md:
            print(f"{PASS} No markdown files changed (RST markup check).")
            return True

        files = changed_md.splitlines()
        ok = True

        # RST external hyperlinks: `text <url>`_ or `__
        result = subprocess.run(
            ["git", "grep", "-n", r"`[^`]*<[^>]*>`__\?", "--"] + files,
            capture_output=True, text=True,
        )
        if result.returncode > 1:
            print(f"{SKIP} git grep failed: {result.stderr.strip()}")
            return True
        if result.stdout.strip():
            print(f"{FAIL} RST-style hyperlinks found in markdown file(s):")
            for line in result.stdout.splitlines():
                print(f"         {line}")
            print("       Use markdown link syntax [text](url) instead.")
            ok = False

        # RST cross-references: :ref:, :doc:, :class:, etc.
        result = subprocess.run(
            ["git", "grep", "-n", r":[a-z]\+:`", "--"] + files,
            capture_output=True, text=True,
        )
        if result.returncode > 1:
            print(f"{SKIP} git grep failed: {result.stderr.strip()}")
            return True
        if result.stdout.strip():
            print(f"{FAIL} RST cross-references found in markdown file(s):")
            for line in result.stdout.splitlines():
                print(f"         {line}")
            print("       Use markdown link syntax [text](url) instead.")
            ok = False

        if ok:
            print(f"{PASS} No RST-style markup in changed markdown files.")
        return ok

    def check_markdown_rst_underlines(self) -> bool:
        '''flag setext/RST underline-style headings in changed markdown files'''
        changed_md = self.run_git(
            ["diff", "--name-only", "--diff-filter=AM",
             f"{self.base_branch}...HEAD", "--", "*.md"],
            show_output=False,
        ).strip()
        if not changed_md:
            print(f"{PASS} No markdown files changed (underline heading check).")
            return True

        underline_re = re.compile(r'^([=\-~^#+*])\1{2,}$')
        errors = []
        for path in changed_md.splitlines():
            path = path.strip()
            if not path or not os.path.exists(path):
                continue
            with open(path, encoding="utf-8", errors="replace") as fh:
                prev_line = ""
                for lineno, raw in enumerate(fh, 1):
                    line = raw.rstrip("\n")
                    if (underline_re.match(line)
                            and len(line.strip()) == len(prev_line.strip())):
                        errors.append((path, lineno, line.strip()))
                    prev_line = line

        if errors:
            print(f"{FAIL} Setext/RST underline-style headings found in markdown file(s):")
            for path, lineno, text in errors:
                print(f"         {path}:{lineno}: {text!r}")
            print("       Use ATX-style headings (# prefix) instead.")
            return False

        print(f"{PASS} No setext/RST underline headings in changed markdown files.")
        return True

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
            self.check_author_emails(),
            self.check_submodule_isolation(),
            self.check_submodule_references_exist(),
            self.check_markdown(),
            self.check_markdown_rst_hyperlinks(),
            self.check_markdown_rst_underlines(),
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
