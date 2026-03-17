#!/usr/bin/env python3

from __future__ import annotations

'''
Build N commits on a branch and track flash size and feature changes.

Produces:
  - A table of flash usage per commit
  - A feature change report showing when features were introduced/removed
  - Optionally, an HTML report with bar chart and feature details

Usage:
  python3 Tools/scripts/size_history.py --num-commits 5 --board MatekF405-Wing --vehicle plane
  python3 Tools/scripts/size_history.py --num-commits 10 --save-html-to report.html

AP_FLAKE8_CLEAN
'''

import argparse
import html
import os
import shutil
import subprocess

from build_script_base import VEHICLE_MAP
from build_script_base import BuildScriptBase


class SizeHistory(BuildScriptBase):

    def __init__(self, branch, num_commits, board, vehicle, jobs, save_html_to):
        super().__init__()

        self.branch = branch
        self.num_commits = num_commits
        self.board = board
        self.vehicle = vehicle
        self.jobs = jobs
        self.save_html_to = save_html_to

        if vehicle not in VEHICLE_MAP:
            raise ValueError(
                "Bad vehicle (%s); choose from %s" %
                (vehicle, ",".join(VEHICLE_MAP.keys())))
        self.binary_name = VEHICLE_MAP[vehicle]

    def progress_prefix(self):
        return 'SH'

    def get_commit_list(self):
        '''get list of (hash, subject) tuples for the last N commits, oldest first'''
        output = self.run_git(
            ["log", "--format=%H %s", "-%d" % self.num_commits, self.branch],
            show_output=False)
        commits = []
        for line in output.strip().splitlines():
            parts = line.split(" ", 1)
            commit_hash = parts[0]
            subject = parts[1] if len(parts) > 1 else ""
            commits.append((commit_hash, subject))
        commits.reverse()  # oldest first
        return commits

    def build_commit(self, commit_hash):
        '''checkout and build a single commit. Returns size dict or None on failure.'''
        try:
            self.run_git(["checkout", commit_hash], show_output=False)
            self.run_git(["submodule", "update", "--recursive"], show_output=False)
        except subprocess.CalledProcessError:
            self.progress("Failed to checkout %s" % commit_hash[:10])
            return None

        build_dir = "build"
        shutil.rmtree(build_dir, ignore_errors=True)

        waf_configure_args = [
            "configure",
            "--board", self.board,
            "--consistent-builds",
        ]
        if self.jobs is not None:
            waf_configure_args.extend(["-j", str(self.jobs)])

        try:
            self.run_waf(waf_configure_args, show_output=False)
            self.run_waf([self.vehicle], show_output=False)
        except subprocess.CalledProcessError:
            self.progress("Build failed for %s" % commit_hash[:10])
            return None

        elf_path = os.path.join("build", self.board, "bin", self.binary_name)
        if not os.path.exists(elf_path):
            self.progress("ELF not found at %s" % elf_path)
            return None

        try:
            sizes = self.size_for_elf(elf_path)
        except (subprocess.CalledProcessError, ValueError) as ex:
            self.progress("size_for_elf failed: %s" % str(ex))
            return None

        try:
            from extract_features import ExtractFeatures
            ef = ExtractFeatures(elf_path)
            (features_in, features_not_in) = ef.extract()
        except Exception as ex:
            self.progress("Feature extraction failed: %s" % str(ex))
            features_in = set()

        sizes['features_in'] = features_in
        return sizes

    def print_size_table(self, results):
        '''print the size table to stdout'''
        # header
        print("")
        print("%-12s %-50s %12s %12s %12s" % (
            "Commit", "Subject", "Flash Used", "Flash Free", "Delta Used"))
        print("-" * 100)

        prev_flash_free = None
        for r in results:
            short_hash = r['hash'][:10]
            subject = r['subject'][:50]
            if r['sizes'] is None:
                print("%-12s %-50s %12s %12s %12s" % (
                    short_hash, subject, "FAILED", "", ""))
                continue

            total = r['sizes']['size_total']
            flash_free = r['sizes'].get('size_free_flash')

            if flash_free is not None and prev_flash_free is not None:
                # invert: flash_used delta is negative of flash_free delta
                delta_used = -(flash_free - prev_flash_free)
                delta_str = "%+d" % delta_used
            else:
                delta_str = ""

            flash_free_str = str(flash_free) if flash_free is not None else "N/A"

            print("%-12s %-50s %12d %12s %12s" % (
                short_hash, subject, total, flash_free_str, delta_str))

            if flash_free is not None:
                prev_flash_free = flash_free

    def print_feature_report(self, results):
        '''print feature change report to stdout'''
        print("")
        print("Feature Changes:")
        print("-" * 80)

        prev_result = None
        for r in results:
            if r['sizes'] is None:
                continue

            if prev_result is not None:
                prev_features = set(prev_result['sizes'].get('features_in', []))
                curr_features = set(r['sizes'].get('features_in', []))

                added = sorted(curr_features - prev_features)
                removed = sorted(prev_features - curr_features)

                if added or removed:
                    flash_free = r['sizes'].get('size_free_flash')
                    prev_flash_free = prev_result['sizes'].get('size_free_flash')
                    if flash_free is not None and prev_flash_free is not None:
                        delta_used = -(flash_free - prev_flash_free)
                        delta_str = "%+d bytes used" % delta_used
                    else:
                        delta_str = "N/A"

                    flash_free_str = str(flash_free) if flash_free is not None else "N/A"

                    print("  %s %s  (delta: %s, flash_free: %s)" % (
                        r['hash'][:10], r['subject'][:50],
                        delta_str, flash_free_str))
                    for f in added:
                        print("    +%s" % f)
                    for f in removed:
                        print("    -%s" % f)

            prev_result = r

    def commit_url(self, commit_hash):
        '''return GitHub URL for a commit'''
        return "https://github.com/ardupilot/ardupilot/commit/%s" % commit_hash

    def produce_html(self, results):
        '''produce an HTML report with bar chart and feature details'''
        if self.save_html_to is None:
            return

        # compute per-commit deltas (in flash-used terms) and feature diffs
        rows = []
        prev_flash_free = None
        prev_features = None
        max_abs_delta = 1  # avoid division by zero

        for r in results:
            delta = None
            added = []
            removed = []

            if r['sizes'] is not None:
                flash_free = r['sizes'].get('size_free_flash')
                if flash_free is not None and prev_flash_free is not None:
                    # flash_used delta: positive means more flash consumed
                    delta = -(flash_free - prev_flash_free)

                curr_features = set(r['sizes'].get('features_in', []))
                if prev_features is not None:
                    added = sorted(curr_features - prev_features)
                    removed = sorted(prev_features - curr_features)
                prev_features = curr_features

                if flash_free is not None:
                    prev_flash_free = flash_free

            if delta is not None and abs(delta) > max_abs_delta:
                max_abs_delta = abs(delta)

            rows.append({
                'hash': r['hash'],
                'subject': r['subject'],
                'sizes': r['sizes'],
                'delta': delta,
                'added': added,
                'removed': removed,
            })

        # build HTML
        h = []
        h.append('<!DOCTYPE html>')
        h.append('<html><head>')
        h.append('<meta charset="utf-8">')
        h.append('<title>Flash Size History: %s on %s</title>' % (
            html.escape(self.vehicle), html.escape(self.board)))
        h.append('<style>')
        h.append('body { font-family: monospace; margin: 20px; background: #fff; }')
        h.append('h1 { font-size: 18px; }')
        h.append('table { border-collapse: collapse; width: 100%; }')
        h.append('th, td { border: 1px solid #ccc; padding: 6px 10px; text-align: right; }')
        h.append('th { background: #f0f0f0; }')
        h.append('td.subject { text-align: left; max-width: 400px; '
                 'overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }')
        h.append('td.hash { text-align: left; font-family: monospace; }')
        h.append('td.bar-cell { text-align: left; padding: 0; width: 300px; }')
        h.append('.bar-container { position: relative; height: 24px; width: 300px; }')
        h.append('.bar { position: absolute; top: 2px; height: 20px; }')
        # green (less flash used) grows left from center
        h.append('.bar-green { background: #2ecc40; right: 50%; }')
        # red (more flash used) grows right from center
        h.append('.bar-red { background: #e74c3c; left: 50%; }')
        h.append('.bar-zero-line { position: absolute; left: 50%; top: 0; '
                 'height: 24px; width: 1px; background: #333; }')
        h.append('.features { margin: 2px 0; font-size: 12px; }')
        h.append('.feat-added { color: #2ecc40; }')
        h.append('.feat-removed { color: #e74c3c; }')
        h.append('a { color: #0366d6; text-decoration: none; }')
        h.append('a:hover { text-decoration: underline; }')
        h.append('tr.failed { background: #fdd; }')
        h.append('td.failed-msg { color: #c00; font-style: italic; text-align: center; }')
        h.append('</style>')
        h.append('</head><body>')
        h.append('<h1>Flash Size History: %s on %s</h1>' % (
            html.escape(self.vehicle), html.escape(self.board)))

        h.append('<table>')
        h.append('<tr><th>Commit</th><th>Subject</th>'
                 '<th>Flash Used</th><th>Flash Free</th>'
                 '<th>Delta Used</th><th>Delta Used</th>'
                 '<th>Features Changed</th></tr>')

        for row in rows:
            short = row['hash'][:10]
            url = self.commit_url(row['hash'])
            subj = html.escape(row['subject'][:60])

            if row['sizes'] is None:
                h.append('<tr class="failed">')
                h.append('<td class="hash"><a href="%s">%s</a></td>' % (url, short))
                h.append('<td class="subject" title="%s">%s</td>' % (
                    html.escape(row['subject']), subj))
                h.append('<td class="failed-msg" colspan="5">build failed</td>')
                h.append('</tr>')
                continue

            total = row['sizes']['size_total']
            flash_free = row['sizes'].get('size_free_flash')
            flash_free_str = str(flash_free) if flash_free is not None else "N/A"
            delta = row['delta']

            h.append('<tr>')
            h.append('<td class="hash"><a href="%s">%s</a></td>' % (url, short))
            h.append('<td class="subject" title="%s">%s</td>' % (
                html.escape(row['subject']), subj))

            h.append('<td>%d</td>' % total)
            h.append('<td>%s</td>' % flash_free_str)

            # delta text: positive = more flash used (red), negative = less used (green)
            if delta is not None:
                color = '#e74c3c' if delta > 0 else '#2ecc40' if delta < 0 else '#999'
                h.append('<td style="color: %s">%+d</td>' % (color, delta))
            else:
                h.append('<td></td>')

            # delta bar: red grows right from center, green grows left
            h.append('<td class="bar-cell">')
            h.append('<div class="bar-container">')
            h.append('<div class="bar-zero-line"></div>')
            if delta is not None and delta != 0:
                bar_pct = abs(delta) / max_abs_delta * 48
                bar_pct = max(bar_pct, 1)
                if delta > 0:
                    # more flash used = bad = red, grows right
                    h.append('<div class="bar bar-red" style="width: %.1f%%"></div>' % bar_pct)
                else:
                    # less flash used = good = green, grows left
                    h.append('<div class="bar bar-green" style="width: %.1f%%"></div>' % bar_pct)
            h.append('</div>')
            h.append('</td>')

            # features
            h.append('<td style="text-align: left;">')
            if row['added'] or row['removed']:
                h.append('<div class="features">')
                for f in row['added']:
                    h.append('<span class="feat-added">+%s</span><br>' % html.escape(f))
                for f in row['removed']:
                    h.append('<span class="feat-removed">-%s</span><br>' % html.escape(f))
                h.append('</div>')
            h.append('</td>')

            h.append('</tr>')

        h.append('</table>')
        h.append('</body></html>')

        with open(self.save_html_to, 'w') as f:
            f.write('\n'.join(h))
        self.progress("HTML report saved to %s" % self.save_html_to)

    def run(self):
        # save current HEAD
        saved_head = self.run_git(
            ["rev-parse", "HEAD"], show_output=False).strip()
        saved_branch = None
        try:
            saved_branch = self.run_git(
                ["symbolic-ref", "--short", "HEAD"],
                show_output=False).strip()
        except subprocess.CalledProcessError:
            pass  # detached HEAD

        commits = self.get_commit_list()
        self.progress("Will build %d commits" % len(commits))

        results = []
        for i, (commit_hash, subject) in enumerate(commits):
            self.progress("Building commit %d/%d: %s %s" % (
                i + 1, len(commits), commit_hash[:10], subject[:40]))
            sizes = self.build_commit(commit_hash)
            results.append({
                'hash': commit_hash,
                'subject': subject,
                'sizes': sizes,
            })

        # restore original HEAD
        try:
            if saved_branch is not None:
                self.run_git(["checkout", saved_branch], show_output=False)
            else:
                self.run_git(["checkout", saved_head], show_output=False)
        except subprocess.CalledProcessError:
            self.progress("WARNING: failed to restore original HEAD (%s)" %
                          (saved_branch or saved_head))

        self.print_size_table(results)
        self.print_feature_report(results)
        self.produce_html(results)

        return results


def main():
    parser = argparse.ArgumentParser(
        description='Track flash size and feature changes across commits')
    parser.add_argument('--branch', default='master',
                        help='branch to walk (default: master)')
    parser.add_argument('--num-commits', type=int, required=True,
                        help='number of commits to compile')
    parser.add_argument('--board', default='MatekF405-Wing',
                        help='board to build for (default: MatekF405-Wing)')
    parser.add_argument('--vehicle', default='plane',
                        help='vehicle to build (default: plane)')
    parser.add_argument('-j', '--jobs', type=int, default=None,
                        help='number of build jobs passed to waf')
    parser.add_argument('--save-html-to', default=None,
                        help='save HTML report to this file path')
    args = parser.parse_args()

    x = SizeHistory(
        branch=args.branch,
        num_commits=args.num_commits,
        board=args.board,
        vehicle=args.vehicle,
        jobs=args.jobs,
        save_html_to=args.save_html_to,
    )
    x.run()


if __name__ == '__main__':
    main()
