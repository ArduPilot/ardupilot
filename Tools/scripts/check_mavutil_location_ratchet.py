#!/usr/bin/env python3

'''
Ratchet check for mavutil.location use in the autotest suite.

mavutil.location carries no altitude frame, and stuffing non-AMSL
altitudes into its alt attribute has been a recurring source of bugs.
It is being replaced by vehicle_test_suite.Location, which tags its
altitude with an AltFrame.

This script counts uses of mavutil.location (and of the deprecated
helpers which produce one) per file, both in the working tree and at
the merge-base with --base-ref, and fails if any file's count has
increased.  There is deliberately no stored budget: the invariant is
simply that a branch must not add uses, whatever the current number
happens to be.

AP_FLAKE8_CLEAN
'''

import argparse
import os
import re
import subprocess
import sys

# each counted pattern is paired with the frame-aware replacement to
# suggest when new uses appear
PATTERNS = [
    (re.compile(r"mavutil\.location\("),
     "vehicle_test_suite.Location with an explicit AltFrame, "
     "or Location.latlon_only() where the altitude is unused"),
    (re.compile(r"\.mav\.location\("),
     "self.get_location(); frame=AltFrame.ABOVE_HOME replaces relative_alt=True"),
    (re.compile(r"\.get_mav_location\("),
     "self.get_location()"),
    (re.compile(r"\.home_position_as_mav_location\("),
     "self.home_position_as_location()"),
    (re.compile(r"\.sim_location\("),
     "self.get_location('SIMSTATE')"),
    (re.compile(r"\.home_relative_loc_ne\("),
     "self.offset_location_ne(self.home_position_as_location(), n, e)"),
    (re.compile(r"\.home_relative_loc_neu\("),
     "self.offset_location_ne(self.home_position_as_location(), n, e) "
     "then set_alt_m(u, AltFrame.ABOVE_HOME)"),
    (re.compile(r"\.position_target_loc\("),
     "a Location built from POSITION_TARGET_GLOBAL_INT carrying the frame "
     "the target was commanded in"),
]

AUTOTEST_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "autotest",
)


def run_git(args):
    return subprocess.run(
        ["git"] + args,
        cwd=AUTOTEST_DIR,
        capture_output=True,
        text=True,
    )


def count_uses(content):
    count = 0
    for line in content.splitlines():
        for pattern, _ in PATTERNS:
            count += len(pattern.findall(line))
    return count


def tree_contents():
    '''return {filename: content} for the working tree'''
    ret = {}
    for filename in os.listdir(AUTOTEST_DIR):
        if not filename.endswith(".py"):
            continue
        with open(os.path.join(AUTOTEST_DIR, filename)) as f:
            ret[filename] = f.read()
    return ret


def base_contents(base):
    '''return {filename: content} for Tools/autotest at base'''
    ls = run_git(["ls-tree", "--name-only", base, "./"])
    if ls.returncode != 0:
        raise SystemExit(f"Failed to list {base}: {ls.stderr.strip()}")
    ret = {}
    for path in ls.stdout.splitlines():
        filename = os.path.basename(path)
        if not filename.endswith(".py"):
            continue
        show = run_git(["show", f"{base}:./{filename}"])
        if show.returncode != 0:
            continue
        ret[filename] = show.stdout
    return ret


GUIDANCE = ("mavutil.location is frameless and being phased out of autotest; "
            "use vehicle_test_suite.Location instead "
            "(for the current position: self.get_location())")


def emit_annotations(filename, old_content, new_content):
    '''emit a GitHub Actions error annotation for each line adding a
    counted use, suggesting the matched helper's specific replacement,
    so the guidance lands inline on the PR diff'''
    if os.environ.get("GITHUB_ACTIONS") != "true":
        return
    old_matching = {}
    for line in old_content.splitlines():
        if count_uses(line):
            old_matching[line] = old_matching.get(line, 0) + 1
    seen = {}
    for lineno, line in enumerate(new_content.splitlines(), 1):
        suggestions = [s for pattern, s in PATTERNS if pattern.search(line)]
        if not suggestions:
            continue
        seen[line] = seen.get(line, 0) + 1
        if seen[line] > old_matching.get(line, 0):
            print(f"::error file=Tools/autotest/{filename},line={lineno},"
                  "title=new frameless location use::"
                  f"replace with {'; '.join(suggestions)} "
                  "(mavutil.location and its producers are being phased "
                  "out of autotest: they carry no altitude frame)")


def resolve_base(base_ref):
    '''return the merge-base of HEAD and base_ref, so a base branch
    which has moved ahead does not affect the comparison'''
    candidates = [base_ref] if base_ref else ["upstream/master", "origin/master"]
    for candidate in candidates:
        mb = run_git(["merge-base", "HEAD", candidate])
        if mb.returncode == 0:
            return mb.stdout.strip(), candidate
    raise SystemExit(f"Could not resolve a base ref (tried {candidates}); pass --base-ref")


def check(base_ref):
    merge_base, ref_name = resolve_base(base_ref)
    print(f"Comparing against {ref_name} (merge-base {merge_base[:10]})")
    old = base_contents(merge_base)
    new = tree_contents()
    failed = False
    for filename in sorted(set(old) | set(new)):
        old_count = count_uses(old.get(filename, ""))
        new_count = count_uses(new.get(filename, ""))
        if new_count > old_count:
            print(f"FAIL {filename}: mavutil.location uses increased {old_count} -> {new_count}; "
                  f"{GUIDANCE}")
            emit_annotations(filename, old.get(filename, ""), new.get(filename, ""))
            failed = True
        elif new_count < old_count:
            print(f"GOOD {filename}: mavutil.location uses lowered {old_count} -> {new_count}")
    return not failed


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-ref", default=None,
                        help="ref to compare against (default: upstream/master, then origin/master)")
    args = parser.parse_args()
    if not check(args.base_ref):
        sys.exit(1)
    print("mavutil.location ratchet OK")
