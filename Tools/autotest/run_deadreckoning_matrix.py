#!/usr/bin/env python3

import argparse
import csv
import math
import os
import re
import shutil
import subprocess
import tempfile
from pathlib import Path

from tqdm import tqdm


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Run DeadreckoningNoAirSpeed stats matrix across branch states.\n\n"
            "Preconditions:\n"
            "- Must be run from the root dir of an ArduPilot checkout.\n"
            "- Environment must be set up to run Autotest Framework.\n"
            "- Local branches must exist: master, feature/adaptive-sideslip-fusion-variance, feature/navl1-lat-acc-feedback.\n"
        ),
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument(
        "--report",
        default="Tools/autotest/deadreckoning_stats_summary.txt",
        help="Output report file",
    )
    parser.add_argument("--runs", type=int, default=30)
    parser.add_argument("--speedup", type=int)
    parser.add_argument("--keep-temp", action="store_true")
    args = parser.parse_args()

    repo = Path(__file__).resolve().parents[2]
    report_path = (repo / args.report).resolve()
    report_path.parent.mkdir(parents=True, exist_ok=True)

    original_branch = subprocess.run(
        ["git", "rev-parse", "--abbrev-ref", "HEAD"],
        cwd=repo,
        check=True,
        text=True,
        capture_output=True,
    ).stdout.strip()

    adaptive_branch = "feature/adaptive-sideslip-fusion-variance"
    navl1_branch = "feature/navl1-lat-acc-feedback"
    for branch in (adaptive_branch, navl1_branch):
        exists = subprocess.run(
            ["git", "show-ref", "--verify", f"refs/heads/{branch}"],
            cwd=repo,
            text=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        if exists.returncode != 0:
            raise SystemExit(f"Missing local branch: {branch}")

    adaptive_over_master = subprocess.run(
        ["git", "rev-list", "--reverse", f"master..{adaptive_branch}"],
        cwd=repo,
        check=True,
        text=True,
        capture_output=True,
    ).stdout.splitlines()

    tmp_dir = Path(tempfile.mkdtemp(prefix="deadreckoning-matrix-"))
    tmp_branch = None
    try:
        blocks = []

        for label, branch in (
            ("MASTER", "master"),
            ("ADAPTIVE_SIDESLIP_FUSION_VARIANCE", adaptive_branch),
            ("NAVL1_LAT_ACC_FEEDBACK", navl1_branch),
        ):
            _checkout_and_surf(repo, branch)
            blocks.append(_run_suite(repo, tmp_dir, label, args.runs, args.speedup))

        tmp_branch = f"tmp/deadreckoning_navl1_plus_adaptive_delta_{os.getpid()}"
        _checkout_and_surf(repo, tmp_branch, create_from=navl1_branch)

        if adaptive_over_master:
            subprocess.run(
                ["git", "cherry-pick", *adaptive_over_master],
                cwd=repo,
                check=True,
                text=True,
            )

        blocks.append(
            _run_suite(
                repo,
                tmp_dir,
                "NAVL1_PLUS_ADAPTIVE_DELTA",
                args.runs,
                args.speedup,
            )
        )

        report_text = "DeadreckoningNoAirSpeed Stats Summary\n\n" + "\n\n".join(blocks)
        report_path.write_text(report_text + "\n", encoding="utf-8")
        print(f"\nCompleted. Report written to {report_path}")

    finally:
        subprocess.run(
            ["git", "cherry-pick", "--abort"],
            cwd=repo,
            text=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        restored = subprocess.run(
            ["git", "checkout", original_branch],
            cwd=repo,
            text=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        if restored.returncode == 0:
            _sync_submodules(repo)

        if tmp_branch:
            subprocess.run(
                ["git", "branch", "-D", tmp_branch],
                cwd=repo,
                text=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

        if args.keep_temp:
            print(f"Kept temp logs at: {tmp_dir}")
        else:
            shutil.rmtree(tmp_dir, ignore_errors=True)


def _checkout_and_surf(repo, branch, create_from=None):
    if create_from is None:
        subprocess.run(["git", "checkout", branch], cwd=repo, check=True, text=True)
    else:
        subprocess.run(
            ["git", "checkout", "-B", branch, create_from],
            cwd=repo,
            check=True,
            text=True,
        )

    _sync_submodules(repo)


def _sync_submodules(repo):
    surf = subprocess.run(
        ["git", "surf"],
        cwd=repo,
        check=False,
        text=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    if surf.returncode == 0:
        return

    subprocess.run(
        ["git", "submodule", "update", "--init", "--recursive"],
        cwd=repo,
        check=True,
        text=True,
    )


def _run_suite(repo, tmp_dir, label, runs_count, speedup):
    state_dir = tmp_dir / label
    state_dir.mkdir(parents=True, exist_ok=True)

    build_log = state_dir / "build.log"
    print(f"\n[{label}] build.Plane")
    with build_log.open("w", encoding="utf-8") as build_file:
        subprocess.run(
            ["./Tools/autotest/autotest.py", "build.Plane"],
            cwd=repo,
            check=True,
            text=True,
            stdout=build_file,
            stderr=subprocess.STDOUT,
        )

    rows = []
    csv_path = state_dir / "results.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["run", "rc", "max_div", "reason"])

        for run_idx in tqdm(
            range(1, runs_count + 1),
            desc=label,
            unit="run",
            leave=True,
        ):
            run_log = state_dir / f"run_{run_idx}.log"
            with run_log.open("w", encoding="utf-8") as run_file:
                cmd = [
                    "./Tools/autotest/autotest.py",
                    "test.PlaneTests1c.DeadreckoningNoAirSpeed",
                ]
                if speedup is not None:
                    cmd.append(f"--speedup={speedup}")

                result = subprocess.run(
                    cmd,
                    cwd=repo,
                    check=False,
                    text=True,
                    stdout=run_file,
                    stderr=subprocess.STDOUT,
                )

            max_div, reason = _parse_run_log(run_log)
            writer.writerow(
                [
                    run_idx,
                    result.returncode,
                    "" if max_div is None else f"{max_div:.6f}",
                    reason,
                ]
            )
            rows.append(
                {
                    "run": run_idx,
                    "rc": result.returncode,
                    "max_div": max_div,
                    "reason": reason,
                }
            )

    return _format_block(label, rows)


def _parse_run_log(path):
    text = path.read_text(encoding="utf-8", errors="replace")

    max_div = None
    reason = ""

    run_max_matches = re.findall(
        r"Maximum divergence was ([0-9]+(?:\.[0-9]+)?)m(?:/s)?", text
    )
    if run_max_matches:
        max_div = float(run_max_matches[-1])

    if max_div is None:
        max_div_matches = re.findall(r"max_div=([0-9]+(?:\.[0-9]+)?)", text)
        if max_div_matches:
            max_div = float(max_div_matches[-1])

    diverged_matches = re.findall(
        r"diverged from simstate by ([0-9]+(?:\.[0-9]+)?)m", text
    )
    if diverged_matches:
        reason = f"diverged {diverged_matches[-1]}m"
        if max_div is None:
            max_div = float(diverged_matches[-1])
    else:
        simple_diverged_matches = re.findall(r"diverged ([0-9]+(?:\.[0-9]+)?)m", text)
        if simple_diverged_matches:
            reason = f"diverged {simple_diverged_matches[-1]}m"
            if max_div is None:
                max_div = float(simple_diverged_matches[-1])

    return max_div, reason


def _format_block(label, rows):
    run_count = len(rows)
    success_count = sum(1 for row in rows if row["rc"] == 0)
    fail_count = run_count - success_count

    values = [row["max_div"] for row in rows if row["max_div"] is not None]
    values.sort()

    if values:
        avg_max = sum(values) / len(values)
        max_of_max = values[-1]
    else:
        avg_max = math.nan
        max_of_max = math.nan

    def q(p):
        if not values:
            return math.nan
        pos = (len(values) - 1) * p
        lo = int(math.floor(pos))
        hi = int(math.ceil(pos))
        if lo == hi:
            return values[lo]
        frac = pos - lo
        return values[lo] + (values[hi] - values[lo]) * frac

    def fmt(v):
        return "nan" if math.isnan(v) else f"{v:.6f}"

    lines = [
        f"[{label}]",
        f"runs: {run_count}  success: {success_count}  fail: {fail_count}",
        f"avg_max: {fmt(avg_max)}  max_of_max: {fmt(max_of_max)}",
        f"p50: {fmt(q(0.50))}  p90: {fmt(q(0.90))}  p95: {fmt(q(0.95))}  p99: {fmt(q(0.99))}",
        "failed_runs:",
    ]

    for row in rows:
        if row["rc"] == 0:
            continue
        if row["max_div"] is None:
            lines.append(f"  - run {row['run']}: max_div=N/A")
        elif row["reason"]:
            lines.append(
                f"  - run {row['run']}: max_div={row['max_div']:.6f}  {row['reason']}"
            )
        else:
            lines.append(f"  - run {row['run']}: max_div={row['max_div']:.6f}")

    return "\n".join(lines)


if __name__ == "__main__":
    main()
