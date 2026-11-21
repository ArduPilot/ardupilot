"""Tests for Tools/autotest/process_scan_build_output.py."""

import os
import plistlib

import pytest

from Tools.autotest.process_scan_build_output import ProcessScanBuildOutput


def write_plist(report_dir, file_rel, issue_hash):
    """One scan-build report carrying a single finding."""
    data = {
        "clang_version": "Ubuntu clang version 14.0.0",
        "files": [file_rel],
        "diagnostics": [{
            "location": {"file": 0},
            "issue_hash_content_of_line_in_context": issue_hash,
            "type": "Dead assignment",
        }],
    }
    with open(report_dir / "report-1.plist", "wb") as f:
        plistlib.dump(data, f)


@pytest.fixture
def checkout(tmp_path, monkeypatch):
    """A checkout whose tmp directory exists, as it must for scan-build to run.

    autotest.py points TMPDIR at <checkout>/tmp, so any checkout that has run
    scan-build locally has one.  The name avoids "ardupilot" so that a path
    derived from the canonical repository name cannot pass by accident.
    """
    root = tmp_path / "renamed-fork"
    (root / "tmp").mkdir(parents=True)
    monkeypatch.delenv("GITHUB_ACTIONS", raising=False)
    monkeypatch.delenv("GITHUB_WORKSPACE", raising=False)
    return root


def scan_build_run(checkout, n):
    """Simulate one scan-build run: its report directory and captured stdout."""
    report = checkout / "tmp" / f"scan-build-2026-09-07-{n}"
    report.mkdir()
    write_plist(report, "libraries/AP_X/AP_X.cpp", f"hash{n}")
    stdout = checkout / f"stdout-{n}.txt"
    stdout.write_text(f"scan-build: Run 'scan-view {report}' to examine bug reports.\n")
    return ProcessScanBuildOutput(str(stdout)), str(report)


def in_github_actions(monkeypatch, workspace):
    monkeypatch.setenv("GITHUB_ACTIONS", "true")
    monkeypatch.setenv("GITHUB_WORKSPACE", str(workspace))


def test_local_run_leaves_reports_where_scan_build_put_them(checkout):
    p, report = scan_build_run(checkout, 1)
    assert p.archive_rename(report) == report
    assert os.path.isdir(report)
    assert not (checkout / "tmp" / "scan-build").exists()


def test_second_local_run_reads_its_own_reports(checkout):
    """Moving a run onto an existing tmp/scan-build would nest its reports
    beneath the previous run's, where the non-recursive plist glob does not
    look, and the previous run's findings would be reported instead."""
    p1, r1 = scan_build_run(checkout, 1)
    p1.archive_rename(r1)
    p2, r2 = scan_build_run(checkout, 2)
    findings = p2.findings_from_plists(p2.archive_rename(r2))
    assert {issue_hash for (_, issue_hash) in findings} == {"hash2"}


def test_github_actions_archives_to_the_workspace(checkout, monkeypatch):
    in_github_actions(monkeypatch, checkout)
    p, report = scan_build_run(checkout, 1)
    dest = checkout / "tmp" / "scan-build"
    assert p.archive_rename(report) == str(dest)
    assert (dest / "report-1.plist").is_file()
    assert p.stdout_filepath == str(dest / "stdout-1.txt")
    assert not os.path.exists(report)


def test_github_actions_refuses_an_existing_archive(checkout, monkeypatch):
    in_github_actions(monkeypatch, checkout)
    p1, r1 = scan_build_run(checkout, 1)
    p1.archive_rename(r1)
    p2, r2 = scan_build_run(checkout, 2)
    with pytest.raises(SystemExit):
        p2.archive_rename(r2)
    assert os.path.isdir(r2)


@pytest.mark.parametrize("workspace", [None, "", "relative/workspace"])
def test_github_actions_requires_an_absolute_workspace(checkout, monkeypatch, workspace):
    """An empty or relative workspace would archive relative to the current
    directory, where the upload step would only warn that nothing was found."""
    monkeypatch.setenv("GITHUB_ACTIONS", "true")
    if workspace is None:
        monkeypatch.delenv("GITHUB_WORKSPACE", raising=False)
    else:
        monkeypatch.setenv("GITHUB_WORKSPACE", workspace)
    elsewhere = checkout.parent / "elsewhere"
    elsewhere.mkdir()
    monkeypatch.chdir(elsewhere)
    p, report = scan_build_run(checkout, 1)
    with pytest.raises(SystemExit):
        p.archive_rename(report)
    assert os.path.isdir(report)
    assert os.listdir(elsewhere) == []
