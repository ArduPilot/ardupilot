"""Add failed master binary builds to the autotest results.

AP_FLAKE8_CLEAN
"""

import os
import sqlite3


def add_build_failure_results(results, database_path, build_start_time):
    """Add failures from this invocation's latest-tag build to autotest results."""
    if not os.path.isfile(database_path):
        return

    connection = sqlite3.connect(database_path)
    try:
        run = connection.execute(
            "SELECT hash, start_time "
            "FROM run "
            "WHERE tag = ? AND start_time >= ? "
            "ORDER BY start_time DESC LIMIT 1",
            ("latest", build_start_time),
        ).fetchone()
        if run is None:
            return

        build_hash, run_start_time = run
        failures = connection.execute(
            "SELECT vehicle, board, frame, MAX(duration) "
            "FROM build "
            "WHERE hash = ? AND tag = ? AND start_time >= ? AND text IS NULL "
            "GROUP BY vehicle, board, frame "
            "ORDER BY vehicle, board, frame",
            (build_hash, "latest", run_start_time),
        ).fetchall()
    except sqlite3.OperationalError as error:
        if "no such table" in str(error):
            return
        raise
    finally:
        connection.close()

    for vehicle, board, frame, duration in failures:
        frame_suffix = f" {frame}" if frame else ""
        name = f"Build failure: {vehicle} {board}{frame_suffix}"
        results.add(name, '<span class="failed-text">FAILED</span>', duration)
