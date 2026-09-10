#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Process lifecycle helpers shared by Renode launchers and tests."""

import os
import signal
import subprocess
import sys
import time


def _linux_process_group_has_live_members(process_group):
    """Return whether a Linux process group has non-zombie members."""
    if not sys.platform.startswith('linux'):
        return None

    try:
        entries = os.scandir('/proc')
    except OSError:
        return None

    with entries:
        for entry in entries:
            if not entry.name.isdigit():
                continue
            try:
                with open(entry.path + '/stat', encoding='ascii') as stat_file:
                    stat = stat_file.read()
            except FileNotFoundError:
                continue
            except OSError:
                return None

            # The command name is enclosed in parentheses and may contain
            # spaces or parentheses. The fields after its final ')' begin
            # with state, parent PID and process-group ID.
            command_end = stat.rfind(')')
            if command_end < 0:
                continue
            fields = stat[command_end + 1:].split()
            if len(fields) < 3:
                continue
            try:
                member_group = int(fields[2])
            except ValueError:
                continue
            if member_group == process_group and fields[0] not in ('X', 'Z'):
                return True
    return False


def _process_group_exists(process_group):
    has_live_members = _linux_process_group_has_live_members(process_group)
    if has_live_members is not None:
        return has_live_members
    try:
        os.killpg(process_group, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _wait_process_group(process, process_group, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        # Reap the group leader when it has exited. Descendants can outlive it,
        # so group existence rather than process.poll() decides when cleanup is
        # complete.
        process.poll()
        if not _process_group_exists(process_group):
            return True
        time.sleep(0.05)
    process.poll()
    return not _process_group_exists(process_group)


def terminate_process_group(process, graceful_signal=signal.SIGTERM,
                            graceful_timeout=5):
    """Stop a start_new_session subprocess and every process it spawned."""
    process_group = process.pid
    stages = [(graceful_signal, graceful_timeout)]
    if graceful_signal != signal.SIGTERM:
        stages.append((signal.SIGTERM, 5))
    stages.append((signal.SIGKILL, 3))

    for stop_signal, timeout in stages:
        try:
            os.killpg(process_group, stop_signal)
        except ProcessLookupError:
            process.poll()
            return
        if _wait_process_group(process, process_group, timeout):
            return

    raise subprocess.TimeoutExpired(
        'process group %u' % process_group,
        sum(timeout for _stop_signal, timeout in stages),
    )
