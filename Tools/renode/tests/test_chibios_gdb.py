#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Tests for the ChibiOS GDB proxy's halt handling."""

import socket
import sys
import threading
import time

from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import chibios_gdb  # noqa: E402


def fake_renode(sock, received, replies):
    """Acknowledge each packet and answer it from the replies table."""
    connection = chibios_gdb.RSPConnection(sock)
    try:
        while True:
            kind, payload = connection.event()
            if kind != b"$":
                continue
            received.append(payload)
            connection.send_raw(b"+")
            for prefix, reply in replies:
                if payload.startswith(prefix):
                    if reply is not None:
                        connection.send_packet(reply)
                    break
    except (EOFError, OSError):
        pass


def make_proxy(replies):
    client, gdb = socket.socketpair()
    upstream, renode = socket.socketpair()
    received = []
    thread = threading.Thread(
        target=fake_renode, args=(renode, received, replies), daemon=True)
    thread.start()
    proxy = chibios_gdb.Proxy(
        chibios_gdb.RSPConnection(client),
        chibios_gdb.RSPConnection(upstream), 0,
        {"ch_debug": 0, "ch_system": 0})
    return proxy, gdb, received


def test_ok_is_not_console_output():
    assert not chibios_gdb.RSPConnection.is_console_output(b"OK")
    assert chibios_gdb.RSPConnection.is_console_output(b"O48690a")


def test_detach_resyncs_time_and_resumes():
    proxy, _gdb, received = make_proxy(((b"qRcmd,", b"OK"), (b"c", None)))
    proxy.halted_since = time.monotonic() - 1
    assert proxy.handle(b"D") == b"OK"
    assert received == [
        b"qRcmd," + chibios_gdb.RESYNC_COMMAND.hex().encode(), b"c"]
    assert proxy.halted_since is None

    # nothing left to release once the target runs
    proxy.release_target()
    assert len(received) == 2


def test_short_halt_skips_resync():
    proxy, _gdb, received = make_proxy(((b"c", None),))
    proxy.halted_since = time.monotonic()
    proxy.release_target()
    assert received == [b"c"]
