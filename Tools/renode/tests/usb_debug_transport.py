# AP_FLAKE8_CLEAN
"""Userspace USB/IP transport for opt-in firmware debugger integration tests."""

import importlib.util
import queue
import socket
import struct
import sys
import threading
import time
import types

from concurrent.futures import Future
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from usbip_attach import import_device  # noqa: E402


class USB:
    def __init__(self, port=23451):
        end = time.monotonic() + 60
        while True:
            try:
                self.sock, self.devid, _ = import_device(types.SimpleNamespace(host="127.0.0.1", port=port, busid="1-0"))
                break
            except OSError:
                if time.monotonic() > end:
                    raise
                time.sleep(0.2)
        self.sock.settimeout(None)
        self.seq = 0
        self.pending = {}
        self.lock = threading.Lock()
        self.queues = {2: queue.Queue(), 4: queue.Queue()}
        self.running = True
        threading.Thread(target=self.reader, daemon=True).start()
        self.ctrl(0, 5, 1, 0, 0)
        print("device descriptor", self.ctrl(0x80, 6, 0x100, 0, 18).hex(), flush=True)
        cfg = self.ctrl(0x80, 6, 0x200, 0, 9)
        self.ctrl(0x80, 6, 0x200, 0, int.from_bytes(cfg[2:4], "little"))
        self.ctrl(0, 9, 1, 0, 0)
        for interface in (0, 2):
            self.ctrl(0x21, 0x22, 3, interface, 0)
        for ep in (2, 4):
            threading.Thread(target=self.read_ep, args=(ep,), daemon=True).start()

    def exact(self, n):
        b = b""
        while len(b) < n:
            d = self.sock.recv(n - len(b))
            if not d:
                raise EOFError("USB disconnected")
            b += d
        return b

    def reader(self):
        try:
            while True:
                h = struct.unpack(">12I", self.exact(48))
                data = self.exact(h[6]) if h[3] == 1 and h[6] else b""
                with self.lock:
                    f = self.pending.pop(h[1], None)
                if f:
                    status = struct.unpack('>i', struct.pack('>I', h[5]))[0]
                    if status:
                        f.set_exception(OSError(-status, "USB/IP transfer failed"))
                    else:
                        f.set_result(data)
        except (OSError, EOFError, struct.error) as e:
            self.running = False
            with self.lock:
                for f in self.pending.values():
                    f.set_exception(e)
                self.pending.clear()

    def submit(self, ep, data=None, length=64, setup=b"\0" * 8):
        with self.lock:
            self.seq += 1
            f = Future()
            self.pending[self.seq] = f
            direction = int(data is None)
            payload = struct.pack(
                ">10I8s", 1, self.seq, self.devid, direction, ep, 0, length if direction else len(data), 0, 0, 0, setup
            )
            self.sock.sendall(payload + (data or b""))
        return f

    def ctrl(self, typ, req, val, idx, n, data=b""):
        return self.submit(0, None if typ & 128 else data, n, struct.pack("<BBHHH", typ, req, val, idx, n)).result(10)

    def read_ep(self, ep):
        try:
            while self.running:
                data = self.submit(ep, length=512).result()
                if data:
                    self.queues[ep].put(data)
        except (OSError, EOFError):
            pass

    def write(self, ep, data):
        for i in range(0, len(data), 64):
            self.submit(ep, data[i : i + 64]).result(5)

    def close(self):
        self.running = False
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.sock.close()


class GDB:
    def __init__(self, usb):
        self.u = usb
        self.buf = b""

    def read(self, n):
        if not self.buf:
            try:
                self.buf = self.u.queues[4].get(timeout=0.1)
            except queue.Empty:
                return b""
        b, self.buf = self.buf[:n], self.buf[n:]
        return b

    def write(self, b):
        self.u.write(4, b)

    def packet(self, b):
        self.write(gdb.rsp_packet(b))
        return gdb.read_packet(self, time.monotonic() + 10)

    def stop(self):
        self.write(b"\x03")
        return gdb.read_packet(self, time.monotonic() + 10)


spec = importlib.util.spec_from_file_location("gdb", str(Path(__file__).resolve().parents[2] / "debug/gdb_usb.py"))
gdb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gdb)


class GCS:
    def __init__(self, u):
        from pymavlink import mavutil

        self.u = u
        self.m = mavutil.mavlink.MAVLink(self, srcSystem=255)
        self.m.robust_parsing = True
        self.messages = []

    def write(self, b):
        self.u.write(2, b)

    def recv(self, kind, timeout=10):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            for i, m in enumerate(self.messages):
                if m.get_type() == kind:
                    return self.messages.pop(i)
            try:
                data = self.u.queues[2].get(timeout=0.1)
            except queue.Empty:
                continue
            self.messages.extend(self.m.parse_buffer(data) or [])
        raise TimeoutError(kind)

    def param(self):
        self.m.param_request_read_send(1, 1, b"ARMING_SKIPCHK", -1)
        m = self.recv("PARAM_VALUE")
        print("parameter reply", m, flush=True)
        assert m.param_id == "ARMING_SKIPCHK"
