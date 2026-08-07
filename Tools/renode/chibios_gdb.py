#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Expose ChibiOS threads through Renode's GDB remote server.

Renode exposes emulated CPU cores as GDB threads, but it does not inspect an
RTOS running on a core.  This proxy uses ChibiOS's ``ch_debug`` target-memory
signature to enumerate the thread registry and provide the saved Cortex-M
register context for threads which are not currently executing.

All packets unrelated to guest threads are passed through unchanged.  In
particular, Renode's reverse execution packets continue to reach Renode.
"""

import argparse
import html
import select
import shutil
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

THREAD_STATES = (
    "READY", "CURRENT", "WTSTART", "SUSPENDED", "QUEUED", "WTSEM",
    "WTMTX", "WTCOND", "SLEEPING", "WTEXIT", "WTOREVT", "WTANDEVT",
    "SNDMSGQ", "SNDMSG", "WTMSG", "FINAL",
)


def elf_symbols(elf):
    """Return the addresses needed by the ChibiOS thread registry."""
    nm = shutil.which("arm-none-eabi-nm") or shutil.which("nm")
    if nm is None:
        sys.exit("nm is required for ChibiOS-aware GDB")
    try:
        output = subprocess.check_output(
            [nm, str(elf)], stderr=subprocess.STDOUT, text=True
        )
    except (OSError, subprocess.CalledProcessError) as error:
        sys.exit("failed to read symbols from %s: %s" % (elf, error))
    wanted = {"ch_debug", "ch_system"}
    symbols = {}
    for line in output.splitlines():
        fields = line.split()
        if len(fields) == 3 and fields[2] in wanted:
            symbols[fields[2]] = int(fields[0], 16)
    missing = wanted - symbols.keys()
    if missing:
        sys.exit("%s has no %s symbol" % (elf, " or ".join(sorted(missing))))
    return symbols


def packet(payload):
    checksum = sum(payload) & 0xFF
    return b"$" + payload + b"#%02x" % checksum


class RSPConnection:
    """A small, acknowledgement-mode GDB remote protocol connection."""

    def __init__(self, sock):
        self.sock = sock
        self.buffer = bytearray()

    def send_raw(self, data):
        self.sock.sendall(data)

    def send_packet(self, payload):
        self.send_raw(packet(payload))

    def event(self):
        while True:
            if self.buffer:
                first = self.buffer[0]
                if first in (ord("+"), ord("-"), 3):
                    del self.buffer[0]
                    return bytes((first,)), None
                if first == ord("$"):
                    marker = self.buffer.find(b"#", 1)
                    if marker >= 0 and len(self.buffer) >= marker + 3:
                        payload = bytes(self.buffer[1:marker])
                        claimed = bytes(self.buffer[marker + 1:marker + 3])
                        del self.buffer[:marker + 3]
                        try:
                            valid = (sum(payload) & 0xFF) == int(claimed, 16)
                        except ValueError:
                            valid = False
                        if not valid:
                            self.send_raw(b"-")
                            continue
                        return b"$", payload
                elif first not in (ord("+"), ord("-"), 3):
                    # Ignore noise outside a packet.  Renode and GDB do not
                    # normally produce any, but this avoids a permanent stall.
                    del self.buffer[0]
                    continue
            data = self.sock.recv(4096)
            if not data:
                raise EOFError
            self.buffer.extend(data)

    def receive_packet(self):
        while True:
            kind, payload = self.event()
            if kind == b"$":
                self.send_raw(b"+")
                return payload

    def transact(self, payload):
        response = self.send_command(payload)
        if response is not None:
            return response
        while True:
            response = self.receive_packet()
            if self.expects_stop(payload) or not self.is_stop(response):
                return response

    @staticmethod
    def is_stop(payload):
        return payload.startswith((b"S", b"T", b"W", b"X"))

    @staticmethod
    def expects_stop(payload):
        return payload == b"?" or payload.startswith(
            (b"c", b"s", b"vCont", b"bc", b"bs")
        )

    def send_command(self, payload):
        early_response = None
        while True:
            self.send_packet(payload)
            while True:
                kind, response = self.event()
                if kind == b"-":
                    break
                if kind == b"$":
                    self.send_raw(b"+")
                    # Renode can send an asynchronous stop notification when
                    # a new connection halts a running target.  Do not mistake
                    # it for the response to the next query.
                    if self.is_stop(response) and not self.expects_stop(payload):
                        continue
                    early_response = response
                    continue
                if kind != b"+":
                    raise RuntimeError("GDB server did not acknowledge a packet")
                return early_response


class ChibiOS:
    """ChibiOS registry and saved-context reader."""

    def __init__(self, upstream, ch_debug, ch_system):
        self.upstream = upstream
        self.ch_debug = ch_debug
        self.ch_system = ch_system
        self.signature = None
        self.threads = {}
        self.current = 1
        self.names = {}

    def read(self, address, size):
        response = self.upstream.transact(
            b"m%x,%x" % (address, size)
        )
        if response.startswith(b"E") or len(response) != size * 2:
            raise RuntimeError("cannot read %u bytes at 0x%x" % (size, address))
        try:
            return bytes.fromhex(response.decode("ascii"))
        except ValueError as error:
            raise RuntimeError("invalid memory response at 0x%x" % address) from error

    def u32(self, address):
        return struct.unpack("<I", self.read(address, 4))[0]

    def load_signature(self):
        header = self.read(self.ch_debug, 8)
        size = header[5]
        if header[:5] != b"main\0" or size < 43:
            raise RuntimeError("invalid ChibiOS ch_debug signature")
        data = self.read(self.ch_debug, size)
        if data[8] != 4:
            raise RuntimeError("only 32-bit ChibiOS targets are supported")
        self.signature = {
            "thread_size": data[10],
            "ctx": data[12],
            "newer": data[13],
            "older": data[14],
            "name": data[15],
            "state": data[17],
            "intctx_size": data[26],
            "instances": data[28],
            "sys_instances": data[30],
            "sys_reglist": data[31],
            "inst_current": data[37],
            "inst_rlist": data[38],
            "inst_reglist": data[40],
        }

    def read_name(self, pointer):
        if pointer == 0:
            return "No Name"
        if pointer in self.names:
            return self.names[pointer]
        raw = self.read(pointer, 64).split(b"\0", 1)[0]
        name = raw.decode("utf-8", errors="replace") or "No Name"
        self.names[pointer] = name
        return name

    @staticmethod
    def data_u32(data, offset):
        return struct.unpack_from("<I", data, offset)[0]

    def update(self):
        """Refresh thread details while the emulation is halted."""
        self.threads = {}
        self.current = 1
        try:
            if self.signature is None:
                self.load_signature()
            sig = self.signature
            instances = []
            for index in range(sig["instances"]):
                instance = self.u32(self.ch_system + sig["sys_instances"] + index * 4)
                if instance:
                    instances.append(instance)
            if not instances:
                return

            if sig["sys_reglist"]:
                registry = self.ch_system + sig["sys_reglist"]
            else:
                registry = instances[0] + sig["inst_reglist"]

            current = self.u32(
                instances[0] + sig["inst_rlist"] + sig["inst_current"]
            )
            node = self.u32(registry)
            previous = registry
            visited = set()
            while node != registry:
                if node == 0 or node in visited or len(visited) >= 256:
                    raise RuntimeError("corrupt ChibiOS thread registry")
                visited.add(node)
                thread_id = node - sig["newer"]
                thread = self.read(thread_id, sig["thread_size"])
                if self.data_u32(thread, sig["older"]) != previous:
                    raise RuntimeError("broken ChibiOS registry back-link")
                name_pointer = self.data_u32(thread, sig["name"])
                state_number = thread[sig["state"]]
                state = (THREAD_STATES[state_number]
                         if state_number < len(THREAD_STATES) else "UNKNOWN")
                self.threads[thread_id] = {
                    "name": self.read_name(name_pointer),
                    "state": state,
                    "stack": self.data_u32(thread, sig["ctx"]),
                }
                previous = node
                node = self.data_u32(thread, sig["newer"])
            if current in self.threads:
                self.current = current
        except RuntimeError:
            # The registry legitimately does not exist during early startup.
            # Keep exposing Renode's physical CPU until ChibiOS is initialized.
            self.threads = {}
            self.current = 1

    def xml(self):
        if not self.threads:
            details = ((1, "CPU", "ChibiOS not initialized"),)
        else:
            details = (
                (thread_id, item["name"], "State: " + item["state"])
                for thread_id, item in self.threads.items()
            )
        lines = ['<?xml version="1.0"?>', "<threads>"]
        for thread_id, name, state in details:
            lines.append(
                '<thread id="%x" core="0" name="%s">%s</thread>' %
                (thread_id, html.escape(name, quote=True), html.escape(state))
            )
        lines.append("</threads>")
        return ("\n".join(lines) + "\n").encode("utf-8")

    def extra_info(self, thread_id):
        if thread_id == 1 and not self.threads:
            return "ChibiOS not initialized"
        item = self.threads.get(thread_id)
        return None if item is None else "State: " + item["state"]

    def saved_registers(self, thread_id, physical):
        """Overlay a stopped thread's saved Cortex-M context on a g reply."""
        sig = self.signature
        item = self.threads.get(thread_id)
        if item is None:
            raise RuntimeError("unknown ChibiOS thread")
        stack = item["stack"]
        context_size = sig["intctx_size"]
        if context_size < 36:
            raise RuntimeError("invalid ChibiOS Cortex-M context size")
        context = self.read(stack + context_size - 36, 36)

        # Renode's g packet contains r0-r15, all 32-bit little-endian.  Other
        # registers are sparse in its target description and fetched with p.
        # Mark core registers not saved by a context switch unavailable instead
        # of borrowing their values from the running thread.
        minimum = 16 * 8
        if len(physical) < minimum:
            raise RuntimeError("short Cortex-M register packet")
        registers = [physical[index:index + 8] for index in range(0, len(physical), 8)]
        unavailable = "xxxxxxxx"
        for index in (0, 1, 2, 3, 12, 14):
            registers[index] = unavailable
        for index in range(4, 12):
            offset = (index - 4) * 4
            registers[index] = context[offset:offset + 4].hex()
        registers[13] = struct.pack("<I", stack + context_size).hex()
        registers[15] = context[32:36].hex()
        return "".join(registers).encode("ascii")


class Proxy:
    def __init__(self, client, upstream, upstream_port, symbols):
        self.client = client
        self.upstream = upstream
        self.upstream_port = upstream_port
        self.chibios = ChibiOS(
            upstream, symbols["ch_debug"], symbols["ch_system"]
        )
        self.selected_thread = 1
        self.threads_fresh = False

    def refresh_threads(self):
        if not self.threads_fresh:
            self.chibios.update()
            self.threads_fresh = True

    def forward(self, payload):
        return self.upstream.transact(payload)

    def reconnect_upstream(self):
        """Connect to the GDB stub restored from a reverse snapshot."""
        # Renode reports the reverse stop just before the restored machine
        # finishes replacing its GDB listener. Wait for the old listener to
        # close; reconnecting to it exposes only its physical CPU thread and
        # confuses GDB's existing RTOS thread model.
        deadline = time.monotonic() + 5
        while True:
            timeout = deadline - time.monotonic()
            if timeout <= 0:
                break
            ready, _, _ = select.select((self.upstream.sock,), (), (), timeout)
            if not ready:
                break
            try:
                if not self.upstream.sock.recv(4096):
                    break
            except OSError:
                break
        self.upstream.sock.close()
        self.upstream = RSPConnection(connect_upstream(self.upstream_port))
        self.chibios.upstream = self.upstream
        self.threads_fresh = False

    def forward_execution(self, payload):
        """Wait for a stop while continuing to relay GDB interrupts."""
        response = self.upstream.send_command(payload)
        if response is not None:
            return response

        while True:
            if self.upstream.buffer:
                ready = (self.upstream.sock,)
            elif self.client.buffer:
                ready = (self.client.sock,)
            else:
                ready, _, _ = select.select(
                    (self.upstream.sock, self.client.sock), (), ()
                )

            if self.upstream.sock in ready:
                kind, response = self.upstream.event()
                if kind == b"$":
                    self.upstream.send_raw(b"+")
                    return response
                if kind == b"-":
                    raise RuntimeError("Renode rejected an execution packet")

            if self.client.sock in ready:
                kind, client_payload = self.client.event()
                if kind == b"\x03":
                    self.upstream.send_raw(kind)
                elif kind == b"$":
                    self.client.send_raw(b"+")
                    raise RuntimeError(
                        "unexpected GDB packet while target runs: %r" %
                        client_payload
                    )

    @staticmethod
    def physical_execution_packet(payload):
        """Map an RTOS thread-qualified run command to Renode's CPU 1."""
        if payload.startswith(b"vCont;"):
            actions = payload.split(b";")
            for index in range(1, len(actions)):
                action, separator, _ = actions[index].partition(b":")
                if separator:
                    actions[index] = action + b":1"
            return b";".join(actions)
        if payload.startswith((b"c:", b"s:", b"bc:", b"bs:")):
            return payload.split(b":", 1)[0] + b":1"
        return payload

    def thread_id(self, value):
        try:
            return int(value, 16)
        except ValueError:
            return None

    def handle(self, payload):
        text = payload.decode("ascii", errors="replace")

        if text.startswith("qSupported"):
            response = self.forward(payload)
            features = [item for item in response.split(b";")
                        if not item.startswith(b"QStartNoAckMode")]
            if b"qXfer:threads:read+" not in features:
                features.append(b"qXfer:threads:read+")
            return b";".join(features)
        if text == "QStartNoAckMode":
            # Keep both sides in acknowledgement mode.  This also handles
            # GDB versions which probe the command without first checking the
            # qSupported reply.
            return b""

        if text.startswith("qXfer:threads:read::"):
            try:
                offset_text, length_text = text.rsplit(":", 1)[1].split(",", 1)
                offset = int(offset_text, 16)
                length = int(length_text, 16)
            except ValueError:
                return b"E01"
            if offset == 0:
                self.refresh_threads()
            xml = self.chibios.xml()
            chunk = xml[offset:offset + length]
            return (b"l" if offset + len(chunk) >= len(xml) else b"m") + chunk

        if text == "qfThreadInfo":
            self.refresh_threads()
            ids = self.chibios.threads or {1: None}
            return b"m" + b",".join(b"%x" % item for item in ids)
        if text == "qsThreadInfo":
            return b"l"
        if text.startswith("qThreadExtraInfo,"):
            thread_id = self.thread_id(text.split(",", 1)[1])
            info = self.chibios.extra_info(thread_id)
            return b"E01" if info is None else info.encode("utf-8").hex().encode("ascii")
        if text == "qC":
            self.refresh_threads()
            return b"QC%x" % self.chibios.current

        if text.startswith("Hg"):
            thread_id = self.thread_id(text[2:])
            if thread_id in (0, -1):
                self.selected_thread = self.chibios.current
            elif thread_id == 1 and not self.chibios.threads:
                self.selected_thread = 1
            elif thread_id in self.chibios.threads:
                self.selected_thread = thread_id
            else:
                return b"E01"
            return self.forward(b"Hg1")
        if text.startswith("Hc"):
            return self.forward(b"Hc1" if text[2:] not in ("0", "-1") else payload)
        if text.startswith("T"):
            thread_id = self.thread_id(text[1:])
            alive = ((thread_id == 1 and not self.chibios.threads) or
                     thread_id in self.chibios.threads)
            return b"OK" if alive else b"E01"

        if text == "g" and self.selected_thread != self.chibios.current:
            try:
                return self.chibios.saved_registers(
                    self.selected_thread, self.forward(payload).decode("ascii")
                )
            except RuntimeError:
                return b"E01"
        if text.startswith("p") and self.selected_thread != self.chibios.current:
            try:
                register = int(text[1:], 16)
                if register < 16:
                    physical = self.forward(b"g").decode("ascii")
                    saved = self.chibios.saved_registers(
                        self.selected_thread, physical
                    )
                    return saved[register * 8:(register + 1) * 8]
                # The context-switch frame does not contain system and most
                # floating-point registers.  Ask Renode only to learn the
                # register width, then report the value as unavailable.
                physical = self.forward(payload)
                return b"x" * len(physical)
            except (RuntimeError, ValueError):
                return b"E01"
        if ((text.startswith("P") or text.startswith("G")) and
                self.selected_thread != self.chibios.current):
            return b"E01"

        execution = text.startswith(("c", "s", "vCont", "bc", "bs"))
        forwarded = self.physical_execution_packet(payload) if execution else payload
        if execution:
            self.threads_fresh = False
        response = (self.forward_execution(forwarded)
                    if execution else self.forward(forwarded))
        if text == "?" or execution:
            reverse = text.startswith(("bc", "bs"))
            if reverse and RSPConnection.is_stop(response):
                self.reconnect_upstream()
            self.refresh_threads()
            self.selected_thread = self.chibios.current
            marker = b"thread:1;"
            if marker in response:
                response = response.replace(
                    marker, b"thread:%x;" % self.chibios.current, 1
                )
            elif response.startswith(b"T") and b"thread:" not in response:
                response += b"thread:%x;" % self.chibios.current
            elif response.startswith(b"S"):
                response = (b"T" + response[1:] +
                            b"thread:%x;" % self.chibios.current)
        return response

    def run(self):
        try:
            while True:
                kind, payload = self.client.event()
                if kind in (b"+", b"-"):
                    continue
                if kind == b"\x03":
                    self.upstream.send_raw(kind)
                    response = self.upstream.receive_packet()
                    self.threads_fresh = False
                    self.refresh_threads()
                    self.selected_thread = self.chibios.current
                    if response.startswith(b"T") and b"thread:" not in response:
                        response += b"thread:%x;" % self.chibios.current
                else:
                    self.client.send_raw(b"+")
                    if payload.startswith(b"qRcmd,"):
                        self.forward_monitor_command(payload)
                        continue
                    try:
                        response = self.handle(payload)
                    except RuntimeError as error:
                        raise RuntimeError(
                            "%s while handling %r" % (error, payload)
                        ) from error
                self.client.send_packet(response)
        finally:
            self.upstream.sock.close()

    def forward_monitor_command(self, payload):
        """Relay a monitor command, including its zero or more O packets."""
        response = self.upstream.send_command(payload)
        while True:
            if response is None:
                response = self.upstream.receive_packet()
            while True:
                self.client.send_packet(response)
                kind, client_payload = self.client.event()
                if kind == b"+":
                    break
                if kind != b"-":
                    raise RuntimeError(
                        "GDB did not acknowledge monitor output: %r %r" %
                        (kind, client_payload)
                    )
            if not response.startswith(b"O"):
                return
            response = None


def connect_upstream(port):
    # The first reverse-execution checkpoint is created before Renode starts
    # its GDB server and can take well over 30 seconds on a large board.
    deadline = time.monotonic() + 300
    while True:
        try:
            upstream = socket.create_connection(("127.0.0.1", port), timeout=1)
            upstream.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            upstream.settimeout(None)
            return upstream
        except OSError:
            if time.monotonic() >= deadline:
                raise
            time.sleep(0.1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--listen", type=int, required=True)
    parser.add_argument("--upstream", type=int, required=True)
    parser.add_argument("--elf", required=True)
    parser.add_argument("--ready-file", required=True)
    args = parser.parse_args()
    symbols = elf_symbols(args.elf)
    ready_file = Path(args.ready_file)

    with socket.socket() as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(("127.0.0.1", args.listen))
        listener.listen(1)
        ready_file.touch()
        try:
            while True:
                client_socket, _ = listener.accept()
                try:
                    client_socket.setsockopt(
                        socket.IPPROTO_TCP, socket.TCP_NODELAY, 1
                    )
                    with client_socket, connect_upstream(args.upstream) as upstream_socket:
                        Proxy(
                            RSPConnection(client_socket),
                            RSPConnection(upstream_socket), args.upstream, symbols
                        ).run()
                except EOFError:
                    pass
                except (OSError, RuntimeError) as error:
                    print("ChibiOS GDB connection closed: %s" % error, file=sys.stderr)
        finally:
            try:
                ready_file.unlink()
            except FileNotFoundError:
                pass


if __name__ == "__main__":
    main()
