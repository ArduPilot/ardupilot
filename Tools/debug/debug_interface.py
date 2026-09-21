#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""GDB remote server for Cortex-M CrashCatcher dumps.

This is a portable Python replacement for the platform-specific CrashDebug
executables.  It combines FLASH from an ELF (or raw binary) with RAM and CPU
state from a CrashCatcher dump, then exposes that state through GDB's remote
serial protocol.  ChibiOS thread state is reconstructed from the kernel's
``ch_debug`` memory signature.
"""

import argparse
import dataclasses
import re
import struct
import sys

CRASH_SIGNATURE = b"cC"
CRASH_VERSION = (3, 0)
CRASH_VERSION_2 = (2, 0)
CRASH_FLAG_FLOATING_POINT = 1
DEFAULT_STACK_POINTER = 0xBAADBAAD
STACK_OVERFLOW_MARKER = b"\xAC\xCE\x55\xED"
PACKET_SIZE = 0x4000
MAX_THREADS = 50

PT_LOAD = 1
SHT_SYMTAB = 2
SHT_DYNSYM = 11

R0 = 0
R4 = 4
R11 = 11
SP = 13
LR = 14
PC = 15
XPSR = 16
MSP = 17
PSP = 18
CORE_REGISTER_COUNT = 19
FLOAT_REGISTER_COUNT = 33

THREAD_STATES = (
    "READY", "CURRENT", "WTSTART", "SUSPENDED", "QUEUED", "WTSEM",
    "WTMTX", "WTCOND", "SLEEPING", "WTEXIT", "WTOREVT", "WTANDEVT",
    "SNDMSGQ", "SNDMSG", "WTMSG", "FINAL",
)


TARGET_XML = """<?xml version="1.0"?>
<!DOCTYPE feature SYSTEM "gdb-target.dtd">
<target>
<feature name="org.gnu.gdb.arm.m-profile">
<reg name="r0" bitsize="32"/>
<reg name="r1" bitsize="32"/>
<reg name="r2" bitsize="32"/>
<reg name="r3" bitsize="32"/>
<reg name="r4" bitsize="32"/>
<reg name="r5" bitsize="32"/>
<reg name="r6" bitsize="32"/>
<reg name="r7" bitsize="32"/>
<reg name="r8" bitsize="32"/>
<reg name="r9" bitsize="32"/>
<reg name="r10" bitsize="32"/>
<reg name="r11" bitsize="32"/>
<reg name="r12" bitsize="32"/>
<reg name="sp" bitsize="32" type="data_ptr"/>
<reg name="lr" bitsize="32"/>
<reg name="pc" bitsize="32" type="code_ptr"/>
<reg name="xpsr" bitsize="32" regnum="25"/>
</feature>
<feature name="org.gnu.gdb.arm.m-system">
<reg name="msp" bitsize="32" regnum="26"/>
<reg name="psp" bitsize="32" regnum="27"/>
</feature>
{floating_point}
</target>
"""

FPU_XML = """<feature name="org.gnu.gdb.arm.vfp">
<reg name="d0" bitsize="64" type="ieee_double"/>
<reg name="d1" bitsize="64" type="ieee_double"/>
<reg name="d2" bitsize="64" type="ieee_double"/>
<reg name="d3" bitsize="64" type="ieee_double"/>
<reg name="d4" bitsize="64" type="ieee_double"/>
<reg name="d5" bitsize="64" type="ieee_double"/>
<reg name="d6" bitsize="64" type="ieee_double"/>
<reg name="d7" bitsize="64" type="ieee_double"/>
<reg name="d8" bitsize="64" type="ieee_double"/>
<reg name="d9" bitsize="64" type="ieee_double"/>
<reg name="d10" bitsize="64" type="ieee_double"/>
<reg name="d11" bitsize="64" type="ieee_double"/>
<reg name="d12" bitsize="64" type="ieee_double"/>
<reg name="d13" bitsize="64" type="ieee_double"/>
<reg name="d14" bitsize="64" type="ieee_double"/>
<reg name="d15" bitsize="64" type="ieee_double"/>
<reg name="fpscr" bitsize="32" type="int" group="float"/>
</feature>"""


class CrashDebugError(Exception):
    """Input data cannot be used for post-mortem debugging."""


class MemoryAccessError(CrashDebugError):
    """The requested target address isn't present in the dump or image."""


@dataclasses.dataclass
class MemoryRegion:
    start: int
    data: bytearray
    read_only: bool
    kind: str

    @property
    def end(self):
        return self.start + len(self.data)


class MemoryImage:
    """Sparse 32-bit address space assembled from ELF and dump regions."""

    def __init__(self):
        self.regions = []
        self.aliases = []

    def add(self, start, data, read_only=False, kind="ram"):
        self.regions.append(MemoryRegion(start, bytearray(data), read_only, kind))

    def add_alias(self, start, size, target):
        self.aliases.append((start, start + size, target))

    def _translate(self, address, size):
        for start, end, target in self.aliases:
            if start <= address and address + size <= end:
                return target + address - start
        return address

    def _region(self, address, size):
        address = self._translate(address, size)
        for region in self.regions:
            if region.start <= address and address + size <= region.end:
                return region, address - region.start
        raise MemoryAccessError(f"memory 0x{address:08x}+0x{size:x} is unavailable")

    def read(self, address, size):
        region, offset = self._region(address, size)
        return bytes(region.data[offset:offset + size])

    def write(self, address, data):
        region, offset = self._region(address, len(data))
        if region.read_only:
            raise MemoryAccessError(f"memory 0x{address:08x} is read-only")
        region.data[offset:offset + len(data)] = data

    def u8(self, address):
        return self.read(address, 1)[0]

    def u32(self, address):
        return struct.unpack("<I", self.read(address, 4))[0]

    def c_string(self, address, limit=63):
        if address == 0:
            return ""
        result = bytearray()
        for offset in range(limit):
            value = self.u8(address + offset)
            if value == 0:
                break
            result.append(value)
        return result.decode("utf-8", errors="replace")

    def memory_map_xml(self):
        items = [
            '<?xml version="1.0"?>',
            '<!DOCTYPE memory-map PUBLIC "+//IDN gnu.org//DTD GDB Memory Map V1.0//EN" '
            '"http://sourceware.org/gdb/gdb-memory-map.dtd">',
            "<memory-map>",
        ]
        for region in self.regions:
            items.append(f'<memory type="{region.kind}" start="0x{region.start:x}" length="0x{len(region.data):x}">')
            if region.kind == "flash":
                items.append('<property name="blocksize">1</property>')
            items.append("</memory>")
        for start, end, target in self.aliases:
            try:
                target_region, _ = self._region(target, 1)
                kind = target_region.kind
            except MemoryAccessError:
                kind = "ram"
            items.append(f'<memory type="{kind}" start="0x{start:x}" length="0x{end - start:x}">')
            if kind == "flash":
                items.append('<property name="blocksize">1</property>')
            items.append("</memory>")
        items.append("</memory-map>")
        return "".join(items)


@dataclasses.dataclass
class ElfImage:
    data: bytes
    symbols: dict


def _elf_sections(data):
    if len(data) < 52 or data[:4] != b"\x7fELF" or data[4:6] != b"\x01\x01":
        raise CrashDebugError("only little-endian ELF32 images are supported")
    header = struct.unpack_from("<16sHHIIIIIHHHHHH", data)
    if header[1] != 2:
        raise CrashDebugError("ELF image is not executable")
    section_offset, section_size, section_count = header[6], header[11], header[12]
    if section_size < 40:
        raise CrashDebugError("ELF has an invalid section header size")
    sections = []
    for index in range(section_count):
        offset = section_offset + index * section_size
        if offset + 40 > len(data):
            raise CrashDebugError("ELF section header is outside the file")
        sections.append(struct.unpack_from("<IIIIIIIIII", data, offset))
    return header, sections


def load_elf(filename, memory):
    with open(filename, "rb") as file:
        data = file.read()
    header, sections = _elf_sections(data)
    program_offset, program_size, program_count = header[5], header[9], header[10]
    if program_size < 32:
        raise CrashDebugError("ELF has an invalid program header size")
    loaded = 0
    for index in range(program_count):
        offset = program_offset + index * program_size
        if offset + 32 > len(data):
            raise CrashDebugError("ELF program header is outside the file")
        program = struct.unpack_from("<IIIIIIII", data, offset)
        program_type, file_offset, _, physical_address, file_size, memory_size, _, _ = program
        if program_type != PT_LOAD or file_size == 0 or memory_size < file_size:
            continue
        if file_offset + file_size > len(data):
            raise CrashDebugError("ELF load segment is outside the file")
        memory.add(physical_address, data[file_offset:file_offset + file_size], read_only=True, kind="flash")
        loaded += 1
    if loaded == 0:
        raise CrashDebugError("ELF contains no loadable data")

    symbols = {}
    for section in sections:
        _, section_type, _, _, offset, size, link, _, _, entry_size = section
        if section_type not in (SHT_SYMTAB, SHT_DYNSYM) or entry_size < 16 or link >= len(sections):
            continue
        string_section = sections[link]
        string_offset, string_size = string_section[4], string_section[5]
        if offset + size > len(data) or string_offset + string_size > len(data):
            raise CrashDebugError("ELF symbol table is outside the file")
        strings = data[string_offset:string_offset + string_size]
        for symbol_offset in range(offset, offset + size, entry_size):
            if symbol_offset + 16 > len(data):
                break
            name_offset, value, _, _, _, _ = struct.unpack_from("<IIIBBH", data, symbol_offset)
            if value == 0 or name_offset >= len(strings):
                continue
            name_end = strings.find(b"\0", name_offset)
            if name_end < 0:
                continue
            name = strings[name_offset:name_end].decode("utf-8", errors="replace")
            symbols[name] = value
    return ElfImage(data, symbols)


@dataclasses.dataclass
class CrashContext:
    flags: int
    registers: list
    exception_psr: int
    floating_point: list

    def copy_registers(self):
        return self.registers.copy(), self.floating_point.copy()


def _unpad_sd_dump(data):
    """Return just the CrashCatcher payload from a preallocated SD file."""
    if len(data) < 512 or len(data) % 512 != 0:
        return data
    dump_size = struct.unpack_from("<I", data, len(data) - 4)[0]
    if 0 < dump_size < len(data) and all(value == 0xFF for value in data[dump_size:min(dump_size + 16, len(data))]):
        return data[:dump_size]
    return data


GDB_REGISTER_INDEX = {
    **{f"r{index}": index for index in range(13)},
    "sp": SP,
    "lr": LR,
    "pc": PC,
    "xpsr": XPSR,
    "msp": MSP,
    "psp": PSP,
}


def _load_gdb_log(data, memory):
    """Load memory and registers emitted by Tools/debug/crash_dump.scr."""
    try:
        lines = data.decode("utf-8", errors="replace").splitlines()
    except AttributeError as error:
        raise CrashDebugError("invalid GDB log data") from error
    registers = [0] * CORE_REGISTER_COUNT
    registers[MSP] = registers[PSP] = DEFAULT_STACK_POINTER
    floating_point = [0] * FLOAT_REGISTER_COUNT
    has_floating_point = False
    memory_lines = []
    memory_pattern = re.compile(r"^0x([0-9a-fA-F]{8})(?:\s+<[^>]*>)?:\s*(.*)$")
    register_pattern = re.compile(r"^(r(?:1[0-2]|[0-9])|sp|lr|pc|xpsr|msp|psp|s(?:[12][0-9]|3[01]|[0-9])|fpscr)\s+(.*)$")
    for line in lines:
        match = memory_pattern.match(line)
        if match:
            address = int(match.group(1), 16)
            values_text = re.sub(r"<[^>]*>", "", match.group(2))
            values = [int(value, 16) for value in re.findall(r"(?<!\S)0x[0-9a-fA-F]+", values_text)[:4]]
            if values:
                memory_lines.append((address, values))
            continue
        match = register_pattern.match(line)
        if not match:
            continue
        name, value_text = match.groups()
        if name.startswith("s") and name != "sp":
            raw = re.search(r"\(raw\s+(0x[0-9a-fA-F]+)\)", value_text)
            floating_point[int(name[1:])] = int(raw.group(1), 0) if raw else 0xFFFFFFFF
            has_floating_point = True
        elif name == "fpscr":
            floating_point[-1] = int(value_text.split()[0], 0)
        else:
            registers[GDB_REGISTER_INDEX[name]] = int(value_text.split()[0], 0)

    if not memory_lines:
        raise CrashDebugError("file is neither a CrashCatcher dump nor a GDB memory log")
    region_start = None
    region_data = bytearray()
    expected_address = None
    for address, values in memory_lines:
        if address != expected_address:
            if region_start is not None:
                memory.add(region_start, region_data)
            region_start = address
            region_data = bytearray()
        for value in values:
            region_data.extend(struct.pack("<I", value))
        expected_address = address + len(values) * 4
    if region_start is not None:
        memory.add(region_start, region_data)
    flags = CRASH_FLAG_FLOATING_POINT if has_floating_point else 0
    return CrashContext(flags, registers, 0, floating_point if has_floating_point else [])


def load_dump(filename, memory):
    with open(filename, "rb") as file:
        data = file.read()
    data = _unpad_sd_dump(data)
    if data[:2] != CRASH_SIGNATURE:
        if data[:4].upper() == b"6343":
            try:
                data = bytes.fromhex(data.decode("ascii"))
            except (UnicodeDecodeError, ValueError) as error:
                raise CrashDebugError("invalid hexadecimal CrashCatcher dump") from error
        else:
            return _load_gdb_log(data, memory)
    if len(data) < 8 or data[:2] != CRASH_SIGNATURE:
        raise CrashDebugError("dump is too short or has an invalid CrashCatcher signature")
    version = tuple(data[2:4])
    if version not in (CRASH_VERSION, CRASH_VERSION_2):
        raise CrashDebugError(f"unsupported CrashCatcher dump version {version[0]}.{version[1]}")

    offset = 4

    def take(size, description):
        nonlocal offset
        if offset + size > len(data):
            raise CrashDebugError(f"dump is too short to contain {description}")
        result = data[offset:offset + size]
        offset += size
        return result

    flags = struct.unpack("<I", take(4, "flags"))[0]
    integer_count = CORE_REGISTER_COUNT if version == CRASH_VERSION else CORE_REGISTER_COUNT - 2
    registers = list(struct.unpack(f"<{integer_count}I", take(integer_count * 4, "integer registers")))
    if version == CRASH_VERSION_2:
        registers.extend((DEFAULT_STACK_POINTER, DEFAULT_STACK_POINTER))
    exception_psr = struct.unpack("<I", take(4, "exception PSR"))[0]
    floating_point = []
    if flags & CRASH_FLAG_FLOATING_POINT:
        floating_point = list(struct.unpack(f"<{FLOAT_REGISTER_COUNT}I", take(FLOAT_REGISTER_COUNT * 4,
                                                                              "floating-point registers")))

    while offset < len(data):
        remaining = len(data) - offset
        if remaining == len(STACK_OVERFLOW_MARKER) and data[offset:] == STACK_OVERFLOW_MARKER:
            raise CrashDebugError("CrashCatcher detected a stack overflow while creating the dump")
        if remaining < 8:
            raise CrashDebugError("dump contains a truncated memory-region header")
        start, end = struct.unpack("<II", take(8, "memory-region header"))
        if end < start:
            raise CrashDebugError(f"dump has an invalid memory region 0x{start:08x}-0x{end:08x}")
        memory.add(start, take(end - start, f"memory region 0x{start:08x}-0x{end:08x}"))
    return CrashContext(flags, registers, exception_psr, floating_point)


@dataclasses.dataclass
class ChibiOSThread:
    identifier: int
    name: str
    state: str

    @property
    def extra_info(self):
        if self.name == "Current Execution" and self.state == "No RTOS thread":
            return self.state
        return f"Name: {self.name} State: {self.state}"


class ChibiOSThreads:
    """Decode ChibiOS's self-describing registry and saved contexts."""

    LEGACY_SIGNATURE_SIZE = 22
    CURRENT_SIGNATURE_SIZE = 43

    def __init__(self, memory, symbols, crash_context):
        self.memory = memory
        self.symbols = symbols
        self.crash_context = crash_context
        self.signature = None
        self.threads = []
        self.current = 0
        try:
            self._load()
        except MemoryAccessError:
            # A partial dump can still provide useful fault registers and a
            # backtrace even when it does not contain the ChibiOS registry.
            self.signature = None
            self.threads = []
            self.current = 0

    def _load(self):
        signature_address = self.symbols.get("ch_debug", 0)
        if signature_address == 0:
            return
        prefix = self.memory.read(signature_address, self.LEGACY_SIGNATURE_SIZE)
        if prefix[:4] != b"main" or prefix[4] != 0:
            return
        signature_size = prefix[5]
        if signature_size < self.LEGACY_SIGNATURE_SIZE:
            return
        signature = self.memory.read(signature_address, min(signature_size, self.CURRENT_SIGNATURE_SIZE))
        if signature[8] != 4:
            return
        self.signature = signature
        if signature_size >= self.CURRENT_SIGNATURE_SIZE:
            self._load_current()
        else:
            self._load_legacy()
        self._load_details()
        for index, thread in enumerate(self.threads):
            if thread.identifier == self.current:
                self.threads[0], self.threads[index] = self.threads[index], self.threads[0]
                break

    def _offset(self, index):
        return self.signature[index]

    def _load_current(self):
        # ch_debug byte offsets mirror struct chibios_chdebug in ChibiOS.
        newer, older = self._offset(13), self._offset(14)
        system = self.symbols.get("ch_system", 0)
        instance = self.symbols.get("ch0", 0)
        system_instances, system_registry = self._offset(30), self._offset(31)
        instance_current, instance_registry = self._offset(37), self._offset(40)
        if system:
            instance = self.memory.u32(system + system_instances)
        if instance == 0:
            return
        registry = system + system_registry if system_registry else instance + instance_registry
        previous_node = registry
        node = self.memory.u32(registry)
        identifiers = []
        while node != registry:
            if node == 0 or len(identifiers) >= MAX_THREADS:
                self.threads = []
                return
            thread = node - newer
            if self.memory.u32(thread + older) != previous_node:
                self.threads = []
                return
            identifiers.append(thread)
            previous_node = node
            node = self.memory.u32(thread + newer)
        self.current = self.memory.u32(instance + instance_current)
        self.threads = [ChibiOSThread(identifier, "", "") for identifier in identifiers]

    def _load_legacy(self):
        system = self.symbols.get("ch", 0)
        if system == 0:
            return
        newer, older, current_offset = self._offset(13), self._offset(14), self._offset(15)
        current = previous = system
        identifiers = []
        while True:
            current = self.memory.u32(current + newer)
            if current == 0 or self.memory.u32(current + older) != previous:
                self.threads = [ChibiOSThread(1, "Current Execution", "No RTOS thread")]
                self.current = 1
                return
            if current == system:
                break
            if len(identifiers) >= MAX_THREADS:
                return
            identifiers.append(current)
            previous = current
        self.current = self.memory.u32(system + current_offset)
        self.threads = [ChibiOSThread(identifier, "", "") for identifier in identifiers]

    def _load_details(self):
        if not self.signature:
            return
        if (len(self.threads) == 1 and self.threads[0].identifier == 1 and
                self.threads[0].name == "Current Execution"):
            return
        name_offset, state_offset = self._offset(15), self._offset(17)
        for thread in self.threads:
            try:
                name_pointer = self.memory.u32(thread.identifier + name_offset)
                thread.name = self.memory.c_string(name_pointer) or "No Name"
            except MemoryAccessError:
                thread.name = "No Name"
            try:
                state = self.memory.u8(thread.identifier + state_offset)
                thread.state = THREAD_STATES[state] if state < len(THREAD_STATES) else "Unknown"
            except MemoryAccessError:
                thread.state = "Unknown"

    def context(self, identifier):
        registers, floating_point = self.crash_context.copy_registers()
        if identifier == self.current:
            return registers, floating_point
        if not self.signature or not any(thread.identifier == identifier for thread in self.threads):
            raise CrashDebugError(f"unknown ChibiOS thread 0x{identifier:x}")
        context_offset = self._offset(12)
        stack_pointer = self.memory.u32(identifier + context_offset)
        if stack_pointer == 0:
            raise CrashDebugError(f"thread 0x{identifier:x} has a null saved stack pointer")
        if len(self.signature) >= self.CURRENT_SIGNATURE_SIZE:
            context_size = self._offset(26)
            if context_size < 36:
                raise CrashDebugError("ChibiOS saved context is smaller than the core register context")
            core_offset = context_size - 36
            stack_data = self.memory.read(stack_pointer, context_size)
            for register in range(R4, R11 + 1):
                registers[register] = struct.unpack_from("<I", stack_data, core_offset + (register - R4) * 4)[0]
            registers[LR] = registers[PC] = struct.unpack_from("<I", stack_data, context_size - 4)[0]
        else:
            fpu_enabled = bool(self.crash_context.flags & CRASH_FLAG_FLOATING_POINT)
            if fpu_enabled:
                try:
                    fpu_enabled = bool(self.memory.u32(0xE000ED88) & 0x00F00000)
                except MemoryAccessError:
                    fpu_enabled = False
            context_size = 0x64 if fpu_enabled else 0x24
            core_offset = 0x40 if fpu_enabled else 0
            stack_data = self.memory.read(stack_pointer, context_size)
            for register in range(R4, R11 + 1):
                registers[register] = struct.unpack_from("<I", stack_data, core_offset + (register - R4) * 4)[0]
            registers[PC] = struct.unpack_from("<I", stack_data, context_size - 4)[0]
        registers[SP] = stack_pointer + context_size
        return registers, floating_point

    def get(self, identifier):
        return next((thread for thread in self.threads if thread.identifier == identifier), None)


class GDBServer:
    """Small post-mortem GDB Remote Serial Protocol server."""

    def __init__(self, memory, context, threads):
        self.memory = memory
        self.context = context
        self.threads = threads
        self.selected_thread = threads.current
        self.registers, self.floating_point = context.copy_registers()
        self.no_ack = False
        self.stop = False
        exception = context.exception_psr & 0xFF
        self.signal = {2: 2, 3: 11, 4: 11, 5: 10, 6: 4, 12: 5}.get(exception, 19)
        self.target_xml = TARGET_XML.format(floating_point=FPU_XML if context.floating_point else "")
        self.memory_map = memory.memory_map_xml()

    @staticmethod
    def _packet(payload):
        encoded = payload.encode("ascii")
        return b"$" + encoded + b"#" + f"{sum(encoded) & 0xFF:02x}".encode("ascii")

    @staticmethod
    def _unescape(payload):
        result = bytearray()
        index = 0
        while index < len(payload):
            value = payload[index]
            if value == ord("}"):
                index += 1
                if index >= len(payload):
                    return None
                value = payload[index] ^ 0x20
            result.append(value)
            index += 1
        return result

    def _read_packet(self):
        stdin = sys.stdin.buffer
        while True:
            value = stdin.read(1)
            if not value:
                return None
            if value == b"\x03":
                return "\x03"
            if value != b"$":
                continue
            payload = bytearray()
            while True:
                value = stdin.read(1)
                if not value:
                    return None
                if value == b"#":
                    break
                payload.extend(value)
            checksum = stdin.read(2)
            try:
                valid = int(checksum, 16) == (sum(payload) & 0xFF)
            except ValueError:
                valid = False
            if not self.no_ack:
                sys.stdout.buffer.write(b"+" if valid else b"-")
                sys.stdout.buffer.flush()
            if valid:
                payload = self._unescape(payload)
                if payload is not None:
                    return payload.decode("latin-1")

    def _send(self, payload):
        sys.stdout.buffer.write(self._packet(payload))
        sys.stdout.buffer.flush()

    @staticmethod
    def _xfer(data, request):
        try:
            location = request.rsplit(":", 1)[1]
            offset_text, length_text = location.split(",", 1)
            offset, length = int(offset_text, 16), int(length_text, 16)
        except (IndexError, ValueError):
            return "E01"
        if offset >= len(data):
            return "l"
        chunk = data[offset:offset + length]
        return ("l" if offset + len(chunk) >= len(data) else "m") + chunk

    def _stop_reply(self):
        result = f"T{self.signal:02x}"
        for register in (12, SP, LR, PC):
            result += f"{register:02x}:" + struct.pack("<I", self.registers[register]).hex() + ";"
        if self.threads.current:
            result += f"thread:{self.threads.current:x};"
        return result

    def _register_bytes(self):
        values = self.registers + self.floating_point
        return struct.pack(f"<{len(values)}I", *values)

    def _select_thread(self, identifier):
        if identifier in (0, -1):
            return "OK"
        try:
            self.registers, self.floating_point = self.threads.context(identifier)
        except (CrashDebugError, MemoryAccessError):
            return "E01"
        self.selected_thread = identifier
        return "OK"

    def command(self, request):
        if request == "\x03" or request == "?":
            return self._stop_reply()
        if request.startswith("qSupported"):
            return f"qXfer:memory-map:read+;qXfer:features:read+;QStartNoAckMode+;PacketSize={PACKET_SIZE:x}"
        if request == "QStartNoAckMode":
            return "OK"
        if request.startswith("qXfer:features:read:target.xml:"):
            return self._xfer(self.target_xml, request)
        if request.startswith("qXfer:memory-map:read::"):
            return self._xfer(self.memory_map, request)
        if request == "qfThreadInfo":
            if not self.threads.threads:
                return "l"
            return "m" + ",".join(f"{thread.identifier:x}" for thread in self.threads.threads)
        if request == "qsThreadInfo":
            return "l"
        if request.startswith("qThreadExtraInfo,"):
            try:
                thread = self.threads.get(int(request.split(",", 1)[1], 16))
            except ValueError:
                return "E01"
            return thread.extra_info.encode().hex() if thread else ""
        if request == "qC":
            return f"QC{self.threads.current:x}" if self.threads.current else ""
        if request.startswith("qSymbol"):
            return "OK"
        if request == "qAttached":
            return "1"
        if request.startswith("q") or request.startswith("vMustReplyEmpty"):
            return ""
        if request.startswith("H"):
            if len(request) < 3 or request[1] not in "gc":
                return "E01"
            try:
                identifier = int(request[2:], 16) if not request[2:].startswith("-") else -1
            except ValueError:
                return "E01"
            return self._select_thread(identifier) if request[1] == "g" else "OK"
        if request.startswith("T"):
            try:
                identifier = int(request[1:], 16)
            except ValueError:
                return "E01"
            return "OK" if self.threads.get(identifier) else "E01"
        if request == "g":
            return self._register_bytes().hex()
        if request.startswith("G"):
            try:
                values = bytes.fromhex(request[1:])
            except ValueError:
                return "E01"
            expected = (len(self.registers) + len(self.floating_point)) * 4
            if len(values) != expected:
                return "E01"
            unpacked = list(struct.unpack(f"<{expected // 4}I", values))
            self.registers = unpacked[:CORE_REGISTER_COUNT]
            self.floating_point = unpacked[CORE_REGISTER_COUNT:]
            return "OK"
        if request.startswith("m"):
            try:
                address_text, size_text = request[1:].split(",", 1)
                return self.memory.read(int(address_text, 16), int(size_text, 16)).hex()
            except (ValueError, MemoryAccessError):
                return "E03"
        if request.startswith("M"):
            try:
                location, value = request[1:].split(":", 1)
                address_text, size_text = location.split(",", 1)
                data = bytes.fromhex(value)
                if len(data) != int(size_text, 16):
                    return "E01"
                self.memory.write(int(address_text, 16), data)
                return "OK"
            except (ValueError, MemoryAccessError):
                return "E03"
        if request.startswith("X"):
            try:
                location, value = request[1:].split(":", 1)
                address_text, size_text = location.split(",", 1)
                data = value.encode("latin-1")
                if len(data) != int(size_text, 16):
                    return "E01"
                self.memory.write(int(address_text, 16), data)
                return "OK"
            except (UnicodeEncodeError, ValueError, MemoryAccessError):
                return "E03"
        if request in ("c", "s") or request[:1] in ("C", "S"):
            return self._stop_reply()
        if request[:1] in ("z", "Z"):
            return "OK"
        if request == "D":
            self.stop = True
            return "OK"
        if request == "k" or request.startswith("vKill"):
            self.stop = True
            return "OK"
        return ""

    def run(self):
        while not self.stop:
            request = self._read_packet()
            if request is None:
                break
            response = self.command(request)
            self._send(response)
            if request == "QStartNoAckMode" and response == "OK":
                self.no_ack = True


def create_server(arguments):
    memory = MemoryImage()
    symbols = {}
    if arguments.elf:
        symbols = load_elf(arguments.elf, memory).symbols
    else:
        with open(arguments.binary[0], "rb") as file:
            memory.add(int(arguments.binary[1], 0), file.read(), read_only=True, kind="flash")
    context = load_dump(arguments.dump, memory)
    for start, size, target in arguments.alias:
        memory.add_alias(int(start, 0), int(size, 0), int(target, 0))
    threads = ChibiOSThreads(memory, symbols, context)
    return GDBServer(memory, context, threads)


def main():
    parser = argparse.ArgumentParser(description="debug a Cortex-M CrashCatcher dump with GDB")
    image = parser.add_mutually_exclusive_group(required=True)
    image.add_argument("--elf", help="ELF containing the firmware FLASH image and symbols")
    image.add_argument("--bin", dest="binary", nargs=2, metavar=("IMAGE", "BASE_ADDRESS"),
                       help="raw firmware image and its target base address")
    parser.add_argument("--dump", required=True, help="binary or hexadecimal CrashCatcher dump")
    parser.add_argument("--alias", action="append", nargs=3, default=[],
                        metavar=("BASE_ADDRESS", "SIZE", "REDIRECT_ADDRESS"),
                        help="redirect an address range to another memory range")
    arguments = parser.parse_args()
    try:
        create_server(arguments).run()
    except (CrashDebugError, OSError, struct.error) as error:
        print(f"debug_interface.py: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
