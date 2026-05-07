#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""RP2350 heap budget analyzer.

This tool combines linker-map SRAM numbers with source-level allocation
callsites to suggest a safer minimum heap threshold for RP2350 targets.

It is intentionally conservative: the suggestion is a heuristic, not a proof.
Use the output as evidence for setting __rp2350_min_heap__ in common_rp2350_smp.ld.
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple


RE_MAP_SYMBOL = re.compile(r"^\s*0x([0-9a-fA-F]+)\s+.*?(__[A-Za-z0-9_]+)\b")

# Heap allocation patterns from C/C++ code. Keep these simple and fast.
RE_MALLOC = re.compile(r"\bmalloc\s*\(")
RE_CALLOC = re.compile(r"\bcalloc\s*\(")
RE_REALLOC = re.compile(r"\brealloc\s*\(")
RE_NEW = re.compile(r"\bnew\b")
RE_NEW_ARRAY = re.compile(r"\bnew\s+[^;\n\[]+\[")

# Try to extract literal allocation size expressions for extra signal.
RE_MALLOC_LITERAL = re.compile(r"\bmalloc\s*\(\s*(0x[0-9a-fA-F]+|\d+)\s*\)")
RE_CALLOC_LITERAL = re.compile(
    r"\bcalloc\s*\(\s*(0x[0-9a-fA-F]+|\d+)\s*,\s*(0x[0-9a-fA-F]+|\d+)\s*\)"
)
RE_NEW_ARRAY_LITERAL = re.compile(
    r"\bnew\s+[A-Za-z_][A-Za-z0-9_:<>\*\s]*\[\s*(0x[0-9a-fA-F]+|\d+)\s*\]"
)
RE_BLOCK_COMMENT = re.compile(r"/\*.*?\*/", re.DOTALL)

# Parse useful runtime profiling rows from @SYS/threads.txt and @SYS/tasks.txt.
# Group 3 is either a decimal total or the literal "MSP" (emitted by newer firmware for idle threads that run on the hardware main stack and have no conventional ChibiOS working area).
RE_THREADS_STACK_ROW = re.compile(
    r"^\s*([A-Za-z0-9_:/\-\.\*]+)\s+PRI=\s*\d+\s+sp=0x[0-9a-fA-F]+\s+STACK=(\d+)/(\d+|MSP)"
)
RE_TASKS_AVG = re.compile(r"AVG=(\d+)")
RE_TASKS_OVR = re.compile(r"OVR=(\d+)")
RE_TASKS_TOT = re.compile(r"TOT=([0-9]+(?:\.[0-9]+)?)%")
SOURCE_EXTS = {".c", ".cc", ".cpp", ".cxx", ".h", ".hpp", ".hh", ".hxx", ".ipp"}

HWDEF_INT_KEYS = {
    "MAIN_STACK",
    "FLASH_SIZE_KB",
    "STORAGE_FLASH_PAGE",
    "STORAGE_FLASH_PAGES",
    "SCHED_LOOP_RATE",
}

HWDEF_BOOL_DEFINE_KEYS = {
    "AP_FLASH_STORAGE_DOUBLE_PAGE",
    "AP_FLASH_STORAGE_QUAD_PAGE",
    "AP_COMPASS_PROBING_ENABLED",
    "HAL_ENABLE_SAVE_PERSISTENT_PARAMS",
    "HAL_HAVE_IMU_HEATER",
    "HAL_LOGGING_FILESYSTEM_ENABLED",
    "AP_SCRIPTING_ENABLED",
}


@dataclass
class MapMetrics:
    ram0_start: int
    ram0_size: int
    ram0_end: int
    heap_base: int
    heap_end: int
    heap_size: int
    min_heap: int
    sram_allowed: int
    sram_used: int
    sram_over_limit: int
    ramfunc_start: int
    ramfunc_end: int
    ramfunc_size: int
    # Per-section sizes extracted from the linker map (0 when absent).
    # These let us show a full slice-by-slice RAM breakdown without ambiguity.
    c0_msp_size: int     # core0 IRQ/main stack (descends from __main_stack_end__)
    c0_psp_size: int     # core0 thread/process stack
    c1_msp_size: int     # core1 IRQ/main stack (0 when SMP not enabled)
    c1_psp_size: int     # core1 thread/process stack (0 when SMP not enabled)
    data_size: int       # .data section (initialized globals, excluding RAMFUNC bytes)
    bss_size: int        # .bss section (includes thread working areas and all other globals)


@dataclass
class AllocationMetrics:
    malloc_calls: int
    calloc_calls: int
    realloc_calls: int
    new_calls: int
    new_array_calls: int
    literal_bytes: int
    files_scanned: int
    callsites: List[Tuple[str, int, str]]


@dataclass
class HwdefMetrics:
    hwdef_path: str
    main_stack: Optional[int]
    flash_size_kb: Optional[int]
    storage_flash_page: Optional[int]
    storage_flash_pages: Optional[int]
    sched_loop_rate: Optional[int]
    bool_defines: Dict[str, int]


@dataclass
class ThreadRow:
    name: str
    used: int
    total: int
# True when the total value is invalid (e.g.
# ChibiOS idle thread reports a wrapped/garbage stack-size because it runs on the MSP rather than a normal working area, producing UINT32_MAX-like values).
    invalid_total: bool

    @property
    def used_pct(self) -> Optional[int]:
        if self.invalid_total:
            return None
        return int((self.used * 100) / self.total) if self.total > 0 else None


@dataclass
class RuntimeMetrics:
    threads_file: Optional[str]
    tasks_file: Optional[str]
    max_thread_used_pct: Optional[int]
    worst_thread_name: Optional[str]
    fast_loop_avg_us: Optional[int]
    total_overruns: Optional[int]
    max_task_tot_pct: Optional[float]
    # All parsed thread rows, including those with invalid totals.
    thread_rows: List[ThreadRow]
# Byte-level sums across all threads for RAM accounting.
# total_stack_used_bytes: sum of bytes "used" from every thread (including MSP-based idle).
    total_stack_used_bytes: int = 0
    total_stack_allocated_bytes: int = 0


def parse_int_literal(value: str) -> int:
    return int(value, 16) if value.lower().startswith("0x") else int(value)


def parse_map_symbols(map_path: Path) -> Dict[str, int]:
    symbols: Dict[str, int] = {}
    with map_path.open("r", encoding="utf-8", errors="replace") as f:
        for line in f:
            m = RE_MAP_SYMBOL.match(line)
            if not m:
                continue
            addr = int(m.group(1), 16)
            symbol = m.group(2)
            symbols[symbol] = addr
    return symbols


def required_symbol(symbols: Dict[str, int], name: str) -> int:
    if name not in symbols:
        raise KeyError(f"missing linker symbol: {name}")
    return symbols[name]


def collect_map_metrics(map_path: Path) -> MapMetrics:
    symbols = parse_map_symbols(map_path)
    ram0_start = required_symbol(symbols, "__ram0_start__")
    ram0_size = required_symbol(symbols, "__ram0_size__")
    ram0_end = required_symbol(symbols, "__ram0_end__")
    heap_base = required_symbol(symbols, "__heap_base__")
    heap_end = required_symbol(symbols, "__heap_end__")
    min_heap = required_symbol(symbols, "__rp2350_min_heap__")
    ramfunc_start = required_symbol(symbols, "__ramfunc_start__")
    ramfunc_end = required_symbol(symbols, "__ramfunc_end__")

    # Compute derived metrics from base addresses; linker map "value" for
    # expression symbols can be location-counter based, not evaluated result.
    heap_size = heap_end - heap_base
    sram_used = heap_base - ram0_start
    sram_allowed = ram0_size - min_heap
    sram_over_limit = max(sram_used - sram_allowed, 0)
    ramfunc_size = ramfunc_end - ramfunc_start

    # Optional per-section symbols: parse defensively so the script still works
    # on simpler linker scripts that omit them.
    def _span(base_sym: str, end_sym: str) -> int:
        """Return end_sym - base_sym if both exist and make sense, else 0."""
        b = symbols.get(base_sym, 0)
        e = symbols.get(end_sym, 0)
        return (e - b) if (b and e and e > b) else 0

    # Core0 stacks: MSP lives at the very bottom of RAM in ChibiOS SMP layout.
    c0_msp_size = _span("__main_stack_base__", "__main_stack_end__")
    c0_psp_size = _span("__process_stack_base__", "__process_stack_end__")
    # Core1 stacks: only present when SMP is enabled (RP_CORE1_START == TRUE).
    c1_msp_size = _span("__c1_main_stack_base__", "__c1_main_stack_end__")
    c1_psp_size = _span("__c1_process_stack_base__", "__c1_process_stack_end__")
# .data: initialized globals, NOT counting the RAMFUNC bytes that are interleaved in the same region.
# In the ChibiOS RP2350 ld script the.data region starts at __data_base__ and the RAMFUNC code follows immediately at __ramfunc_start__, so pure data = ramfunc_start - data_base.
    data_base = symbols.get("__data_base__", 0)
    data_size = (ramfunc_start - data_base) if (data_base and data_base < ramfunc_start) else 0
    # .bss: zero-initialised globals, includes all thread working areas.
    bss_size = _span("__bss_base__", "__bss_end__")

    return MapMetrics(
        ram0_start=ram0_start,
        ram0_size=ram0_size,
        ram0_end=ram0_end,
        heap_base=heap_base,
        heap_end=heap_end,
        heap_size=heap_size,
        min_heap=min_heap,
        sram_allowed=sram_allowed,
        sram_used=sram_used,
        sram_over_limit=sram_over_limit,
        ramfunc_start=ramfunc_start,
        ramfunc_end=ramfunc_end,
        ramfunc_size=ramfunc_size,
        c0_msp_size=c0_msp_size,
        c0_psp_size=c0_psp_size,
        c1_msp_size=c1_msp_size,
        c1_psp_size=c1_psp_size,
        data_size=data_size,
        bss_size=bss_size,
    )


def sanitize_source_line(line: str) -> str:
    # Trim C++ single-line comments and quoted strings to reduce false positives.
    no_comment = line.split("//", 1)[0]
    no_strings = re.sub(r'"[^"\\]*(?:\\.[^"\\]*)*"', '""', no_comment)
    no_strings = re.sub(r"'[^'\\]*(?:\\.[^'\\]*)*'", "''", no_strings)
    return no_strings


def strip_block_comments(text: str) -> str:
    # Remove C/C++ block comments before line scanning to avoid false
    # allocation hits from prose, e.g. "new" in documentation comments.
    return RE_BLOCK_COMMENT.sub("", text)


def should_scan(path: Path) -> bool:
    if path.suffix.lower() not in SOURCE_EXTS:
        return False
    parts = set(path.parts)
    if (
        ".git" in parts
        or "build" in parts
        or "modules" in parts
        or "Tools" in parts
        or "AP_HAL_Linux" in parts
        or "SITL" in parts
        or "AP_HAL_QURT" in parts
        or "AP_HAL_ESP32" in parts
        or "AP_Networking" in parts
        or "AP_Periph" in parts
        or "generator" in parts
        or "_tests" in parts
        or "tests" in parts
        or "examples" in parts
    ):
        return False
    return True


def scan_allocations(source_root: Path) -> AllocationMetrics:
    malloc_calls = 0
    calloc_calls = 0
    realloc_calls = 0
    new_calls = 0
    new_array_calls = 0
    literal_bytes = 0
    files_scanned = 0
    callsites: List[Tuple[str, int, str]] = []

    for path in source_root.rglob("*"):
        if not path.is_file() or not should_scan(path):
            continue
        files_scanned += 1
        try:
            text = path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue

        lines = strip_block_comments(text).splitlines()
        for idx, line in enumerate(lines, start=1):
            code = sanitize_source_line(line)
            stripped = code.strip()
            if not stripped:
                continue
            if stripped.startswith("*") or stripped.startswith("/*") or stripped.startswith("*/"):
                continue

            hit = False
            if RE_MALLOC.search(code):
                malloc_calls += 1
                hit = True
                for m in RE_MALLOC_LITERAL.finditer(code):
                    literal_bytes += parse_int_literal(m.group(1))
            if RE_CALLOC.search(code):
                calloc_calls += 1
                hit = True
                for m in RE_CALLOC_LITERAL.finditer(code):
                    literal_bytes += parse_int_literal(m.group(1)) * parse_int_literal(m.group(2))
            if RE_REALLOC.search(code):
                realloc_calls += 1
                hit = True
            if RE_NEW_ARRAY.search(code):
                new_array_calls += 1
                hit = True
                for m in RE_NEW_ARRAY_LITERAL.finditer(code):
                    literal_bytes += parse_int_literal(m.group(1))
            # Count plain new after new[] to avoid double-counting where possible.
            if RE_NEW.search(code):
                new_calls += 1
                hit = True

            if hit:
                callsites.append((str(path), idx, code.strip()))

    return AllocationMetrics(
        malloc_calls=malloc_calls,
        calloc_calls=calloc_calls,
        realloc_calls=realloc_calls,
        new_calls=new_calls,
        new_array_calls=new_array_calls,
        literal_bytes=literal_bytes,
        files_scanned=files_scanned,
        callsites=callsites,
    )


def parse_hwdef_metrics(hwdef_path: Path) -> HwdefMetrics:
    metrics: Dict[str, Optional[int]] = {k: None for k in HWDEF_INT_KEYS}
    bool_defines = {k: 0 for k in HWDEF_BOOL_DEFINE_KEYS}

    with hwdef_path.open("r", encoding="utf-8", errors="replace") as f:
        for raw in f:
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            tokens = line.split()
            if not tokens:
                continue

            key = tokens[0]
            if key in HWDEF_INT_KEYS and len(tokens) >= 2:
                try:
                    metrics[key] = parse_int_literal(tokens[1])
                except ValueError:
                    pass
                continue

            if key == "define" and len(tokens) >= 3:
                dkey = tokens[1]
                if dkey in HWDEF_BOOL_DEFINE_KEYS:
                    try:
                        bool_defines[dkey] = 1 if parse_int_literal(tokens[2]) else 0
                    except ValueError:
                        bool_defines[dkey] = 0

    return HwdefMetrics(
        hwdef_path=str(hwdef_path),
        main_stack=metrics["MAIN_STACK"],
        flash_size_kb=metrics["FLASH_SIZE_KB"],
        storage_flash_page=metrics["STORAGE_FLASH_PAGE"],
        storage_flash_pages=metrics["STORAGE_FLASH_PAGES"],
        sched_loop_rate=metrics["SCHED_LOOP_RATE"],
        bool_defines=bool_defines,
    )


def parse_runtime_metrics(
    threads_path: Optional[Path],
    tasks_path: Optional[Path],
) -> RuntimeMetrics:
    max_thread_used_pct: Optional[int] = None
    worst_thread_name: Optional[str] = None
    fast_loop_avg_us: Optional[int] = None
    total_overruns: Optional[int] = None
    max_task_tot_pct: Optional[float] = None
    thread_rows: List[ThreadRow] = []
    total_stack_used_bytes: int = 0
    total_stack_allocated_bytes: int = 0

    if threads_path and threads_path.exists():
        with threads_path.open("r", encoding="utf-8", errors="replace") as f:
            for line in f:
                m = RE_THREADS_STACK_ROW.match(line)
                if not m:
                    continue

                name = m.group(1)
                used = int(m.group(2))
                total_raw = m.group(3)

# Newer firmware emits "MSP" as the total for idle threads (which run on the hardware main stack with no ChibiOS working area).
# 4294966900 = 0xFFFFFF74, produced by unsigned pointer subtraction in ChibiOS).
                if total_raw == "MSP":
                    total = 0
                    invalid = True
                else:
                    total = int(total_raw)
                    invalid = total <= 0 or total > 1_000_000
                    if invalid:
                        total = 0  # normalise wrapped/garbage value
                thread_rows.append(ThreadRow(name=name, used=used, total=total, invalid_total=invalid))

                # Accumulate used bytes for every thread; only count valid WA sizes
                # towards the allocated total so MSP-based idle doesn't skew it.
                total_stack_used_bytes += used
                if not invalid:
                    total_stack_allocated_bytes += total

                if invalid:
                    continue

                used_pct = int((used * 100) / total)
                if max_thread_used_pct is None or used_pct > max_thread_used_pct:
                    max_thread_used_pct = used_pct
                    worst_thread_name = name

    if tasks_path and tasks_path.exists():
        fast_sum = 0
        total_ovr = 0
        max_tot = 0.0
        seen = 0

        with tasks_path.open("r", encoding="utf-8", errors="replace") as f:
            for line in f:
                if "AVG=" not in line:
                    continue
                seen += 1
                parts = line.split()
                name = parts[0] if parts else ""

                m_avg = RE_TASKS_AVG.search(line)
                if m_avg:
                    avg = int(m_avg.group(1))
                    if "*" in name:
                        fast_sum += avg

                m_ovr = RE_TASKS_OVR.search(line)
                if m_ovr:
                    total_ovr += int(m_ovr.group(1))

                m_tot = RE_TASKS_TOT.search(line)
                if m_tot:
                    max_tot = max(max_tot, float(m_tot.group(1)))

        if seen > 0:
            fast_loop_avg_us = fast_sum
            total_overruns = total_ovr
            max_task_tot_pct = max_tot

    return RuntimeMetrics(
        threads_file=str(threads_path) if threads_path else None,
        tasks_file=str(tasks_path) if tasks_path else None,
        max_thread_used_pct=max_thread_used_pct,
        worst_thread_name=worst_thread_name,
        fast_loop_avg_us=fast_loop_avg_us,
        total_overruns=total_overruns,
        max_task_tot_pct=max_task_tot_pct,
        thread_rows=thread_rows,
        total_stack_used_bytes=total_stack_used_bytes,
        total_stack_allocated_bytes=total_stack_allocated_bytes,
    )


def recommend_min_heap(
    map_metrics: MapMetrics,
    alloc_metrics: AllocationMetrics,
    hwdef_metrics: Optional[HwdefMetrics],
    runtime_metrics: Optional[RuntimeMetrics],
) -> Dict[str, int]:
    # Base floors: keep at least 12.5% of RAM or 64KB, whichever is larger.
    ram_percent_floor = map_metrics.ram0_size // 8
    absolute_floor = 64 * 1024

    # Allocation pressure: charge per callsite + literal allocations when discoverable.
    total_alloc_calls = (
        alloc_metrics.malloc_calls
        + alloc_metrics.calloc_calls
        + alloc_metrics.realloc_calls
        + alloc_metrics.new_calls
    )
    callsite_pressure = total_alloc_calls * 1024
    literal_pressure = alloc_metrics.literal_bytes * 2

    # Runtime pressure: near-full thread stacks and overloaded fast loop imply
    # less practical free SRAM, so reserve additional minimum heap headroom.
    stack_pressure = 0
    runtime_overrun_pressure = 0

    if runtime_metrics and runtime_metrics.max_thread_used_pct is not None:
        if runtime_metrics.max_thread_used_pct >= 98:
            stack_pressure = 24 * 1024
        elif runtime_metrics.max_thread_used_pct >= 95:
            stack_pressure = 16 * 1024
        elif runtime_metrics.max_thread_used_pct >= 90:
            stack_pressure = 8 * 1024

# Thread WA pressure: if the total bytes committed to working areas is a significant fraction of total SRAM, the heap faces more pressure in practice even when no single thread is near its limit.
    stack_wa_pressure = 0
    if runtime_metrics and runtime_metrics.total_stack_allocated_bytes > 0:
        wa_total = runtime_metrics.total_stack_allocated_bytes
        if wa_total > map_metrics.ram0_size * 0.40:
            stack_wa_pressure = 16 * 1024
        elif wa_total > map_metrics.ram0_size * 0.30:
            stack_wa_pressure = 8 * 1024
        elif wa_total > map_metrics.ram0_size * 0.20:
            stack_wa_pressure = 4 * 1024

    if runtime_metrics and runtime_metrics.fast_loop_avg_us is not None:
        if runtime_metrics.fast_loop_avg_us >= 5000:
            runtime_overrun_pressure = max(runtime_overrun_pressure, 16 * 1024)
        elif runtime_metrics.fast_loop_avg_us >= 4500:
            runtime_overrun_pressure = max(runtime_overrun_pressure, 8 * 1024)

    if runtime_metrics and runtime_metrics.total_overruns is not None:
        if runtime_metrics.total_overruns >= 1000:
            runtime_overrun_pressure = max(runtime_overrun_pressure, 8 * 1024)
        elif runtime_metrics.total_overruns >= 250:
            runtime_overrun_pressure = max(runtime_overrun_pressure, 4 * 1024)

    # Board config pressure: high scheduler rates and non-trivial main stack
    # reservation leave less room for dynamic allocations in practice.
    hwdef_pressure = 0
    if hwdef_metrics and hwdef_metrics.main_stack is not None and hwdef_metrics.main_stack >= 0x4000:
        hwdef_pressure = max(hwdef_pressure, 8 * 1024)
    if hwdef_metrics and hwdef_metrics.sched_loop_rate is not None and hwdef_metrics.sched_loop_rate >= 200:
        hwdef_pressure = max(hwdef_pressure, 4 * 1024)

    recommended = max(
        map_metrics.min_heap,
        ram_percent_floor,
        absolute_floor,
        callsite_pressure,
        literal_pressure,
        stack_pressure,
        stack_wa_pressure,
        runtime_overrun_pressure,
        hwdef_pressure,
    )

    # Keep recommendation sane and inside RAM.
    max_reasonable = map_metrics.ram0_size - (32 * 1024)
    recommended = min(recommended, max_reasonable)

    return {
        "ram_percent_floor": ram_percent_floor,
        "absolute_floor": absolute_floor,
        "callsite_pressure": callsite_pressure,
        "literal_pressure": literal_pressure,
        "stack_pressure": stack_pressure,
        "stack_wa_pressure": stack_wa_pressure,
        "runtime_overrun_pressure": runtime_overrun_pressure,
        "hwdef_pressure": hwdef_pressure,
        "recommended_min_heap": recommended,
        "delta_vs_current": recommended - map_metrics.min_heap,
    }


def human_bytes(value: int) -> str:
    if value >= 1024 * 1024:
        return f"{value / (1024 * 1024):.2f} MiB"
    if value >= 1024:
        return f"{value / 1024:.2f} KiB"
    return f"{value} B"


def detect_default_map(repo_root: Path) -> Optional[Path]:
    # Prefer the common RP2350 targets first, then fall back to newest build map.
    preferred = [
        repo_root / "build" / "Laurel" / "Linker.map",
        repo_root / "build" / "Pico2" / "Linker.map",
    ]
    for candidate in preferred:
        if candidate.exists():
            return candidate

    maps = list((repo_root / "build").glob("*/Linker.map"))
    if not maps:
        return None
    maps.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return maps[0]


def detect_default_hwdef(repo_root: Path, map_path: Path) -> Optional[Path]:
    board = map_path.parent.name
    preferred = repo_root / "libraries" / "AP_HAL_ChibiOS" / "hwdef" / board / "hwdef.dat"
    if preferred.exists():
        return preferred

    fallbacks = [
        repo_root / "libraries" / "AP_HAL_ChibiOS" / "hwdef" / "Laurel" / "hwdef.dat",
        repo_root / "libraries" / "AP_HAL_ChibiOS" / "hwdef" / "Pico2" / "hwdef.dat",
    ]
    for candidate in fallbacks:
        if candidate.exists():
            return candidate
    return None


def detect_default_runtime_file(repo_root: Path, names: List[str]) -> Optional[Path]:
    for name in names:
        candidate = repo_root / name
        if candidate.exists() and candidate.is_file():
            return candidate
    return None


def build_report(
    map_path: Path,
    map_metrics: MapMetrics,
    alloc_metrics: AllocationMetrics,
    recommendation: Dict[str, int],
    hwdef_metrics: Optional[HwdefMetrics],
    runtime_metrics: Optional[RuntimeMetrics],
) -> Dict[str, object]:
    report: Dict[str, object] = {
        "map_file": str(map_path),
        "sram": {
            "ram0_start": map_metrics.ram0_start,
            "ram0_size": map_metrics.ram0_size,
            "ram0_end": map_metrics.ram0_end,
            "sram_used": map_metrics.sram_used,
            "sram_allowed": map_metrics.sram_allowed,
            "sram_over_limit": map_metrics.sram_over_limit,
            "heap_base": map_metrics.heap_base,
            "heap_end": map_metrics.heap_end,
            "heap_size": map_metrics.heap_size,
            "current_min_heap": map_metrics.min_heap,
            "ramfunc_size": map_metrics.ramfunc_size,
            # Per-section sizes from linker map symbols (0 when absent).
            "c0_msp_size": map_metrics.c0_msp_size,
            "c0_psp_size": map_metrics.c0_psp_size,
            "c1_msp_size": map_metrics.c1_msp_size,
            "c1_psp_size": map_metrics.c1_psp_size,
            "data_size": map_metrics.data_size,
            "bss_size": map_metrics.bss_size,
        },
        "allocations": {
            "files_scanned": alloc_metrics.files_scanned,
            "malloc_calls": alloc_metrics.malloc_calls,
            "calloc_calls": alloc_metrics.calloc_calls,
            "realloc_calls": alloc_metrics.realloc_calls,
            "new_calls": alloc_metrics.new_calls,
            "new_array_calls": alloc_metrics.new_array_calls,
            "literal_bytes_detected": alloc_metrics.literal_bytes,
            "callsites": [
                {"file": f, "line": line, "code": code}
                for (f, line, code) in alloc_metrics.callsites
            ],
        },
        "recommendation": recommendation,
    }

    if hwdef_metrics:
        report["hwdef"] = {
            "path": hwdef_metrics.hwdef_path,
            "main_stack": hwdef_metrics.main_stack,
            "flash_size_kb": hwdef_metrics.flash_size_kb,
            "storage_flash_page": hwdef_metrics.storage_flash_page,
            "storage_flash_pages": hwdef_metrics.storage_flash_pages,
            "sched_loop_rate": hwdef_metrics.sched_loop_rate,
            "bool_defines": hwdef_metrics.bool_defines,
        }

    if runtime_metrics:
        report["runtime"] = {
            "threads_file": runtime_metrics.threads_file,
            "tasks_file": runtime_metrics.tasks_file,
            "max_thread_used_pct": runtime_metrics.max_thread_used_pct,
            "worst_thread_name": runtime_metrics.worst_thread_name,
            "fast_loop_avg_us": runtime_metrics.fast_loop_avg_us,
            "total_overruns": runtime_metrics.total_overruns,
            "max_task_tot_pct": runtime_metrics.max_task_tot_pct,
            "total_stack_used_bytes": runtime_metrics.total_stack_used_bytes,
            "total_stack_allocated_bytes": runtime_metrics.total_stack_allocated_bytes,
            "thread_rows": [
                {
                    "name": r.name,
                    "used": r.used,
                    "total": r.total,
                    "used_pct": r.used_pct,
                    "invalid_total": r.invalid_total,
                }
                for r in runtime_metrics.thread_rows
            ],
        }

    return report


def print_human_report(report: Dict[str, object]) -> None:
    sram = report["sram"]
    alloc = report["allocations"]
    rec = report["recommendation"]

    print("RP2350 Heap Budget Report")
    print("========================")
    print(f"map: {report['map_file']}")
    print()
    print("SRAM / Heap")
    print(f"- RAM total:          {human_bytes(sram['ram0_size'])}")
    # Show both bytes and % so the summary line is immediately actionable.
    sram_used_pct = sram['sram_used'] * 100 // sram['ram0_size'] if sram['ram0_size'] else 0
    print(f"- SRAM used:          {human_bytes(sram['sram_used'])}  ({sram_used_pct}% of SRAM)")
    print(f"- SRAM allowed:       {human_bytes(sram['sram_allowed'])}")
    print(f"- SRAM over limit:    {human_bytes(sram['sram_over_limit'])}")
    print(f"- Heap available:     {human_bytes(sram['heap_size'])}")
    print(f"- Current min heap:   {human_bytes(sram['current_min_heap'])}")
    # Show RAMFUNC footprint both as bytes and as a % of total SRAM, so it's
    # obvious how much of the 512 KiB is locked into SRAM-resident code.
    ramfunc_pct = sram['ramfunc_size'] * 100 // sram['ram0_size'] if sram['ram0_size'] else 0
    print(f"- RAMFUNC size:       {human_bytes(sram['ramfunc_size'])}  ({ramfunc_pct}% of SRAM)")
    print()
    print("Allocation scan (source)")
    print(f"- Files scanned:      {alloc['files_scanned']}")
    print(f"- malloc():           {alloc['malloc_calls']}")
    print(f"- calloc():           {alloc['calloc_calls']}")
    print(f"- realloc():          {alloc['realloc_calls']}")
    print(f"- new/new[]:          {alloc['new_calls']} / {alloc['new_array_calls']}")
    print(f"- Literal bytes seen: {human_bytes(alloc['literal_bytes_detected'])}")

    hwdef = report.get("hwdef")
    if hwdef is not None:
        print()
        print("Board config inputs (hwdef.dat)")
        print(f"- File:               {hwdef['path']}")
        print(f"- MAIN_STACK:         {human_bytes(hwdef['main_stack']) if hwdef['main_stack'] is not None else 'n/a'}")
        print(f"- FLASH_SIZE_KB:      {hwdef['flash_size_kb'] if hwdef['flash_size_kb'] is not None else 'n/a'}")
        print(f"- STORAGE page/count: {hwdef['storage_flash_page']} / {hwdef['storage_flash_pages']}")
        print(f"- SCHED_LOOP_RATE:    {hwdef['sched_loop_rate'] if hwdef['sched_loop_rate'] is not None else 'n/a'}")

    runtime = report.get("runtime")
    if runtime is not None:
        print()
        print("Runtime profiling inputs")
        print(f"- Threads file:       {runtime['threads_file'] if runtime['threads_file'] else 'n/a'}")
        print(f"- Tasks file:         {runtime['tasks_file'] if runtime['tasks_file'] else 'n/a'}")

        max_thread_txt = "n/a"
        if runtime["max_thread_used_pct"] is not None:
            worst = runtime["worst_thread_name"] if runtime["worst_thread_name"] else "unknown"
            max_thread_txt = f"{runtime['max_thread_used_pct']}% ({worst})"
        print(f"- Max thread usage:   {max_thread_txt}")

        fast_loop_txt = f"{runtime['fast_loop_avg_us']} us" if runtime["fast_loop_avg_us"] is not None else "n/a"
        overruns_txt = str(runtime["total_overruns"]) if runtime["total_overruns"] is not None else "n/a"
        max_tot_txt = f"{runtime['max_task_tot_pct']}%" if runtime["max_task_tot_pct"] is not None else "n/a"
        print(f"- Fast loop avg sum:  {fast_loop_txt}")
        print(f"- Total overruns:     {overruns_txt}")
        print(f"- Max task TOT:       {max_tot_txt}")

        rows = runtime.get("thread_rows", [])
        if rows:
            print()
            print("Thread stack usage")
            print(f"  {'Thread':<14} {'Used':>6} {'Total':>7}  {'Pct':>4}")
            print(f"  {'-'*14} {'-'*6} {'-'*7}  {'-'*4}")
            for row in rows:
                if row["invalid_total"]:
# The idle thread (and any other MSP-based thread) reports a wrapped/garbage total because ChibiOS computes it from stack-pointer arithmetic on the main stack rather than a working area.
# The value 4294966900 (= 0xFFFFFF74) is effectively -140 as a signed int.
                    print(f"  {row['name']:<14} {row['used']:>6}  {'(no WA)':>7}  {'n/a':>4}  <- MSP-based, total invalid")
                else:
                    pct = row["used_pct"]
                    pct_txt = f"{pct}%" if pct is not None else "?"
                    flag = " ***" if pct is not None and pct >= 90 else ""
                    print(f"  {row['name']:<14} {row['used']:>6} /{row['total']:>6}  {pct_txt:>4}{flag}")

            # Footer: sum of used bytes and total working-area bytes across all threads.
            sum_used = sum(r["used"] for r in rows)
            sum_wa = sum(r["total"] for r in rows if not r["invalid_total"])
            print(f"  {'-'*14} {'-'*6} {'-'*7}  {'-'*4}")
            wa_pct_note = ""
            if sram.get("ram0_size"):
                wa_pct = sum_wa * 100 // sram["ram0_size"]
                wa_pct_note = f"  ({wa_pct}% of SRAM committed to thread WAs)"
            print(f"  {'TOTAL':<14} {human_bytes(sum_used):>6}  {human_bytes(sum_wa):>7}{wa_pct_note}")

# Full breakdown of every major RAM consumer as a fraction of total SRAM, slice by slice from low to high address.
# when absent we fall back to estimates so the block still renders with a clear "estimated" note.
    print()
    print("SRAM breakdown — slice by slice (low → high address)")
    _ram  = sram['ram0_size']
    _used = sram['sram_used']
    _rf   = sram['ramfunc_size']
    _mhp  = sram['current_min_heap']
    _heap = sram['heap_size']

    # Per-section sizes from linker map (0 when the symbols are absent).
    _c0_msp = sram.get('c0_msp_size', 0)
    _c0_psp = sram.get('c0_psp_size', 0)
    _c1_msp = sram.get('c1_msp_size', 0)
    _c1_psp = sram.get('c1_psp_size', 0)
    _data   = sram.get('data_size', 0)
    _bss    = sram.get('bss_size', 0)

    # Thread working-area total from runtime profiling (zero if threads.txt absent).
    _wa = (runtime.get('total_stack_allocated_bytes') or 0) if runtime is not None else 0

    # Determine whether we have the full linker-map slices.
    _have_map_sections = bool(_c0_msp or _c1_msp or _data or _bss)

    def _pct(v: int) -> int:
        """Return v as an integer percentage of total SRAM."""
        return v * 100 // _ram if _ram else 0

    _C = 34  # label column width for alignment
    _DIV = '-' * (_C + 28)

    if _have_map_sections:
# Use real linker-symbol data for a precise slice-by-slice view.
# .bss contains both thread WAs and other global objects
        _bss_other = max(_bss - _wa, 0) if _wa else 0

        if _c0_msp:
            print(f"  {'core0 MSP (IRQ/hard-fault stack):':<{_C}} {human_bytes(_c0_msp):>10}  ({_pct(_c0_msp):2}%)")
        if _c0_psp:
            print(f"  {'core0 PSP (scheduler / idle stack):':<{_C}} {human_bytes(_c0_psp):>10}  ({_pct(_c0_psp):2}%)")
        if _c1_msp:
            print(f"  {'core1 MSP (SMP IRQ stack):':<{_C}} {human_bytes(_c1_msp):>10}  ({_pct(_c1_msp):2}%)")
        if _c1_psp:
            print(f"  {'core1 PSP (SMP scheduler stack):':<{_C}} {human_bytes(_c1_psp):>10}  ({_pct(_c1_psp):2}%)")
        if _data:
            print(f"  {'.data (initialized globals):':<{_C}} {human_bytes(_data):>10}  ({_pct(_data):2}%)")
        if _rf:
            print(f"  {'.ramtext (RAMFUNC — XIP bypass):':<{_C}} {human_bytes(_rf):>10}  ({_pct(_rf):2}%)")
        if _bss:
            if _wa:
                # Split .bss into thread WAs vs other globals.
                print(f"  {'.bss (globals + thread WAs):':<{_C}} {human_bytes(_bss):>10}  ({_pct(_bss):2}%)")
                print(f"  {'  of which thread working areas:':<{_C}} {human_bytes(_wa):>10}  ({_pct(_wa):2}%)  [from threads.txt]")
                print(f"  {'  of which other globals:':<{_C}} {human_bytes(_bss_other):>10}  ({_pct(_bss_other):2}%)")
            else:
                print(f"  {'.bss (globals + thread WAs mixed):':<{_C}} {human_bytes(_bss):>10}  ({_pct(_bss):2}%)  [provide threads.txt to split]")
    else:
        # Linker-map section symbols absent: fall back to estimated values.
        # Main stack from hwdef MAIN_STACK directive (zero if hwdef not found).
        _ms = (hwdef.get('main_stack') or 0) if hwdef is not None else 0
        _dbss = max(_used - _rf - _wa - _ms, 0)
        _ms_note = "" if _ms else "  (n/a — hwdef not found)"
        _wa_note = "  (incl. thread WAs — provide threads.txt to separate)" if not _wa else ""
        print(f"  {'[estimated] data+bss+stacks:':<{_C}} {human_bytes(_dbss):>10}  ({_pct(_dbss):2}%){_wa_note}")
        print(f"  {'[estimated] RAMFUNC (.ramtext):':<{_C}} {human_bytes(_rf):>10}  ({_pct(_rf):2}%)")
        print(f"  {'[estimated] thread working areas:':<{_C}} {human_bytes(_wa):>10}  ({_pct(_wa):2}%)")
        print(f"  {'[estimated] main stack:':<{_C}} {human_bytes(_ms):>10}  ({_pct(_ms):2}%){_ms_note}")

    print(f"  {_DIV}")
    print(f"  {'USED (static total):':<{_C}} {human_bytes(_used):>10}  ({_pct(_used):2}%)")
    print(f"  {'heap min reserved:':<{_C}} {human_bytes(_mhp):>10}  ({_pct(_mhp):2}%)")
    _free = max(_heap - _mhp, 0)
    print(f"  {'heap free above min:':<{_C}} {human_bytes(_free):>10}  ({_pct(_free):2}%)  <- available to assign")
    print(f"  {_DIV}")
    print(f"  {'RAM total:':<{_C}} {human_bytes(_ram):>10}  (100%)")

    print()
    print("Suggested threshold")
    print(f"- ram_percent_floor:  {human_bytes(rec['ram_percent_floor'])}")
    print(f"- absolute_floor:     {human_bytes(rec['absolute_floor'])}")
    print(f"- callsite_pressure:  {human_bytes(rec['callsite_pressure'])}")
    print(f"- literal_pressure:   {human_bytes(rec['literal_pressure'])}")
    print(f"- stack_pressure:     {human_bytes(rec['stack_pressure'])}")
    print(f"- stack_wa_pressure:  {human_bytes(rec['stack_wa_pressure'])}")
    print(f"- runtime_overrun:    {human_bytes(rec['runtime_overrun_pressure'])}")
    print(f"- hwdef_pressure:     {human_bytes(rec['hwdef_pressure'])}")
    print(f"- recommended min:    {human_bytes(rec['recommended_min_heap'])}")

    delta = rec["delta_vs_current"]
    sign = "+" if delta >= 0 else "-"
    print(f"- delta vs current:   {sign}{human_bytes(abs(delta))}")

# Keep callsite scan data for recommendation math, but leave the original console output code commented out.
# print() print("Allocation callsites") callsites = alloc["callsites"] if not callsites: print("- none found") else: for callsite in callsites: print(f"- {callsite['file']}:{callsite['line']}: {callsite['code']}")


def main() -> int:
    repo_root = Path(__file__).resolve().parents[2]

    parser = argparse.ArgumentParser(description="Analyze RP2350 heap budget from map + source allocation callsites")
    parser.add_argument(
        "--map",
        default="",
        help="Path to linker map file (default: auto-detect Laurel/Pico2/latest Linker.map)",
    )
    parser.add_argument(
        "--source-root",
        default="",
        help="Source root for allocation scan (default: repository root)",
    )
    parser.add_argument(
        "--hwdef",
        default="",
        help="Path to hwdef.dat (default: board auto-detect from map target)",
    )
    parser.add_argument(
        "--threads-file",
        default="",
        help="Optional @SYS/threads.txt capture file",
    )
    parser.add_argument(
        "--tasks-file",
        default="",
        help="Optional @SYS/tasks.txt capture file",
    )
    args = parser.parse_args()

    if args.map:
        map_path = Path(args.map).expanduser().resolve()
    else:
        detected = detect_default_map(repo_root)
        if detected is None:
            print("error: no Linker.map found under build/*/Linker.map", file=sys.stderr)
            print("hint: build a RP2350 target first (e.g. ./waf configure --board=Laurel --debug && ./waf copter)", file=sys.stderr)
            return 2
        map_path = detected

    source_root = Path(args.source_root).expanduser().resolve() if args.source_root else repo_root

    if not map_path.exists():
        print(f"error: map file not found: {map_path}", file=sys.stderr)
        return 2
    if not source_root.exists():
        print(f"error: source root not found: {source_root}", file=sys.stderr)
        return 2

    try:
        map_metrics = collect_map_metrics(map_path)
    except KeyError as ex:
        print(f"error: {ex}", file=sys.stderr)
        print("hint: ensure map file is from RP2350 link with new SRAM symbols", file=sys.stderr)
        return 3

    alloc_metrics = scan_allocations(source_root)

    hwdef_metrics: Optional[HwdefMetrics] = None
    if args.hwdef:
        hwdef_path = Path(args.hwdef).expanduser().resolve()
    else:
        detected_hwdef = detect_default_hwdef(repo_root, map_path)
        hwdef_path = detected_hwdef if detected_hwdef else None

    if hwdef_path and hwdef_path.exists():
        try:
            hwdef_metrics = parse_hwdef_metrics(hwdef_path)
        except OSError as ex:
            print(f"warning: failed to parse hwdef file ({hwdef_path}): {ex}", file=sys.stderr)

    if args.threads_file:
        threads_path = Path(args.threads_file).expanduser().resolve()
    else:
        threads_path = detect_default_runtime_file(repo_root, ["threads.new2.txt", "threads.new.txt","threads.current.txt" ])

    if args.tasks_file:
        tasks_path = Path(args.tasks_file).expanduser().resolve()
    else:
        tasks_path = detect_default_runtime_file(repo_root, ["tasks.new2.txt", "tasks.new.txt", "tasks.current.txt"])

    runtime_metrics: Optional[RuntimeMetrics] = None
    if (threads_path and threads_path.exists()) or (tasks_path and tasks_path.exists()):
        runtime_metrics = parse_runtime_metrics(threads_path, tasks_path)

    recommendation = recommend_min_heap(map_metrics, alloc_metrics, hwdef_metrics, runtime_metrics)

    report = build_report(
        map_path=map_path,
        map_metrics=map_metrics,
        alloc_metrics=alloc_metrics,
        recommendation=recommendation,
        hwdef_metrics=hwdef_metrics,
        runtime_metrics=runtime_metrics,
    )

    print_human_report(report)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
