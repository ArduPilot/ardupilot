#!/usr/bin/env python3
"""
Parse a Saleae async-serial CSV capture of IBUS2 traffic.

Groups bytes into frames using inter-byte gap timing, then decodes
each frame as IBUS1 (checksum) or IBUS2 (CRC-8 poly 0x25) and
displays the results.

Usage:
    ./parse_capture.py [CSV] [--gap-us GAP] [--max-frames N] [--offset N]
    ./parse_capture.py [CSV] --split-dir DIR   # write one CSV per frame
"""

import argparse
import csv
import os
from dataclasses import dataclass, field
from typing import List, Optional

csv.field_size_limit(10 * 1024 * 1024)


# ---------------------------------------------------------------------------
# CRC-8 (poly 0x25, init 0x00) — mirrors ibus2_crc8() in AP_IBus2.cpp
# ---------------------------------------------------------------------------
def _crc8_dvb(crc: int, byte: int, poly: int) -> int:
    crc ^= byte
    for _ in range(8):
        if crc & 0x80:
            crc = ((crc << 1) ^ poly) & 0xFF
        else:
            crc = (crc << 1) & 0xFF
    return crc


def crc8_ibus2(data: bytes) -> int:
    """CRC-8 with polynomial 0x25, initial value 0xFF."""
    crc = 0xFF
    for b in data:
        crc = _crc8_dvb(crc, b, 0x25)
    return crc


# ---------------------------------------------------------------------------
# CSV parsing
# ---------------------------------------------------------------------------
@dataclass
class Sample:
    time_s: float
    value: int    # 0-255
    raw_row: list = field(default_factory=list)  # original CSV fields


def parse_value(raw: str) -> Optional[int]:
    # Single char (including space = 0x20): return its ordinal directly.
    # Must check BEFORE stripping so that ' ' is not treated as empty.
    if len(raw) == 1:
        return ord(raw)
    stripped = raw.strip()
    if not stripped:
        return None
    if stripped.startswith('\\x') or stripped.startswith(r'\x'):
        return int(stripped[2:], 16)
    # Saleae encodes 0x00 as '\0', 0x0A as '\n', etc. — handle escape sequences.
    if stripped.startswith('\\'):
        try:
            ch = stripped.encode('utf-8').decode('unicode_escape')
            if len(ch) == 1:
                return ord(ch)
        except (UnicodeDecodeError, ValueError):
            pass
    if len(stripped) == 1:
        return ord(stripped)
    try:
        return int(stripped, 16)
    except ValueError:
        return None


def load_csv(path: str) -> List[Sample]:
    samples: List[Sample] = []
    with open(path, newline='') as fh:
        reader = csv.reader(fh)
        next(reader)  # header
        for row in reader:
            if len(row) < 2:
                continue
            v = parse_value(row[1])
            if v is None:
                continue
            try:
                t = float(row[0])
            except ValueError:
                continue
            samples.append(Sample(t, v, row))
    return samples


# ---------------------------------------------------------------------------
# Frame segmentation
# ---------------------------------------------------------------------------
@dataclass
class Frame:
    start_time: float
    gap_before_us: Optional[float]  # µs gap from end of previous frame
    data: bytes
    rows: List[list] = field(default_factory=list)  # original CSV rows


def segment_frames(samples: List[Sample], gap_threshold_us: float) -> List[Frame]:
    """Split samples into frames wherever the inter-byte gap exceeds threshold."""
    if not samples:
        return []

    frames: List[Frame] = []
    current_bytes: List[int] = []
    current_rows: List[list] = []
    current_start = samples[0].time_s
    prev_time = samples[0].time_s
    prev_end_time: Optional[float] = None

    for s in samples:
        gap_us = (s.time_s - prev_time) * 1e6
        if current_bytes and gap_us > gap_threshold_us:
            gap_before = None
            if prev_end_time is not None:
                gap_before = (current_start - prev_end_time) * 1e6
            frames.append(Frame(current_start, gap_before, bytes(current_bytes), current_rows))
            prev_end_time = prev_time
            current_bytes = []
            current_rows = []
            current_start = s.time_s
        current_bytes.append(s.value)
        current_rows.append(s.raw_row)
        prev_time = s.time_s

    if current_bytes:
        gap_before = None
        if prev_end_time is not None:
            gap_before = (current_start - prev_end_time) * 1e6
        frames.append(Frame(current_start, gap_before, bytes(current_bytes), current_rows))

    return frames


# ---------------------------------------------------------------------------
# Split: write one CSV per frame
# ---------------------------------------------------------------------------
HEADER = ['Time [s]', 'Value', 'Parity Error', 'Framing Error']


def split_frames(frames: List[Frame], out_dir: str) -> None:
    os.makedirs(out_dir, exist_ok=True)
    digits = len(str(len(frames)))
    for i, frame in enumerate(frames, start=1):
        name = f"packet{str(i).zfill(digits)}.csv"
        path = os.path.join(out_dir, name)
        with open(path, 'w', newline='') as fh:
            writer = csv.writer(fh)
            writer.writerow(HEADER)
            for row in frame.rows:
                # Pad to 4 fields if needed
                padded = list(row) + [''] * max(0, 4 - len(row))
                writer.writerow(padded[:4])
    print(f"Wrote {len(frames)} files to {out_dir}/")


# ---------------------------------------------------------------------------
# Decoding helpers
# ---------------------------------------------------------------------------
CMD_NAMES = {0: 'RESET', 1: 'GET_TYPE', 2: 'GET_VALUE', 3: 'GET_PARAM', 4: 'SET_PARAM'}


def _crc_result(data: bytes) -> str:
    got = data[-1]
    want = crc8_ibus2(data[:-1])
    return 'OK' if got == want else f'FAIL(got=0x{got:02X} want=0x{want:02X})'


def describe_frame(frame: Frame, idx: int) -> str:
    gap_str = f"{frame.gap_before_us:.1f}µs gap" if frame.gap_before_us is not None else "first frame"
    hex_str = ' '.join(f'{b:02X}' for b in frame.data)
    n = len(frame.data)
    header = f"Packet {idx:4d} @{frame.start_time:.6f}s ({gap_str}) len={n}: {hex_str}"

    if n < 2:
        return f"{header}\n  (too short to decode)"

    b0 = frame.data[0]
    pkt_type = b0 & 0x03
    crc = _crc_result(frame.data)

    if pkt_type == 0:
        subtype   = (b0 >> 2) & 0x0F
        sync_lost = (b0 >> 6) & 0x01
        failsafe  = (b0 >> 7) & 0x01
        addr1 = frame.data[2] & 0x07 if n > 2 else 0
        addr2 = (frame.data[2] >> 3) & 0x07 if n > 2 else 0
        payload = frame.data[3:-1]  # between addr byte and CRC

        lines = [header,
                 f"  Frame1 subtype={subtype} sync_lost={sync_lost} failsafe={failsafe} "
                 f"addr1={addr1} addr2={addr2} crc={crc}",
                 f"  header : {' '.join(f'{b:02X}' for b in frame.data[:3])}",
                 f"  payload: {' '.join(f'{b:02X}' for b in payload)}  ({len(payload)} bytes)",
                 f"  crc    : {frame.data[-1]:02X}",
                 "  payload bits (LSB first per byte):"]

        # bit stream, LSB of each byte first
        bits = []
        for b in payload:
            for i in range(8):
                bits.append((b >> i) & 1)
        lines.append('    ' + ' '.join(str(b) for b in bits))

        # build MSB-first bit stream too
        bits_msb = []
        for b in payload:
            for i in range(7, -1, -1):
                bits_msb.append((b >> i) & 1)

        # try several channel-width decodings, both bit orderings
        for width in (11, 12):
            chans_lsb = []
            for i in range(len(bits) // width):
                val = sum(bits[i * width + j] << j for j in range(width))
                chans_lsb.append(val)
            chans_msb = []
            for i in range(len(bits_msb) // width):
                val = sum(bits_msb[i * width + j] << (width - 1 - j) for j in range(width))
                chans_msb.append(val)
            lines.append(f"  {width}-bit LSB-first: {chans_lsb}")
            lines.append(f"  {width}-bit MSB-first: {chans_msb}")

        # 16-bit little-endian pairs (raw)
        le16 = [payload[i] | (payload[i+1] << 8) for i in range(0, len(payload) - 1, 2)]
        lines.append(f"  16-bit LE pairs: {le16}")

        return '\n'.join(lines)

    elif pkt_type in (1, 2):
        cmd_code  = (b0 >> 2) & 0x3F
        type_name = {1: 'Frame2', 2: 'Frame3'}[pkt_type]
        detail    = f"{type_name} cmd={CMD_NAMES.get(cmd_code, f'CMD({cmd_code})')}"
    else:
        detail = f"unknown pkt_type={pkt_type}"

    return f"{header}\n  {detail} crc={crc}"


# ---------------------------------------------------------------------------
# Gap statistics
# ---------------------------------------------------------------------------
def print_gap_stats(samples: List[Sample]) -> None:
    if len(samples) < 2:
        return
    gaps = [(samples[i].time_s - samples[i - 1].time_s) * 1e6
            for i in range(1, len(samples))]
    gaps.sort()
    n = len(gaps)
    print(f"Inter-sample gap stats (n={n}):")
    for pct in (50, 90, 95, 99, 100):
        idx = min(int(n * pct / 100), n - 1)
        print(f"  p{pct:3d}: {gaps[idx]:.1f} µs")

    # histogram buckets
    buckets = [0, 150, 300, 600, 1200, 2500, 5000, float('inf')]
    counts = [0] * (len(buckets) - 1)
    for g in gaps:
        for i in range(len(counts)):
            if buckets[i] <= g < buckets[i + 1]:
                counts[i] += 1
                break
    print("  histogram:")
    for i, c in enumerate(counts):
        lo = f"{buckets[i]:.0f}"
        hi = "∞" if buckets[i + 1] == float('inf') else f"{buckets[i + 1]:.0f}"
        print(f"    [{lo:>6}, {hi:>6}) µs: {c}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', nargs='?',
                        default='libraries/AP_IBus2/examples/captures/ibus2-FTr8B.csv',
                        help='Saleae async-serial CSV file')
    parser.add_argument('--gap-us', type=float, default=50.0,
                        help='Gap threshold in µs to split frames (default: 50)')
    parser.add_argument('--max-frames', type=int, default=20,
                        help='Maximum number of frames to print (default: 20)')
    parser.add_argument('--offset', type=int, default=0,
                        help='Skip first N frames before printing')
    parser.add_argument('--stats', action='store_true',
                        help='Print inter-sample gap statistics and exit')
    parser.add_argument('--split-dir', metavar='DIR',
                        help='Write one CSV per frame into DIR (packet1.csv, packet2.csv, …)')
    args = parser.parse_args()

    print(f"Loading {args.csv}...")
    samples = load_csv(args.csv)
    print(f"  {len(samples)} samples loaded")

    if args.stats:
        print_gap_stats(samples)
        return

    frames = segment_frames(samples, args.gap_us)
    print(f"  {len(frames)} frames (gap_threshold={args.gap_us}µs)\n")

    if args.split_dir:
        split_frames(frames, args.split_dir)
        return

    shown = 0
    for i, frame in enumerate(frames):
        if i < args.offset:
            continue
        if shown >= args.max_frames:
            print(f"... ({len(frames) - i} more frames, use --max-frames to see more)")
            break
        print(describe_frame(frame, i))
        shown += 1


if __name__ == '__main__':
    main()
