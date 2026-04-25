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
import struct
from dataclasses import dataclass, field
from typing import Dict, List, Optional

csv.field_size_limit(10 * 1024 * 1024)


# ---------------------------------------------------------------------------
# CRC-8 (poly 0x25, init 0xFF) — mirrors ibus2_crc8() in AP_IBus2.cpp
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
# SES channel decode — matches SES_UnpackChannels from FTr8B firmware
# ---------------------------------------------------------------------------
# Factor table indexed by the 5-bit ChannelType value.
# Lower 4 bits (ChannelType & 0xF) = NbBits (number of bits per channel in the payload).
# Upper bit (bit 4) doubles the scale, giving higher resolution for standard RC channels.
_SES_FACTORS = [
    0,          0,          0x40000000, 0,          0,          0,          0x028F5C29, 0x0147AE15,
    0x0083126F, 0x00418938, 0x0020C49C, 0x0010624E, 0x00083127, 0x00041894, 0,          0,
    0,          0,          0,          0x20000000, 0x10000000, 0x06666667, 0x03333334, 0,
    0x0147AE15, 0x00A3D70B, 0x00418938, 0x0020C49C, 0x0010624E, 0x00083127, 0,          0,
]


def _ses_max_output(channel_type: int) -> int:
    """Maximum positive SES output for a given channel type (precomputed for µs scaling)."""
    nb_bits = channel_type & 0xF
    if nb_bits < 2:
        return 1
    factor = _SES_FACTORS[channel_type] if channel_type < 32 else 0
    max_mag = (1 << (nb_bits - 1)) - 1
    return max(1, (max_mag * factor + (1 << 15)) >> 16)


def _ses_read_bits(payload: bytes, bit_pos: int, nb_bits: int) -> int:
    """Read nb_bits starting at bit_pos (LSB-first) from payload."""
    byte_idx = bit_pos // 8
    bit_off = bit_pos % 8
    # Read up to 4 bytes as little-endian uint32 (safe: caller checks bounds)
    raw_bytes = (payload[byte_idx:byte_idx + 4] + b'\x00' * 4)[:4]
    chunk = struct.unpack_from('<I', raw_bytes)[0]
    return (chunk >> bit_off) & ((1 << nb_bits) - 1)


def ses_decode_channels(payload: bytes, channel_types: List[int]) -> List[Optional[int]]:
    """
    Decode Frame1 subtype=0 payload using the SES_UnpackChannels algorithm.

    Returns a list of µs values (one per channel in channel_types), or None for
    failsafe-hold entries.  Center = 1500µs, range approximately 988–2012µs.
    """
    results: List[Optional[int]] = []
    bit_pos = 0
    total_bits = len(payload) * 8

    for ct in channel_types:
        nb_bits = ct & 0xF
        if nb_bits < 2:
            results.append(1500)  # no bits consumed; channel at center
            continue

        if bit_pos + nb_bits > total_bits:
            break  # payload exhausted

        raw = _ses_read_bits(payload, bit_pos, nb_bits)
        bit_pos += nb_bits

        sign_bit = 1 << (nb_bits - 1)
        mag_mask = sign_bit - 1

        # Failsafe sentinels
        if raw == sign_bit:
            results.append(None)  # keep-failsafe: caller should retain last value
            continue
        if nb_bits >= 6 and raw == sign_bit + 1:
            results.append(None)  # stop-failsafe
            continue

        # Sign-magnitude decode
        negative = bool(raw & sign_bit)
        if negative:
            raw = (-raw) & mag_mask

        factor = _SES_FACTORS[ct] if ct < 32 else 0
        ses = (raw * factor + (1 << 15)) >> 16
        max_ses = _ses_max_output(ct)
        ses = min(ses, max_ses)

        # Convert to µs: center=1500, ±512µs = 100% deflection at ses=max_ses
        offset = ses * 512 // max_ses
        us = 1500 + (-offset if negative else offset)
        us = max(988, min(2012, us))
        results.append(us)

    return results


def decode_subtype1_channel_types(payload: bytes) -> List[int]:
    """Extract the packed 5-bit channel-type array from a Frame1 subtype=1 payload."""
    bits: List[int] = []
    for b in payload:
        for i in range(8):
            bits.append((b >> i) & 1)

    types: List[int] = []
    i = 0
    while i + 4 < len(bits):
        ct = sum(bits[i + j] << j for j in range(5))
        types.append(ct)
        i += 5
    return types


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


def describe_frame(frame: Frame, idx: int, state: Optional[Dict] = None) -> str:
    """
    Decode and format one frame for display.

    state is a mutable dict shared across calls; it accumulates the channel-type
    key from subtype=1 packets so that subtype=0 packets can be decoded.
    Supported keys: 'channel_types' (list[int]).
    """
    if state is None:
        state = {}

    gap_str = f"{frame.gap_before_us:.1f}µs gap" if frame.gap_before_us is not None else "first frame"
    hex_str = ' '.join(f'{b:02X}' for b in frame.data)
    n = len(frame.data)
    header = f"Packet {idx:4d} @{frame.start_time:.6f}s ({gap_str}) len={n}: {hex_str}"

    if n < 2:
        return f"{header}\n  (too short to decode)"

    b0 = frame.data[0]
    pkt_type = b0 & 0x03
    crc = _crc_result(frame.data)
    crc_ok = (crc == 'OK')

    if pkt_type == 0:
        subtype   = (b0 >> 2) & 0x0F
        sync_lost = (b0 >> 6) & 0x01
        failsafe  = (b0 >> 7) & 0x01
        addr1 = frame.data[2] & 0x07 if n > 2 else 0
        addr2 = (frame.data[2] >> 3) & 0x07 if n > 2 else 0
        payload = frame.data[3:-1]  # between addr byte and CRC

        lines = [header,
                 f"  {pkt_type}(Frame1) subtype={subtype} sync_lost={sync_lost} failsafe={failsafe} "
                 f"addr1={addr1} addr2={addr2} crc={crc}",
                 f"  header : {' '.join(f'{b:02X}' for b in frame.data[:3])}",
                 f"  payload: {' '.join(f'{b:02X}' for b in payload)}  ({len(payload)} bytes)",
                 f"  crc    : {frame.data[-1]:02X}"]

        if subtype == 1 and crc_ok and addr1 == 7:
            # Decompression key: defines how subtype=0 channel data is packed
            types = decode_subtype1_channel_types(payload)
            active = [ct for ct in types if (ct & 0xF) >= 2]
            state['channel_types'] = types

            lines.append(f"  channel types ({len(active)} active):")
            for ch_i, ct in enumerate(types):
                if (ct & 0xF) < 2:
                    break  # trailing zeros = no more channels
                nb = ct & 0xF
                lines.append(f"    ch{ch_i:2d}: type=0x{ct:02X} NbBits={nb}")

        elif subtype == 0 and crc_ok:
            channel_types = state.get('channel_types')
            if channel_types:
                # Only decode channels that actually consume bits (NbBits >= 2)
                active_types = []
                for ct in channel_types:
                    if (ct & 0xF) < 2:
                        break
                    active_types.append(ct)
                if active_types:
                    us_vals = ses_decode_channels(payload, active_types)
                    parts = []
                    for ch_i, us in enumerate(us_vals):
                        if us is None:
                            parts.append(f"ch{ch_i}=FAIL")
                        else:
                            parts.append(f"ch{ch_i}={us}µs")
                    lines.append(f"  channels: {', '.join(parts)}")

        return '\n'.join(lines)

    elif pkt_type == 1:
        cmd_code  = (b0 >> 2) & 0x3F
        type_name = 'Frame2'
        detail    = f"{pkt_type}({type_name}) cmd={cmd_code}({CMD_NAMES.get(cmd_code, f'CMD({cmd_code})')})"
    elif pkt_type == 2:
        cmd_code  = (b0 >> 2) & 0x3F
        type_name = 'Frame3'
        detail    = f"{pkt_type}({type_name}) cmd={cmd_code}({CMD_NAMES.get(cmd_code, f'CMD({cmd_code})')})"
        detail += f" devicetype=0x{frame.data[1]:02x}"
        if len(frame.data) > 2:
            detail += f" valuelength={frame.data[2]}"
        if len(frame.data) > 3:
            b2 = frame.data[3]
            flag_names = {0: "ChannelsType", 1: "Failsafe", 2: "ReceiverInternalSensors"}
            detail += f" b2={b2}"
            for i in range(8):
                if b2 & (1 << i):
                    detail += f" {flag_names.get(i, f'BIT{i}')}"
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

    # Shared mutable state: accumulated channel-type key from subtype=1 packets.
    # Passed through all describe_frame calls so subtype=0 packets can decode channels
    # even if the --offset skips the subtype=1 packet.
    decode_state: Dict = {}

    # Pre-scan all frames to pick up any subtype=1 keys before the display window
    for frame in frames:
        describe_frame(frame, 0, decode_state)

    shown = 0
    for i, frame in enumerate(frames):
        if i < args.offset:
            continue
        if shown >= args.max_frames:
            print(f"... ({len(frames) - i} more frames, use --max-frames to see more)")
            break
        print(describe_frame(frame, i, decode_state))
        shown += 1


if __name__ == '__main__':
    main()
