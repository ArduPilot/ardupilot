# SPDX-License-Identifier: GPL-3.0-or-later
# AP_FLAKE8_CLEAN
"""Generates .skyb fixtures for the show module host tests.

Run from the skybrush-sitl-tester checkout with `uv run python <this file>`.
Writes, next to this script:
  <show>_dNN.skyb          per-drone show files encoded by the tester
  <show>_dNN.player.txt    "t x y z" (mm) sampled with the tester's Python player
  synth_NN.skyb            random trajectories (degree 0/1/3/7) and light programs
  hdr_*.skyb               container edge cases (v1, no CRC, bad CRC)
  rth_NN.skyb              trajectories with a collective RTH plan (block 4)

`python gen_fixtures.py rth` writes only the rth_NN.skyb files.
"""

import asyncio
import random
import struct
import sys
from pathlib import Path

OUT = Path(__file__).resolve().parent
MAX_DRONES = 4


def crc32_ap(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ (0xEDB88320 if crc & 1 else 0)
    return crc


def container(blocks, version=2, crc=True, corrupt=False) -> bytes:
    body = b"".join(struct.pack("<BH", t, len(b)) + b for t, b in blocks)
    if version == 1:
        return b"skyb\x01" + body
    if not crc:
        return b"skyb\x02\x00" + body
    data = bytearray(b"skyb\x02\x01\x00\x00\x00\x00" + body)
    c = crc32_ap(bytes(data))
    if corrupt:
        c ^= 0x10
    data[6:10] = struct.pack("<I", c)
    return bytes(data)


def sample_times(end: float):
    ts = [-1.0, 0.0]
    t = 0.0
    while t < end + 3.0:
        ts.append(round(t, 3))
        t += 0.5
    rnd = random.Random(1)
    ts += [rnd.uniform(0, end + 2) for _ in range(50)]
    return ts


async def export_tester_fixtures():
    from skybrush_sitl_tester.show.skyc import load_skyc

    fixtures = Path(__import__("skybrush_sitl_tester.fixtures", fromlist=["x"]).__file__).parent
    for skyc in sorted(fixtures.glob("*.skyc")):
        drones = load_skyc(skyc)
        step = max(1, len(drones) // MAX_DRONES)
        for drone in drones[::step][:MAX_DRONES]:
            stem = f"{skyc.stem}_d{drone.index:02d}"
            (OUT / f"{stem}.skyb").write_bytes(await drone.encode())
            traj = drone.trajectory
            end = traj.takeoff_time + traj.duration
            ts = sample_times(end)
            pts = drone.sample_positions(ts)
            with open(OUT / f"{stem}.player.txt", "w") as f:
                for t, p in zip(ts, pts):
                    f.write(f"{t:.6f} {p[0]*1000:.3f} {p[1]*1000:.3f} {p[2]*1000:.3f}\n")


def rand_traj(rnd: random.Random) -> bytes:
    scale = rnd.choice([1, 1, 2, 3, 10])
    lim = 30000 // scale
    pos = [rnd.randint(-lim // 4, lim // 4), rnd.randint(-lim // 4, lim // 4), 0]
    out = bytearray([scale]) + struct.pack("<hhhh", *pos, 0)

    def seg(fmts, dur, targets):
        nonlocal out
        hdr = fmts[0] | fmts[1] << 2 | fmts[2] << 4
        out += struct.pack("<BH", hdr, dur)
        for axis in range(3):
            n = (1 << fmts[axis]) - 1
            if n == 0:
                continue
            cur = pos[axis]
            tgt = targets[axis]
            for i in range(1, n + 1):
                if i == n:
                    v = tgt
                else:
                    v = int(cur + (tgt - cur) * i / n + rnd.randint(-lim // 20, lim // 20))
                out += struct.pack("<h", max(-32768, min(32767, v)))
            pos[axis] = tgt

    def clamp(v):
        return max(-lim, min(lim, v))

    if rnd.random() < 0.5:
        seg([0, 0, 0], rnd.choice([0, 1500, 3000]), pos[:])  # ground hold
    climb = rnd.randint(3000, 12000) // scale
    seg([0, 0, rnd.choice([1, 2])], rnd.randint(1000, 6000), [pos[0], pos[1], climb])
    for _ in range(rnd.randint(3, 25)):
        fmts = [rnd.choice([0, 1, 2, 3]) for _ in range(3)]
        tgt = [clamp(pos[0] + rnd.randint(-3000, 3000)), clamp(pos[1] + rnd.randint(-3000, 3000)),
               max(500 // scale, clamp(pos[2] + rnd.randint(-2000, 2000)))]
        dur = rnd.choice([0, 1, 200, 999, 2000, 5000, rnd.randint(1, 20000)])
        seg(fmts, dur, tgt)
    # descent, sometimes with XY jitter around the verticality threshold
    for _ in range(rnd.randint(0, 3)):
        jit = rnd.choice([0, 0, 1, 30 // scale, 60 // scale])
        tgt = [pos[0] + jit, pos[1], max(0, pos[2] - rnd.randint(500, 4000) // scale)]
        seg([1 if jit else 0, 0, rnd.choice([1, 2, 3])], rnd.randint(500, 5000), tgt)
    if rnd.random() < 0.3:
        seg([0, 0, 0], 2000, pos[:])
    return bytes(out)


def varuint(v: int) -> bytes:
    out = bytearray()
    while True:
        b = v & 0x7F
        v >>= 7
        if v:
            out.append(b | 0x80)
        else:
            out.append(b)
            return bytes(out)


def rand_lights(rnd: random.Random) -> bytes:
    out = bytearray()
    depth = 0
    t = 0

    def dur():
        return varuint(rnd.choice([0, 0, 1, 5, 25, 50, 150, 300, rnd.randint(0, 2000)]))

    for _ in range(rnd.randint(3, 40)):
        op = rnd.choice([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 12, 13, 13, 4, 8])
        if op == 2:
            out += bytes([2]) + dur()
        elif op == 3:
            t += rnd.randint(0, 500)
            out += bytes([3]) + varuint(t)
        elif op in (4, 8):
            out += bytes([op, rnd.randrange(256), rnd.randrange(256), rnd.randrange(256)]) + dur()
        elif op in (5, 9):
            out += bytes([op, rnd.randrange(256)]) + dur()
        elif op in (6, 7, 10, 11):
            out += bytes([op]) + dur()
        elif op == 12:
            out += bytes([12, rnd.choice([0, 1, 2, 3, 5])])
            depth += 1
        elif op == 13:
            out += bytes([13])
            depth -= 1
        else:
            out += bytes([op])
    if rnd.random() < 0.1:
        out += bytes([0x42])  # unknown opcode
    if rnd.random() < 0.1:
        out += bytes([2, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F])  # overflowing varint
    if rnd.random() < 0.7:
        out += b"\x00"
    return bytes(out)


def export_synthetic():
    rnd = random.Random(42)
    for i in range(24):
        blocks = [(3, b"comment")] if i % 7 == 3 else []
        blocks += [(1, rand_traj(rnd)), (2, rand_lights(rnd))]
        (OUT / f"synth_{i:02d}.skyb").write_bytes(container(blocks))
    traj = rand_traj(random.Random(7))
    lights = rand_lights(random.Random(8))
    (OUT / "hdr_v1.skyb").write_bytes(container([(1, traj), (2, lights)], version=1))
    (OUT / "hdr_nocrc.skyb").write_bytes(container([(2, lights), (1, traj)], crc=False))
    (OUT / "hdr_badcrc.skyb").write_bytes(container([(1, traj)], corrupt=True))
    empty = bytes([0]) + bytes(8)
    (OUT / "hdr_empty_traj.skyb").write_bytes(container([(1, empty)]))


def rth_plan(rnd: random.Random, corrupt=None) -> bytes:
    """RTH plan block in the libskybrush layout (rth_plan.c), not the older
    skybrush-server encoder layout."""
    scale = rnd.choice([1, 1, 2, 10, 0x81])
    lim = 32767
    points = [(rnd.randint(-lim, lim), rnd.randint(-lim, lim)) for _ in range(rnd.randint(1, 6))]
    entries = []
    for i in range(rnd.randint(1, 12)):
        flags = 0 if i and rnd.random() < 0.2 else rnd.randrange(64)
        e = bytearray([flags]) + varuint(rnd.choice([0, 1, 5, 10, 30, 200]))
        if flags & 1:
            e += struct.pack("<h", rnd.randint(-3000, 8000)) + varuint(rnd.choice([0, 1, 3, 10]))
        if flags & 2:
            e += varuint(rnd.randrange(len(points)))
        if flags & 4:
            e += struct.pack("<h", rnd.randint(-2000, 30000))
        if flags & 6:
            e += varuint(rnd.choice([0, 1, 7, 25, 90]))
        if flags & 8:
            e += struct.pack("<h", rnd.randint(-500, 3000))
        if flags & 16:
            e += varuint(rnd.choice([0, 2, 15]))
        if flags & 32:
            e += varuint(rnd.choice([0, 1, 4]))
        entries.append(bytes(e))
    if corrupt == "bad_point":
        entries.append(bytes([2, 1]) + varuint(len(points) + 3) + varuint(5))
    elif corrupt == "big_duration":
        entries.append(bytes([16, 1]) + varuint(16777217))
    elif corrupt == "no_entries":
        entries = None
    out = bytearray(struct.pack("<BBHHhH", rnd.randrange(256), scale, rnd.choice([0, 0, 500, 2000]),
                                rnd.choice([0, 0, 700, 1500]), rnd.randint(-200, 2000), len(points)))
    for p in points:
        out += struct.pack("<hh", *p)
    if entries is not None:
        out += struct.pack("<H", len(entries)) + b"".join(entries)
    if corrupt == "truncated":
        out = out[:-1]
    return bytes(out)


def export_rth():
    rnd = random.Random(99)
    corrupt = {12: "bad_point", 13: "truncated", 14: "big_duration", 15: "no_entries"}
    for i in range(16):
        blocks = [(1, rand_traj(rnd)), (2, rand_lights(rnd)), (4, rth_plan(rnd, corrupt.get(i)))]
        (OUT / f"rth_{i:02d}.skyb").write_bytes(container(blocks))


if __name__ == "__main__":
    if sys.argv[1:] != ["rth"]:
        asyncio.run(export_tester_fixtures())
        export_synthetic()
    export_rth()
