#!/usr/bin/env python3
"""Quick test — prints every UDP packet arriving from X-Plane."""
import socket, struct, sys

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 49005

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('0.0.0.0', PORT))
s.settimeout(3)
print(f'Listening on UDP :{PORT} — press Ctrl+C to stop\n')

count = 0
while True:
    try:
        pkt, addr = s.recvfrom(65536)
    except socket.timeout:
        print(f'  (3 s timeout — nothing received yet, total={count})')
        continue

    count += 1
    tag = pkt[:4]
    print(f'[{addr[0]}:{addr[1]}]  {tag}  len={len(pkt)}  total={count}')

    if tag == b'DATA':
        n = (len(pkt) - 5) // 36
        for i in range(n):
            off  = 5 + i * 36
            code = struct.unpack_from('<I', pkt, off)[0]
            vals = struct.unpack_from('<8f', pkt, off + 4)
            print(f'  row {code:3d}: {[round(v, 3) for v in vals]}')
        print()
