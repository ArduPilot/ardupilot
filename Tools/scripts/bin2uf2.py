#!/usr/bin/env python3

# flake8: noqa

'''
convert a raw binary to a UF2 image, for loading over the RP2350 BOOTSEL
mass-storage drive.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

import struct
from argparse import ArgumentParser

UF2_MAGIC_START0 = 0x0A324655
UF2_MAGIC_START1 = 0x9E5D5157
UF2_MAGIC_END = 0x0AB16F30
UF2_FLAG_FAMILY_ID = 0x00002000

# the RP2350 Arm Secure image family
RP2350_ARM_S_FAMILY_ID = 0xe48bff59

BLOCK_PAYLOAD = 256
BLOCK_DATA = 476


def bin2uf2(bin_file, uf2_file, address, family_id=RP2350_ARM_S_FAMILY_ID):
    data = open(bin_file, 'rb').read()
    nblocks = (len(data) + BLOCK_PAYLOAD - 1) // BLOCK_PAYLOAD
    with open(uf2_file, 'wb') as f:
        for i in range(nblocks):
            f.write(struct.pack('<8I', UF2_MAGIC_START0, UF2_MAGIC_START1,
                                UF2_FLAG_FAMILY_ID, address + i * BLOCK_PAYLOAD,
                                BLOCK_PAYLOAD, i, nblocks, family_id))
            f.write(data[i * BLOCK_PAYLOAD:(i + 1) * BLOCK_PAYLOAD].ljust(BLOCK_DATA, b'\x00'))
            f.write(struct.pack('<I', UF2_MAGIC_END))
    return nblocks


if __name__ == '__main__':
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--offset", type=lambda x: int(x, 0), default=0,
                        help="flash address the binary loads at")
    parser.add_argument("--family-id", type=lambda x: int(x, 0), default=RP2350_ARM_S_FAMILY_ID,
                        help="UF2 family ID")
    parser.add_argument("infile", type=str, help="input binary")
    parser.add_argument("outfile", type=str, help="output UF2")
    args = parser.parse_args()

    bin2uf2(args.infile, args.outfile, args.offset, args.family_id)
