#!/usr/bin/env python3

import subprocess as sp

def main(args):
    with open(args.disk, 'rb') as f:
        f.seek(args.block * args.block_size)
        block = (f.read(args.block_size)
            .ljust(args.block_size, b'\xff'))

    # what did you expect?
    print("%-8s  %-s" % ('off', 'data'))
    return sp.run(['xxd', '-g1', '-'], input=block).returncode

if __name__ == "__main__":
    import argparse
    import sys
    parser = argparse.ArgumentParser(
        description="Hex dump a specific block in a disk.")
    parser.add_argument('disk',
        help="File representing the block device.")
    parser.add_argument('block_size', type=lambda x: int(x, 0),
        help="Size of a block in bytes.")
    parser.add_argument('block', type=lambda x: int(x, 0),
        help="Address of block to dump.")
    sys.exit(main(parser.parse_args()))
