#!/usr/bin/env python3

import struct
import sys
import json
import io
import itertools as it
from readmdir import Tag, MetadataPair

def main(args):
    superblock = None
    gstate = b'\0\0\0\0\0\0\0\0\0\0\0\0'
    dirs = []
    mdirs = []
    corrupted = []
    cycle = False
    with open(args.disk, 'rb') as f:
        tail = (args.block1, args.block2)
        hard = False
        while True:
            for m in it.chain((m for d in dirs for m in d), mdirs):
                if set(m.blocks) == set(tail):
                    # cycle detected
                    cycle = m.blocks
            if cycle:
                break

            # load mdir
            data = []
            blocks = {}
            for block in tail:
                f.seek(block * args.block_size)
                data.append(f.read(args.block_size)
                    .ljust(args.block_size, b'\xff'))
                blocks[id(data[-1])] = block

            mdir = MetadataPair(data)
            mdir.blocks = tuple(blocks[id(p.data)] for p in mdir.pair)

            # fetch some key metadata as a we scan
            try:
                mdir.tail = mdir[Tag('tail', 0, 0)]
                if mdir.tail.size != 8 or mdir.tail.data == 8*b'\xff':
                    mdir.tail = None
            except KeyError:
                mdir.tail = None

            # have superblock?
            try:
                nsuperblock = mdir[
                    Tag(0x7ff, 0x3ff, 0), Tag('superblock', 0, 0)]
                superblock = nsuperblock, mdir[Tag('inlinestruct', 0, 0)]
            except KeyError:
                pass

            # have gstate?
            try:
                ngstate = mdir[Tag('movestate', 0, 0)]
                gstate = bytes((a or 0) ^ (b or 0)
                    for a,b in it.zip_longest(gstate, ngstate.data))
            except KeyError:
                pass

            # corrupted?
            if not mdir:
                corrupted.append(mdir)

            # add to directories
            mdirs.append(mdir)
            if mdir.tail is None or not mdir.tail.is_('hardtail'):
                dirs.append(mdirs)
                mdirs = []

            if mdir.tail is None:
                break

            tail = struct.unpack('<II', mdir.tail.data)
            hard = mdir.tail.is_('hardtail')

    # find paths
    dirtable = {}
    for dir in dirs:
        dirtable[frozenset(dir[0].blocks)] = dir

    pending = [("/", dirs[0])]
    while pending:
        path, dir = pending.pop(0)
        for mdir in dir:
            for tag in mdir.tags:
                if tag.is_('dir'):
                    try:
                        npath = tag.data.decode('utf8')
                        dirstruct = mdir[Tag('dirstruct', tag.id, 0)]
                        nblocks = struct.unpack('<II', dirstruct.data)
                        nmdir = dirtable[frozenset(nblocks)]
                        pending.append(((path + '/' + npath), nmdir))
                    except KeyError:
                        pass

        dir[0].path = path.replace('//', '/')

    # print littlefs + version info
    version = ('?', '?')
    if superblock:
        version = tuple(reversed(
            struct.unpack('<HH', superblock[1].data[0:4].ljust(4, b'\xff'))))
    print("%-47s%s" % ("littlefs v%s.%s" % version,
        "data (truncated, if it fits)"
        if not any([args.no_truncate, args.log, args.all]) else ""))

    # print gstate
    print("gstate 0x%s" % ''.join('%02x' % c for c in gstate))
    tag = Tag(struct.unpack('<I', gstate[0:4].ljust(4, b'\xff'))[0])
    blocks = struct.unpack('<II', gstate[4:4+8].ljust(8, b'\xff'))
    if tag.size or not tag.isvalid:
        print("  orphans >=%d" % max(tag.size, 1))
    if tag.type:
        print("  move dir {%#x, %#x} id %d" % (
            blocks[0], blocks[1], tag.id))

    # print mdir info
    for i, dir in enumerate(dirs):
        print("dir %s" % (json.dumps(dir[0].path)
            if hasattr(dir[0], 'path') else '(orphan)'))

        for j, mdir in enumerate(dir):
            print("mdir {%#x, %#x} rev %d (was %d)%s%s" % (
                mdir.blocks[0], mdir.blocks[1], mdir.rev, mdir.pair[1].rev,
                ' (corrupted!)' if not mdir else '',
                ' -> {%#x, %#x}' % struct.unpack('<II', mdir.tail.data)
                if mdir.tail else ''))

            f = io.StringIO()
            if args.log:
                mdir.dump_log(f, truncate=not args.no_truncate)
            elif args.all:
                mdir.dump_all(f, truncate=not args.no_truncate)
            else:
                mdir.dump_tags(f, truncate=not args.no_truncate)

            lines = list(filter(None, f.getvalue().split('\n')))
            for k, line in enumerate(lines):
                print("%s %s" % (
                    ' ' if j == len(dir)-1 else
                    'v' if k == len(lines)-1 else
                    '|',
                    line))

    errcode = 0
    for mdir in corrupted:
        errcode = errcode or 1
        print("*** corrupted mdir {%#x, %#x}! ***" % (
            mdir.blocks[0], mdir.blocks[1]))

    if cycle:
        errcode = errcode or 2
        print("*** cycle detected {%#x, %#x}! ***" % (
            cycle[0], cycle[1]))

    return errcode

if __name__ == "__main__":
    import argparse
    import sys
    parser = argparse.ArgumentParser(
        description="Dump semantic info about the metadata tree in littlefs")
    parser.add_argument('disk',
        help="File representing the block device.")
    parser.add_argument('block_size', type=lambda x: int(x, 0),
        help="Size of a block in bytes.")
    parser.add_argument('block1', nargs='?', default=0,
        type=lambda x: int(x, 0),
        help="Optional first block address for finding the superblock.")
    parser.add_argument('block2', nargs='?', default=1,
        type=lambda x: int(x, 0),
        help="Optional second block address for finding the superblock.")
    parser.add_argument('-l', '--log', action='store_true',
        help="Show tags in log.")
    parser.add_argument('-a', '--all', action='store_true',
        help="Show all tags in log, included tags in corrupted commits.")
    parser.add_argument('-T', '--no-truncate', action='store_true',
        help="Show the full contents of files/attrs/tags.")
    sys.exit(main(parser.parse_args()))
