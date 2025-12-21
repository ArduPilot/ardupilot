#!/usr/bin/env python3

import struct
import binascii
import sys
import itertools as it

TAG_TYPES = {
    'splice':       (0x700, 0x400),
    'create':       (0x7ff, 0x401),
    'delete':       (0x7ff, 0x4ff),
    'name':         (0x700, 0x000),
    'reg':          (0x7ff, 0x001),
    'dir':          (0x7ff, 0x002),
    'superblock':   (0x7ff, 0x0ff),
    'struct':       (0x700, 0x200),
    'dirstruct':    (0x7ff, 0x200),
    'ctzstruct':    (0x7ff, 0x202),
    'inlinestruct': (0x7ff, 0x201),
    'userattr':     (0x700, 0x300),
    'tail':         (0x700, 0x600),
    'softtail':     (0x7ff, 0x600),
    'hardtail':     (0x7ff, 0x601),
    'gstate':       (0x700, 0x700),
    'movestate':    (0x7ff, 0x7ff),
    'crc':          (0x700, 0x500),
    'ccrc':         (0x780, 0x500),
    'fcrc':         (0x7ff, 0x5ff),
}

class Tag:
    def __init__(self, *args):
        if len(args) == 1:
            self.tag = args[0]
        elif len(args) == 3:
            if isinstance(args[0], str):
                type = TAG_TYPES[args[0]][1]
            else:
                type = args[0]

            if isinstance(args[1], str):
                id = int(args[1], 0) if args[1] not in 'x.' else 0x3ff
            else:
                id = args[1]

            if isinstance(args[2], str):
                size = int(args[2], str) if args[2] not in 'x.' else 0x3ff
            else:
                size = args[2]

            self.tag = (type << 20) | (id << 10) | size
        else:
            assert False

    @property
    def isvalid(self):
        return not bool(self.tag & 0x80000000)

    @property
    def isattr(self):
        return not bool(self.tag & 0x40000000)

    @property
    def iscompactable(self):
        return bool(self.tag & 0x20000000)

    @property
    def isunique(self):
        return not bool(self.tag & 0x10000000)

    @property
    def type(self):
        return (self.tag & 0x7ff00000) >> 20

    @property
    def type1(self):
        return (self.tag & 0x70000000) >> 20

    @property
    def type3(self):
        return (self.tag & 0x7ff00000) >> 20

    @property
    def id(self):
        return (self.tag & 0x000ffc00) >> 10

    @property
    def size(self):
        return (self.tag & 0x000003ff) >> 0

    @property
    def dsize(self):
        return 4 + (self.size if self.size != 0x3ff else 0)

    @property
    def chunk(self):
        return self.type & 0xff

    @property
    def schunk(self):
        return struct.unpack('b', struct.pack('B', self.chunk))[0]

    def is_(self, type):
        try:
            if ' ' in type:
                type1, type3 = type.split()
                return (self.is_(type1) and
                    (self.type & ~TAG_TYPES[type1][0]) == int(type3, 0))

            return self.type == int(type, 0)

        except (ValueError, KeyError):
            return (self.type & TAG_TYPES[type][0]) == TAG_TYPES[type][1]

    def mkmask(self):
        return Tag(
            0x700 if self.isunique else 0x7ff,
            0x3ff if self.isattr else 0,
            0)

    def chid(self, nid):
        ntag = Tag(self.type, nid, self.size)
        if hasattr(self, 'off'):    ntag.off    = self.off
        if hasattr(self, 'data'):   ntag.data   = self.data
        if hasattr(self, 'ccrc'):   ntag.crc    = self.crc
        if hasattr(self, 'erased'): ntag.erased = self.erased
        return ntag

    def typerepr(self):
        if (self.is_('ccrc')
                and getattr(self, 'ccrc', 0xffffffff) != 0xffffffff):
            crc_status = ' (bad)'
        elif self.is_('fcrc') and getattr(self, 'erased', False):
            crc_status = ' (era)'
        else:
            crc_status = ''

        reverse_types = {v: k for k, v in TAG_TYPES.items()}
        for prefix in range(12):
            mask = 0x7ff & ~((1 << prefix)-1)
            if (mask, self.type & mask) in reverse_types:
                type = reverse_types[mask, self.type & mask]
                if prefix > 0:
                    return '%s %#x%s' % (
                        type, self.type & ((1 << prefix)-1), crc_status)
                else:
                    return '%s%s' % (type, crc_status)
        else:
            return '%02x%s' % (self.type, crc_status)

    def idrepr(self):
        return repr(self.id) if self.id != 0x3ff else '.'

    def sizerepr(self):
        return repr(self.size) if self.size != 0x3ff else 'x'

    def __repr__(self):
        return 'Tag(%r, %d, %d)' % (self.typerepr(), self.id, self.size)

    def __lt__(self, other):
        return (self.id, self.type) < (other.id, other.type)

    def __bool__(self):
        return self.isvalid

    def __int__(self):
        return self.tag

    def __index__(self):
        return self.tag

class MetadataPair:
    def __init__(self, blocks):
        if len(blocks) > 1:
            self.pair = [MetadataPair([block]) for block in blocks]
            self.pair = sorted(self.pair, reverse=True)

            self.data = self.pair[0].data
            self.rev  = self.pair[0].rev
            self.tags = self.pair[0].tags
            self.ids  = self.pair[0].ids
            self.log  = self.pair[0].log
            self.all_ = self.pair[0].all_
            return

        self.pair = [self]
        self.data = blocks[0]
        block = self.data

        self.rev, = struct.unpack('<I', block[0:4])
        crc = binascii.crc32(block[0:4])
        fcrctag = None
        fcrcdata = None

        # parse tags
        corrupt = False
        tag = Tag(0xffffffff)
        off = 4
        self.log = []
        self.all_ = []
        while len(block) - off >= 4:
            ntag, = struct.unpack('>I', block[off:off+4])

            tag = Tag((int(tag) ^ ntag) & 0x7fffffff)
            tag.off = off + 4
            tag.data = block[off+4:off+tag.dsize]
            if tag.is_('ccrc'):
                crc = binascii.crc32(block[off:off+2*4], crc)
            else:
                crc = binascii.crc32(block[off:off+tag.dsize], crc)
            tag.crc = crc
            off += tag.dsize

            self.all_.append(tag)

            if tag.is_('fcrc') and len(tag.data) == 8:
                fcrctag = tag
                fcrcdata = struct.unpack('<II', tag.data)
            elif tag.is_('ccrc'):
                # is valid commit?
                if crc != 0xffffffff:
                    corrupt = True
                if not corrupt:
                    self.log = self.all_.copy()
                    # end of commit?
                    if fcrcdata:
                        fcrcsize, fcrc = fcrcdata
                        fcrc_ = 0xffffffff ^ binascii.crc32(
                            block[off:off+fcrcsize])
                        if fcrc_ == fcrc:
                            fcrctag.erased = True
                            corrupt = True

                # reset tag parsing
                crc = 0
                tag = Tag(int(tag) ^ ((tag.type & 1) << 31))
                fcrctag = None
                fcrcdata = None

        # find active ids
        self.ids = list(it.takewhile(
            lambda id: Tag('name', id, 0) in self,
            it.count()))

        # find most recent tags
        self.tags = []
        for tag in self.log:
            if tag.is_('crc') or tag.is_('splice'):
                continue
            elif tag.id == 0x3ff:
                if tag in self and self[tag] is tag:
                    self.tags.append(tag)
            else:
                # id could have change, I know this is messy and slow
                # but it works
                for id in self.ids:
                    ntag = tag.chid(id)
                    if ntag in self and self[ntag] is tag:
                        self.tags.append(ntag)

        self.tags = sorted(self.tags)

    def __bool__(self):
        return bool(self.log)

    def __lt__(self, other):
        # corrupt blocks don't count
        if not self or not other:
            return bool(other)

        # use sequence arithmetic to avoid overflow
        return not ((other.rev - self.rev) & 0x80000000)

    def __contains__(self, args):
        try:
            self[args]
            return True
        except KeyError:
            return False

    def __getitem__(self, args):
        if isinstance(args, tuple):
            gmask, gtag = args
        else:
            gmask, gtag = args.mkmask(), args

        gdiff = 0
        for tag in reversed(self.log):
            if (gmask.id != 0 and tag.is_('splice') and
                    tag.id <= gtag.id - gdiff):
                if tag.is_('create') and tag.id == gtag.id - gdiff:
                    # creation point
                    break

                gdiff += tag.schunk

            if ((int(gmask) & int(tag)) ==
                    (int(gmask) & int(gtag.chid(gtag.id - gdiff)))):
                if tag.size == 0x3ff:
                    # deleted
                    break

                return tag

        raise KeyError(gmask, gtag)

    def _dump_tags(self, tags, f=sys.stdout, truncate=True):
        f.write("%-8s  %-8s  %-13s %4s %4s" % (
            'off', 'tag', 'type', 'id', 'len'))
        if truncate:
            f.write('  data (truncated)')
        f.write('\n')

        for tag in tags:
            f.write("%08x: %08x  %-14s %3s %4s" % (
                tag.off, tag,
                tag.typerepr(), tag.idrepr(), tag.sizerepr()))
            if truncate:
                f.write("  %-23s  %-8s\n" % (
                    ' '.join('%02x' % c for c in tag.data[:8]),
                    ''.join(c if c >= ' ' and c <= '~' else '.'
                        for c in map(chr, tag.data[:8]))))
            else:
                f.write("\n")
                for i in range(0, len(tag.data), 16):
                    f.write("  %08x: %-47s  %-16s\n" % (
                        tag.off+i,
                        ' '.join('%02x' % c for c in tag.data[i:i+16]),
                        ''.join(c if c >= ' ' and c <= '~' else '.'
                            for c in map(chr, tag.data[i:i+16]))))

    def dump_tags(self, f=sys.stdout, truncate=True):
        self._dump_tags(self.tags, f=f, truncate=truncate)

    def dump_log(self, f=sys.stdout, truncate=True):
        self._dump_tags(self.log, f=f, truncate=truncate)

    def dump_all(self, f=sys.stdout, truncate=True):
        self._dump_tags(self.all_, f=f, truncate=truncate)

def main(args):
    blocks = []
    with open(args.disk, 'rb') as f:
        for block in [args.block1, args.block2]:
            if block is None:
                continue
            f.seek(block * args.block_size)
            blocks.append(f.read(args.block_size)
                .ljust(args.block_size, b'\xff'))

    # find most recent pair
    mdir = MetadataPair(blocks)

    try:
        mdir.tail = mdir[Tag('tail', 0, 0)]
        if mdir.tail.size != 8 or mdir.tail.data == 8*b'\xff':
            mdir.tail = None
    except KeyError:
        mdir.tail = None

    print("mdir {%s} rev %d%s%s%s" % (
        ', '.join('%#x' % b
            for b in [args.block1, args.block2]
            if b is not None),
        mdir.rev,
        ' (was %s)' % ', '.join('%d' % m.rev for m in mdir.pair[1:])
        if len(mdir.pair) > 1 else '',
        ' (corrupted!)' if not mdir else '',
        ' -> {%#x, %#x}' % struct.unpack('<II', mdir.tail.data)
        if mdir.tail else ''))
    if args.all:
        mdir.dump_all(truncate=not args.no_truncate)
    elif args.log:
        mdir.dump_log(truncate=not args.no_truncate)
    else:
        mdir.dump_tags(truncate=not args.no_truncate)

    return 0 if mdir else 1

if __name__ == "__main__":
    import argparse
    import sys
    parser = argparse.ArgumentParser(
        description="Dump useful info about metadata pairs in littlefs.")
    parser.add_argument('disk',
        help="File representing the block device.")
    parser.add_argument('block_size', type=lambda x: int(x, 0),
        help="Size of a block in bytes.")
    parser.add_argument('block1', type=lambda x: int(x, 0),
        help="First block address for finding the metadata pair.")
    parser.add_argument('block2', nargs='?', type=lambda x: int(x, 0),
        help="Second block address for finding the metadata pair.")
    parser.add_argument('-l', '--log', action='store_true',
        help="Show tags in log.")
    parser.add_argument('-a', '--all', action='store_true',
        help="Show all tags in log, included tags in corrupted commits.")
    parser.add_argument('-T', '--no-truncate', action='store_true',
        help="Don't truncate large amounts of data.")
    sys.exit(main(parser.parse_args()))
