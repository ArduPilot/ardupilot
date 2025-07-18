#!/usr/bin/env python3

# flake8: noqa

'''
script to create ap_romfs_embedded.h from a set of static files

Andrew Tridgell
May 2017
'''

import os, sys, zlib

def write_encode(out, s):
    out.write(s.encode())

def embed_file(out, f, idx, embedded_name, uncompressed):
    '''embed one file'''
    try:
        contents = open(f,'rb').read()
    except Exception:
        raise Exception("Failed to embed %s" % f)

    if embedded_name.endswith("bootloader.bin"):
        # round size to a multiple of 32 bytes for bootloader, this ensures
        # it can be flashed on a STM32H7 chip
        blen = len(contents)
        pad = (32 - (blen % 32)) % 32
        if pad != 0:
            contents += bytes([0xff]*pad)
            print("Padded %u bytes for %s to %u" % (pad, embedded_name, len(contents)))

    crc = crc32(contents)
    write_encode(out, '__EXTFLASHFUNC__ static const uint8_t ap_romfs_%u[] = {' % idx)

    if uncompressed:
        # terminate if there's not already an existing null. we don't add it to
        # the contents to avoid storing the wrong length
        null_terminate = 0 not in contents
        b = contents
    else:
        # compress it (max level, max window size, raw stream, max mem usage)
        z = zlib.compressobj(level=9, method=zlib.DEFLATED, wbits=-15, memLevel=9)
        b = z.compress(contents)
        b += z.flush()
        # decompressed data will be null terminated at runtime, nothing to do here
        null_terminate = False

    if len(b) == 0:
        raise ValueError(f"Zero-length ROMFS contents ({embedded_name}) not permitted")
    write_encode(out, ",".join(str(c) for c in b))
    if null_terminate:
        write_encode(out, ",0")
    write_encode(out, '};\n\n');
    return crc, len(contents)

def crc32(bytes, crc=0):
    '''crc32 equivalent to crc32_small() from AP_Math/crc.cpp'''
    for byte in bytes:
        crc ^= byte
        for i in range(8):
            mask = (-(crc & 1)) & 0xFFFFFFFF
            crc >>= 1
            crc ^= (0xEDB88320 & mask)
    return crc

def create_embedded_h(filename, files, uncompressed=False):
    '''create a ap_romfs_embedded.h file'''

    done = set()

    out = open(filename, "wb")
    write_encode(out, '''// generated embedded files for AP_ROMFS\n\n''')

    # remove duplicates and sort
    files = sorted(list(set(files)))
    crc = {}
    decompressed_size = {}
    for i in range(len(files)):
        (name, filename) = files[i]
        if name in done:
            print("Duplicate ROMFS file %s" % name)
            sys.exit(1)
        done.add(name)
        try:
            crc[filename], decompressed_size[filename] = embed_file(out, filename, i, name, uncompressed)
        except Exception as e:
            print(e)
            return False

    write_encode(out, '''const AP_ROMFS::embedded_file AP_ROMFS::files[] = {\n''')

    for i in range(len(files)):
        (name, filename) = files[i]
        if uncompressed:
            ustr = ' (uncompressed)'
        else:
            ustr = ''
        print("Embedding file %s:%s%s" % (name, filename, ustr))
        write_encode(out, '{ "%s", sizeof(ap_romfs_%u), %d, 0x%08x, ap_romfs_%u },\n' % (
            name, i, decompressed_size[filename], crc[filename], i))
    write_encode(out, '};\n')
    out.close()
    return True

if __name__ == '__main__':
    import sys
    flist = []
    for i in range(1, len(sys.argv)):
        f = sys.argv[i]
        flist.append((f, f))
    create_embedded_h("/tmp/ap_romfs_embedded.h", flist)
