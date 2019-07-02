#!/usr/bin/env python

'''
script to create ap_romfs_embedded.h from a set of static files

Andrew Tridgell
May 2017
'''

import os, sys, tempfile, gzip

def write_encode(out, s):
    out.write(s.encode())

def embed_file(out, f, idx, embedded_name):
    '''embed one file'''
    try:
        contents = open(f,'rb').read()
    except Exception:
        print("Failed to embed %s" % f)
        return False

    pad = 0
    if embedded_name.endswith("bootloader.bin"):
        # round size to a multiple of 32 bytes for bootloader, this ensures
        # it can be flashed on a STM32H7 chip
        blen = len(contents)
        pad = (32 - (blen % 32)) % 32
        if pad != 0:
            contents += bytes([0xff]*pad)
            print("Padded %u bytes for %s" % (pad, embedded_name))

    write_encode(out, 'static const uint8_t ap_romfs_%u[] = {' % idx)

    # compress it
    compressed = tempfile.NamedTemporaryFile()
    f = open(compressed.name, "wb")
    with gzip.GzipFile(fileobj=f, mode='wb', filename='', compresslevel=9, mtime=0) as g:
        g.write(contents)
    f.close()

    compressed.seek(0)
    b = bytearray(compressed.read())
    compressed.close()
    
    for c in b:
        write_encode(out, '%u,' % c)
    write_encode(out, '};\n\n');
    return True

def create_embedded_h(filename, files):
    '''create a ap_romfs_embedded.h file'''

    out = open(filename, "wb")
    write_encode(out, '''// generated embedded files for AP_ROMFS\n\n''')

    for i in range(len(files)):
        (name, filename) = files[i]
        if not embed_file(out, filename, i, name):
            return False

    write_encode(out, '''const AP_ROMFS::embedded_file AP_ROMFS::files[] = {\n''')

    for i in range(len(files)):
        (name, filename) = files[i]
        print(("Embedding file %s:%s" % (name, filename)).encode())
        write_encode(out, '{ "%s", sizeof(ap_romfs_%u), ap_romfs_%u },\n' % (name, i, i))
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
