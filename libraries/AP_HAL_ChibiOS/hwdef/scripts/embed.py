#!/usr/bin/env python

'''
script to create embedded.c from a set of static files

Andrew Tridgell
May 2017
'''

import os

def embed_file(out, f, idx):
    '''embed one file'''
    contents = open(f).read()
    out.write('''
// %s
static const uint8_t ap_romfs_%u[] = {''' % (f, idx))

    for c in contents:
        out.write('%u,' % ord(c))
    out.write('};\n\n');

def create_embedded_h(filename, files):
    '''create a ap_romfs_embedded.h file'''

    this_dir = os.path.realpath(__file__)
    rootdir = os.path.relpath(os.path.join(this_dir, "../../../../.."))

    out = open(filename, "w")
    out.write('''// generated embedded files for AP_ROMFS\n\n''')

    for i in range(len(files)):
        (name, filename) = files[i]
        filename = os.path.join(rootdir, filename)
        embed_file(out, filename, i)

    out.write('''const AP_ROMFS::embedded_file AP_ROMFS::files[] = {\n''')

    for i in range(len(files)):
        (name, filename) = files[i]
        print("Embedding file %s:%s" % (name, filename))
        out.write('{ "%s", sizeof(ap_romfs_%u), ap_romfs_%u },\n' % (name, i, i))
    out.write('};\n')
    out.close()

if __name__ == '__main__':
    import sys
    flist = []
    for i in range(1, len(sys.argv)):
        f = sys.argv[i]
        flist.append((f, f))
    create_embedded_h("/tmp/ap_romfs_embedded.h", flist)
