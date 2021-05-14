#!/usr/bin/env python

def convert(in_file, out_file):
    '''Compile mcm file to binary'''
    with open(in_file) as inp:
        content = inp.readlines()
    inp.close()

    content.pop(0)

    out = open(out_file, 'wb')
    i = -1
    for line in content:
        i = i + 1
        if i % 64 < 54:
           b = int(line, 2)
           out.write(bytearray([b]))
    out.close()

convert('clarity.mcm', 'font0.bin')
convert('clarity_medium.mcm', 'font1.bin')
convert('bfstyle.mcm', 'font2.bin')
convert('bold.mcm', 'font3.bin')
convert('digital.mcm', 'font4.bin')

