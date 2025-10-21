#!/usr/bin/env python
import sys

if len(sys.argv) < 3:
   print("Usage: ./mcm2bin.py clarity.mcm  clarity.bin")
   exit()

with open(sys.argv[1]) as inp:
    content = inp.readlines()

content.pop(0)

with open(sys.argv[2], 'wb') as out:
    for i, line in enumerate(content):
        if i % 64 < 54:
           b = int(line, 2)
           out.write(bytearray([b]))
