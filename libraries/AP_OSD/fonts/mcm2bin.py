#!/usr/bin/python
import sys;

if len(sys.argv) < 3:
   print "Usage: ./mcm2bin.py clarity.mcm  clarity.bin"
   exit()

with open(sys.argv[1]) as inp:
    content = inp.readlines()
inp.close();

content.pop(0)

out = open(sys.argv[2], 'wb')
i = -1
for line in content:
    i = i + 1
    if i % 64 < 54:
       b = int(line, 2)
       out.write(bytearray([b]))
out.close()
