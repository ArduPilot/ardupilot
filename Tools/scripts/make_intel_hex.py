#!/usr/bin/env python

import sys, os, shutil, struct
import intelhex

# make two intel hex files, one including bootloader and one without
# for loading with DFU based tools

if len(sys.argv) != 4:
    print("Usage: make_intel_hex.py BINFILE BOOTLOADER RESERVE_KB")
    sys.exit(1)

scripts = os.path.dirname(__file__)
binfile = sys.argv[1]
bootloaderfile = sys.argv[2]
reserve_kb = int(sys.argv[3])
(root,ext) = os.path.splitext(binfile)
hexfile = root + ".hex"
hex_with_bl = root + "_with_bl.hex"

if not os.path.exists(binfile):
   print("Can't find bin file %s" % binfile)
   sys.exit(1)

if not os.path.exists(bootloaderfile):
    print("Can't find bootloader file %s" % bootloaderfile)
    sys.exit(1)

blimage = bytes(open(bootloaderfile, "rb").read())
blimage += bytes(struct.pack('B',255) * (reserve_kb * 1024 - len(blimage)))

if reserve_kb > 0 and len(blimage) != reserve_kb * 1024:
   print("Bad blimage size %u (want %u)" % (len(blimage), reserve_kb * 1024))
   sys.exit(1)

appimage = bytes(open(binfile,"rb").read())

with_bl = blimage + appimage

tmpfile = hexfile + ".tmp"

open(tmpfile, "wb").write(appimage)

intelhex.bin2hex(tmpfile, hexfile, offset=(0x08000000 + reserve_kb*1024))

if reserve_kb > 0:
   open(tmpfile, "wb").write(with_bl)
   intelhex.bin2hex(tmpfile, hex_with_bl, offset=0x08000000)
   
os.unlink(tmpfile)

