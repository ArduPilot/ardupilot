#!/usr/bin/env python
'''
Create an apj file from a *.bin binary firmware
'''

import argparse
import json
import base64
import zlib

parser = argparse.ArgumentParser(description='make_apj')

parser.add_argument('bin')
parser.add_argument('apj')
parser.add_argument('--board-id', type=int, default=1, help='board ID')

args = parser.parse_args()

img = open(args.bin, 'rb').read()
d = {
    "board_id": int(args.board_id),
    "magic": "APJFWv1",
    "description": "Firmware for a %s" % args.bin,
    "image": base64.b64encode(zlib.compress(img, 9)).decode('utf-8'),
    "summary": args.bin,
    "version": "0.1",
    "image_size": len(img),
    "board_revision": 0
}

f = open(args.apj, "w")
f.write(json.dumps(d, indent=4))
f.close()
