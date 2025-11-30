#!/usr/bin/env python3

# flake8: noqa

'''
add a set of up to 10 public keys to an ArduPilot bootloader bin file
'''

import sys
import os
import base64

# get command line arguments
from argparse import ArgumentParser
parser = ArgumentParser(description='This program is used to apply signing keys to a signed bootloader.')
parser.add_argument("--omit-ardupilot-keys", action='store_true', default=False, help="omit ArduPilot signing keys")
parser.add_argument("bootloader", type=str, default=None, help="bootloader")
parser.add_argument("keys", nargs='*', type=str, default=[], help="keys")
args = parser.parse_args()
    
descriptor = b'\x4e\xcf\x4e\xa5\xa6\xb6\xf7\x29'
max_keys = 10
key_len = 32

img = open(args.bootloader, 'rb').read()

offset = img.find(descriptor)
if offset == -1:
    print("Failed to find descriptor")
    sys.exit(1)

offset += 8
desc = b''
desc_len = 0

keys = []

if not args.omit_ardupilot_keys:
    print("Adding ArduPilot keys")
    signing_dir = os.path.dirname(os.path.realpath(__file__))
    keydir = os.path.join(signing_dir,"ArduPilotKeys")
    for root, dirs, files in os.walk(keydir):
        for f in files:
            if f.endswith(".dat"):
                keys.append(os.path.relpath(os.path.join(keydir, f)))

keys += args.keys[:]

if len(keys) > max_keys:
    print("Too many key files %u, max is %u" % (len(keys), max_keys))
    sys.exit(1)

if len(keys) <= 0:
    print("At least one key file required")
    sys.exit(1)

def decode_key(ktype, key):
    ktype += "_KEYV1:"
    if not key.startswith(ktype):
        print("Invalid key type")
        sys.exit(1)
    return base64.b64decode(key[len(ktype):])

for kfile in keys:
    key = decode_key("PUBLIC", open(kfile, "r").read())
    print("Applying Public Key %s" % kfile)
    if len(key) != key_len:
        print("Bad key length %u in %s" % (len(key), kfile))
        sys.exit(1)
    desc += key
    desc_len += key_len
img = img[:offset] + desc + img[offset+desc_len:]
open(args.bootloader, 'wb').write(img)
