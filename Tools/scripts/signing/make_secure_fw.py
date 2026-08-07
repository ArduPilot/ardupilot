#!/usr/bin/env python3

# flake8: noqa

'''
sign an ArduPilot APJ firmware with a private key
'''

import sys
import struct
import json, base64, zlib

try:
    import monocypher
except ImportError:
    print("Please install monocypher with: python3 -m pip install pymonocypher==3.1.3.2")
    sys.exit(1)

if monocypher.__version__ != "3.1.3.2":
    print("must use monocypher 3.1.3.2, please run: python3 -m pip install pymonocypher==3.1.3.2")
    sys.exit(1)

from argparse import ArgumentParser
parser = ArgumentParser(description='This program is used to sign the firmware file.')
parser.add_argument('apj_file', metavar='apj-file', type=str, help='Firmware (APJ) to sign')
parser.add_argument('key_file', metavar='key-file', type=str, help='Private key used to sign the firmware')
args = parser.parse_args()

key_len = 32
sig_len = 64
sig_version = 30437
descriptor = b'\x41\xa3\xe5\xf2\x65\x69\x92\x07'

def to_unsigned(i):
    '''convert a possibly signed integer to unsigned'''
    if i < 0:
        i += 2**32
    return i

# open apj file
apj = open(args.apj_file, 'r').read()

# decode json in apj
d = json.loads(apj)

# get image data
img = zlib.decompress(base64.b64decode(d['image']))
img_len = len(img)

def decode_key(ktype, key):
    ktype += "_KEYV1:"
    if not key.startswith(ktype):
        print("Invalid key type")
        sys.exit(1)
    return base64.b64decode(key[len(ktype):])

key = decode_key("PRIVATE", open(args.key_file, 'r').read())
if len(key) != key_len:
    print("Bad key length %u" % len(key))
    sys.exit(1)

offset = img.find(descriptor)
if offset == -1:
    print("No APP_DESCRIPTOR found")
    sys.exit(1)

offset += 8
desc_len = 92

flash1 = img[:offset]
flash2 = img[offset+desc_len:]
flash12 = flash1 + flash2

signature = monocypher.signature_sign(key, flash12)
if len(signature) != sig_len:
    print("Bad signature length %u should be %u" % (len(signature), sig_len))
    sys.exit(1)

# pack signature in 4 bytes length, 8 byte signature version and 64 byte
# signature. We have a signature version to allow for changes to signature
# system in the future
desc = struct.pack("<IQ64s", sig_len+8, sig_version, signature)
img = img[:(offset + 16)] + desc + img[(offset + desc_len):]

if len(img) != img_len:
    print("Error: Image length changed")
    sys.exit(1)

print("Applying signature")

d["image"] = base64.b64encode(zlib.compress(img,9)).decode('utf-8')
d["signed_firmware"] = True

f = open(args.apj_file, "w")
f.write(json.dumps(d, indent=4))
f.close()
print("Wrote %s" % args.apj_file)
