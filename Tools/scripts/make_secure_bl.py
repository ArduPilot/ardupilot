#!/usr/bin/env python

import sys
import struct
import binascii
from Crypto.Signature import DSS
from Crypto.PublicKey import ECC
from Crypto.Hash import SHA256

#0x4ecf4ea5a6b6f729
descriptor = b'\x4e\xcf\x4e\xa5\xa6\xb6\xf7\x29'
img = open(sys.argv[1], 'rb').read()
print(len(img))
offset = img.find(descriptor)
if offset == -1:
    print("Failed to find %s ECC_RAW struct" % descriptor)
    sys.exit(1)
offset += 8
print("OFF: ", hex(offset))
desc = b''
desc_len = 0
if (len(sys.argv) < 3) :
    print("No key file specified")
    sys.exit(1)
if (len(sys.argv) - 2 > 10):
    print("Too many keys specified")
    sys.exit(1)
for i in range (2, len(sys.argv)):
    from Crypto.PublicKey import ECC
    key = ECC.import_key(open(sys.argv[i], 'rb').read())
    X = hex(key._point.x).upper()
    Y = hex(key._point.y).upper()
    print("Applying Public Key %s QX:%s QY:%s" % (sys.argv[i], X, Y))
    desc += struct.pack('<65s65sH', bytes(X[2:], 'ascii'), bytes(Y[2:], 'ascii'), 0)
    desc_len += 132
img = img[:offset] + desc + img[offset+desc_len:]
print(len(img))
open(sys.argv[1], 'wb').write(img)
