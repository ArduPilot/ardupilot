#!/usr/bin/env python

import sys
import struct
import binascii

#0x4ecf4ea5a6b6f729
descriptor = b'\x4e\xcf\x4e\xa5\xa6\xb6\xf7\x29'
img = open(sys.argv[1], 'rb').read()
offset = img.find(descriptor)
if offset == -1:
    print("Failed to find %s ECC_RAW struct" % descriptor)
    sys.exit(1)
offset += 8
print("OFF: ", hex(offset))
from Crypto.PublicKey import ECC
pubkey = ECC.import_key(open(sys.argv[2], 'rb').read())
X = hex(pubkey._point.x).upper()
Y = hex(pubkey._point.y).upper()
desc = struct.pack('<65s65s12s', bytes(X[2:], 'ascii'), bytes(Y[2:], 'ascii'), binascii.unhexlify(sys.argv[3]))
desc_len = 142
img = img[:offset] + desc + img[offset+desc_len:]
print("Applying Descriptor QX:%s QY:%s" % (X, Y))
open(sys.argv[1], 'wb').write(img)
