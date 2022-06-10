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
offset = img.find(descriptor)
if offset == -1:
    print("Failed to find %s ECC_RAW struct" % descriptor)
    sys.exit(1)
offset += 8
print("OFF: ", hex(offset))
from Crypto.PublicKey import ECC
key = ECC.import_key(open(sys.argv[2], 'rb').read())
X = hex(key._point.x).upper()
Y = hex(key._point.y).upper()
desc = struct.pack('<65s65s12s', bytes(X[2:], 'ascii'), bytes(Y[2:], 'ascii'), binascii.unhexlify(sys.argv[3]))
desc_len = 142
img = img[:offset] + desc + img[offset+desc_len:]
print("Applying Public Key QX:%s QY:%s" % (X, Y))
while len(img) % 4 != 0:
    img += b'\x00'

digest = SHA256.new(img)
print("Size hashed: %s hash: %s" % (hex(len(img)), digest.hexdigest()))
print("First word: %s Last word: %s" % (hex(struct.unpack('<I', img[:4])[0]), hex(struct.unpack('<I', img[-4:])[0])))
signer = DSS.new(key, 'fips-186-3', encoding='der')
signature = signer.sign(digest)
img += len(signature).to_bytes(76 - len(signature), 'big')
img += signature

print(len(signature), len(len(signature).to_bytes(76 - len(signature), 'big')))
open(sys.argv[1], 'wb').write(img)
