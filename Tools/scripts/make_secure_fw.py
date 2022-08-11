#!/usr/bin/env python
import sys
import json, base64, zlib
from Crypto.Signature import DSS
from Crypto.PublicKey import ECC
from Crypto.Hash import SHA256
import struct
import binascii

def to_unsigned(i):
    '''convert a possibly signed integer to unsigned'''
    if i < 0:
        i += 2**32
    return i

#sign the image if key declared
if len(sys.argv) == 3:
    # open apj file
    apj = open(sys.argv[1],'r').read()
    # decode json in apj
    d = json.loads(apj)
    # get image data
    img = zlib.decompress(base64.b64decode(d['image']))
    key = ECC.import_key(open(sys.argv[2], "r").read())
    descriptor = b'\x41\xa3\xe5\xf2\x65\x69\x92\x07'
    offset = img.find(descriptor)
    if offset == -1:
        print("No APP_DESCRIPTOR found")
        sys.exit(1)
    offset += 8
    desc_len = 92
    digest = SHA256.new(img[:offset] + img[offset+desc_len:])
    signer = DSS.new(key, 'fips-186-3', encoding='der')
    signature = signer.sign(digest)

    siglen = to_unsigned(len(signature))
    signature += bytes(bytearray([0 for i in range(72 - len(signature))]))
    #pack signature in 4 bytes and length into 72 byte array
    desc = struct.pack("<I72s", siglen, signature)
    img = img[:(offset + 16)] + desc + img[(offset + desc_len):]
    print("Applying APP_DESCRIPTOR Signature %d %s" % (siglen, binascii.hexlify(desc)))
    d["image"] = base64.b64encode(zlib.compress(img,9)).decode('utf-8')
    d["image_size"] = len(img)
    d["flash_free"] = d["flash_total"] - d["image_size"]
    f = open(sys.argv[1], "w")
    f.write(json.dumps(d, indent=4))
    f.close()

    # while len(img) % 4 != 0:
    #     img += b'\0'
    # digest = SHA256.new(img)
    # signer = DSS.new(key, 'fips-186-3', encoding='der')
    # signature = signer.sign(digest)
    # img += bytes(bytearray([0xFF for i in range(d["flash_free"] - 2048)])) # 2KB of flash is reserved at the end 
    # img += len(signature).to_bytes(76 - len(signature), 'big')
    # img += signature
    # img += d["image_size"].to_bytes(4, 'little')
    # img += b'\x46\x49\x58\x53' # magic
    # print(len(signature), len(len(signature).to_bytes(76 - len(signature), 'big')))
    # print(binascii.hexlify(bytearray(signature)))
    # d["image"] = base64.b64encode(zlib.compress(img,9)).decode('utf-8')
    # d["image_size"] = len(img)
    # d["flash_free"] = d["flash_total"] - d["image_size"]
    # apj_file = sys.argv[1]
    # f = open(apj_file, "w")
    # f.write(json.dumps(d, indent=4))
    # f.close()
else:
    print("Usage: make_secure_fw.py <apj_file> <key_file>")
    sys.exit(1)