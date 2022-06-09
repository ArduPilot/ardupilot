#!/usr/bin/env python
import sys
import json, base64, zlib
from Crypto.Signature import DSS
from Crypto.PublicKey import ECC
from Crypto.Hash import SHA256

#sign the image if key declared
if len(sys.argv) == 3:

    # open apj file
    apj = open(sys.argv[1],'r').read()
    # decode json in apj
    d = json.loads(apj)
    # get image data
    img = zlib.decompress(base64.b64decode(d['image']))
    key = ECC.import_key(open(sys.argv[2], "r").read())
    while len(img) % 4 != 0:
        img += b'\0'
    digest = SHA256.new(img)
    signer = DSS.new(key, 'fips-186-3', encoding='der')
    signature = signer.sign(digest)
    img += len(signature).to_bytes(76 - len(signature), 'big')
    img += signature
    img += b'\x46\x49\x58\x53' # magic
    print(len(signature), len(len(signature).to_bytes(76 - len(signature), 'big')), '...............................')

    d["image"] = base64.b64encode(zlib.compress(img,9)).decode('utf-8')
    d["image_size"] = len(img)
    d["flash_free"] = d["flash_total"] - d["image_size"]
    apj_file = sys.argv[1]
    f = open(apj_file, "w")
    f.write(json.dumps(d, indent=4))
    f.close()
