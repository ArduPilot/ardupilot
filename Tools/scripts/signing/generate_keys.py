#!/usr/bin/env python3
'''
generate a public/private key pair using Monocypher
'''

import sys
import base64

try:
    import monocypher
except ImportError:
    print("Please install monocypher with: python3 -m pip install pymonocypher==3.1.3.2")
    sys.exit(1)

if monocypher.__version__ != "3.1.3.2":
    Logs.error("must use monocypher 3.1.3.2, please run: python3 -m pip install pymonocypher==3.1.3.2")
    sys.exit(1)

if len(sys.argv) != 2:
    print("Usage: generate_keys.py BASENAME")
    sys.exit(1)

bname = sys.argv[1]

def encode_key(ktype, key):
    return ktype + "_KEYV1:" + base64.b64encode(key).decode('utf-8')

private_key = monocypher.generate_key()
public_key = monocypher.compute_signing_public_key(private_key)

public_fname = "%s_public_key.dat" % bname
private_fname = "%s_private_key.dat" % bname

open(private_fname, "w").write(encode_key("PRIVATE", private_key))
print("Generated %s" % private_fname)

open(public_fname, "w").write(encode_key("PUBLIC", public_key))
print("Generated %s" % public_fname)
