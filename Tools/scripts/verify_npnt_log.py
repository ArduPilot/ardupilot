#!/usr/bin/env python
# encoding: utf-8

"""
sign Digital Sky NPNT Logs
"""
from Crypto.Signature import pkcs1_15
from Crypto.PublicKey import RSA
from Crypto.Hash import SHA256
import base64
import argparse
import json

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Verify Signed DigitalSky logfiles')
    parser.add_argument("logfile", help="signed log file")
    parser.add_argument("-k", "--key", required=True, help='Public Key')

    args = parser.parse_args()

    #import RSA private key
    key = RSA.importKey(open(args.key).read())

    if key.has_private():
        print("Key is private")
        exit(1)

    #open logfile
    logfile = open(args.logfile, "r")
    if not logfile:
        print("Could not open logfile")
        exit(1)

    #read json file
    log = json.load(logfile)

    hash = SHA256.new(json.dumps(log["flightLog"], separators=(',', ':')).encode("utf-8"))
    print("Hash: " + hash.hexdigest())
    pkcs1_15.new(key).verify(hash, base64.b64decode(log["signature"]))
    print("Signature is valid")