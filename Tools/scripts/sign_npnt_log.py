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

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Sign DigitalSky logfiles')
    parser.add_argument("logfile", help="log file to sign")
    parser.add_argument("-k", "--key", required=True, help='Private Key')

    args = parser.parse_args()

    #import RSA private key
    key = RSA.importKey(open(args.key).read())

    if not key.has_private():
        print("Key is public")
        exit(1)

    #open logfile
    logfile = open(args.logfile, "r")
    if not logfile:
        print("Could not open logfile")
        exit(1)

    #create signed logfile
    signed_logfile = open(args.logfile + ".signed", "w")

    #read the logfile data
    logdata = logfile.read()

    #create SHA256 hash of logdata
    hash = SHA256.new(logdata.encode("utf8"))

    # sign the data with RSA key
    signature = pkcs1_15.new(key).sign(hash)

    # create base64 encoded signature
    b64signature = base64.b64encode(signature)

    # write signature and data to file
    signed_logfile.write("{\"signature\":"+"\"" + b64signature.decode("utf8") + "\",\"flightLog\":" + logdata + "}")

    print("Signed logfile written to " + args.logfile + ".signed")
