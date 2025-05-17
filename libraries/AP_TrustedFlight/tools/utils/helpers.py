#!/usr/bin/env python3
'''
helper methods for Aerobridge Trusted Flight token/ certificate generation scrips
'''

import base64

def base64url_encode(msg):
    return base64.urlsafe_b64encode(msg).replace(b"=", b"")


def base64url_decode(input):
    if isinstance(input, str):
        input = input.encode("ascii")

    rem = len(input) % 4

    if rem > 0:
        input += b"=" * (4 - rem)

    return base64.urlsafe_b64decode(input)
