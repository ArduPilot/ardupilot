#!/usr/bin/env python3
'''
list all messages sorted by DTID
'''

import sys
import logging

from dronecan import dsdl

parsed = list(dsdl.parse_namespaces(['uavcan','com','ardupilot', 'dronecan']))
if not parsed:
    print("Failed to parse namespaces")
    sys.exit(1)

parsed = [ p for p in parsed if p.default_dtid is not None]
parsed.sort(key=lambda x: x.default_dtid)

KIND_MAP = {
    0: "Service",
    1: "Broadcast",
}

for p in parsed:
    print(p.default_dtid, KIND_MAP[p.kind], p.full_name)
