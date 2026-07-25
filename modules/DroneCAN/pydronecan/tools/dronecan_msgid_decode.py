#!/usr/bin/env python
'''
decode dronecan message id
'''

import sys
import optparse

parser = optparse.OptionParser("decode_devid.py")
opts, args = parser.parse_args()
if len(args) == 0:
    print("Please supply a Message ID")
    sys.exit(1)

def num(s):
    try:
        return int(s)
    except ValueError:
        return int(s, 16)

msgid=num(args[0])

#define SOURCE_ID_FROM_ID(x)                        ((uint8_t) (((x) >> 0U)  & 0x7FU))
#define SERVICE_NOT_MSG_FROM_ID(x)                  ((bool)    (((x) >> 7U)  & 0x1U))
#define REQUEST_NOT_RESPONSE_FROM_ID(x)             ((bool)    (((x) >> 15U) & 0x1U))
#define DEST_ID_FROM_ID(x)                          ((uint8_t) (((x) >> 8U)  & 0x7FU))
#define PRIORITY_FROM_ID(x)                         ((uint8_t) (((x) >> 24U) & 0x1FU))
#define MSG_TYPE_FROM_ID(x)                         ((uint16_t)(((x) >> 8U)  & 0xFFFFU))
#define SRV_TYPE_FROM_ID(x)                         ((uint8_t) (((x) >> 16U) & 0xFFU))

def SOURCE_ID_FROM_ID(x):
    return (x >> 0) & 0x7F

def SERVICE_NOT_MSG_FROM_ID(x):
    return (x >> 7) & 0x1

def REQUEST_NOT_RESPONSE_FROM_ID(x):
    return (x >> 15) & 0x1

def DEST_ID_FROM_ID(x):
    return (x >> 8) & 0x7F

def PRIORITY_FROM_ID(x):
    return (x >> 24) & 0x1F

def MSG_TYPE_FROM_ID(x):
    return (x >> 8) & 0xFFFF

def SRV_TYPE_FROM_ID(x):
    return (x >> 16) & 0xFF

# pretty print all the details
print("source id: %d" % SOURCE_ID_FROM_ID(msgid))
print("priority: %d" % PRIORITY_FROM_ID(msgid))
if SERVICE_NOT_MSG_FROM_ID(msgid):
    print("dest id: %d" % DEST_ID_FROM_ID(msgid))
    if REQUEST_NOT_RESPONSE_FROM_ID(msgid):
        print("srv(req) type: %d" % SRV_TYPE_FROM_ID(msgid))
    else:
        print("srv(rsp) type: %d" % SRV_TYPE_FROM_ID(msgid))
else:
    print("msg type: %d" % MSG_TYPE_FROM_ID(msgid))
