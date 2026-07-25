#!/usr/bin/env python3

'''
decode FTP file transfers from tlog
'''


import time
import struct
import sys

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()

from pymavlink import mavutil

class Block(object):
    def __init__(self, offset, size, data):
        self.offset = offset
        self.size = size
        self.data = data

class Transfer(object):
    def __init__(self, filename):
        self.filename = filename
        self.blocks = []

    def extract(self):
        self.blocks.sort(key = lambda x: x.offset)
        data = b""
        for b in self.blocks:
            if b.offset != len(data):
                print("gap at %u" % len(data))
                return None
            data += bytes(b.data)
        return data

def param_decode(data):
    '''decode param packed data'''
    magic = 0x671b
    magic_def = 0x671c
    magic2,num_params,total_params = struct.unpack("<HHH", data[0:6])
    if magic != magic2 and magic_def != magic2:
        print("paramftp: bad magic 0x%x expected 0x%x" % (magic2, magic))
        return
    with_defaults = magic2 == magic_def
    data = data[6:]

    # mapping of data type to type length and format
    data_types = {
        1: (1, 'b'),
        2: (2, 'h'),
        3: (4, 'i'),
        4: (4, 'f'),
    }

    count = 0
    params = []

    pad_byte = 0
    last_name = b""
    ret = []

    while True:
        while len(data) > 0 and data[0] == pad_byte:
            # skip pad bytes
            data = data[1:]

        if len(data) == 0:
            break

        ptype, plen = struct.unpack("<BB", data[0:2])
        flags = (ptype>>4) & 0x0F
        ptype &= 0x0F

        if not ptype in data_types:
            print("paramftp: bad type 0x%x" % ptype)
            return

        (type_len, type_format) = data_types[ptype]

        if flags & 1:
            default_len = type_len
        else:
            default_len = 0

        name_len = ((plen>>4) & 0x0F) + 1
        common_len = (plen & 0x0F)
        name = last_name[0:common_len] + data[2:2+name_len]
        vdata = data[2+name_len:2+name_len+type_len]
        last_name = name
        data = data[2+name_len+type_len+default_len:]
        v, = struct.unpack("<" + type_format, vdata)
        ret.append((name, v, ptype))
        count += 1
    return ret
    


# transfers indexed by session ID
ftp_transfers = {}
ftp_block_size = 0

FTP_OpenFileRO = 4
FTP_ReadFile = 5
FTP_BurstReadFile = 15
FTP_Ack = 128
FTP_NAck = 129

def ftp_add(m):
    session = m.payload[2]
    opcode = m.payload[3]
    size = m.payload[4]
    req_opcode = m.payload[5]
    burst_complete = m.payload[6]
    data = m.payload[12:12+size]
    if opcode == FTP_OpenFileRO:
        filename = data
        ftp_transfers[session] = Transfer(bytes(filename))
    if req_opcode in [FTP_ReadFile, FTP_BurstReadFile] and opcode == FTP_Ack:
        if not session in ftp_transfers:
            print("No session %u" % session)
            return
        offset, = struct.unpack("<I", bytes(m.payload[8:12]))
        ftp_transfers[session].blocks.append(Block(offset,size,data))

def mavparse(logfile):
    '''extract FTP transfers'''
    mlog = mavutil.mavlink_connection(filename)

    while True:
        try:
            m = mlog.recv_match(type=['FILE_TRANSFER_PROTOCOL'])
            if m is None:
                return
        except Exception:
            return
        if m.get_type() == 'FILE_TRANSFER_PROTOCOL':
            ftp_add(m)

for filename in args.logs:
    mavparse(filename)

for session in ftp_transfers:
    f = ftp_transfers[session]
    print('# %s' % f.filename.decode())
    if f.filename.decode().startswith('@PARAM/param.pck'):
        plist = param_decode(f.extract())
        for p in plist:
            print(p[0].decode(), p[1])
