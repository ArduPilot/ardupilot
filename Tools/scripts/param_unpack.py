#!/usr/bin/env python3

# flake8: noqa

'''
unpack a param.pck file from @PARAM/param.pck via mavlink FTP
'''

import struct, sys

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("file", metavar="LOG")

args = parser.parse_args()

data = open(args.file,'rb').read()
last_name = ""

magic = 0x671b

# header of 6 bytes
magic2,num_params,total_params = struct.unpack("<HHH", data[0:6])
if magic != magic2:
    print("Bad magic 0x%x expected 0x%x" % (magic2, magic))
    sys.exit(1)

data = data[6:]

# mapping of data type to type length and format
data_types = {
    1: (1, 'b'),
    2: (2, 'h'),
    3: (4, 'i'),
    4: (4, 'f'),
}


# we discard leading pad bytes (zero) on all params
# pad bytes are inserted to ensure that a value doesn't
# cross a block boundary
pad_byte = 0
if sys.version_info.major < 3:
    pad_byte = chr(0)

count = 0

while True:
    # skip pad bytes
    while len(data) > 0 and data[0] == pad_byte:
        data = data[1:]

    if len(data) == 0:
        break

    ptype, plen = struct.unpack("<BB", data[0:2])
    flags = (ptype>>4) & 0x0F
    ptype &= 0x0F

    if ptype not in data_types:
        raise Exception("bad type 0x%x" % ptype)

    (type_len, type_format) = data_types[ptype]

    name_len = ((plen>>4) & 0x0F) + 1
    common_len = (plen & 0x0F)
    name = last_name[0:common_len] + data[2:2+name_len].decode('utf-8')
    vdata = data[2+name_len:2+name_len+type_len]
    last_name = name
    data = data[2+name_len+type_len:]
    v, = struct.unpack("<" + type_format, vdata)
    count += 1
    print("%-16s %f" % (name, float(v)))

if count != num_params or count > total_params:
    print("Error: Got %u params expected %u/%u" % (count, num_params, total_params))
    sys.exit(1)
sys.exit(0)
