#!/usr/bin/env python

"""
script to encode a telemetry value using dshot and gcr and then display in binary and ascii art
see https://github.com/betaflight/betaflight/pull/8554#issuecomment-512507625
"""

import argparse

parser = argparse.ArgumentParser("bdshot_encoder.py")
parser.add_argument('value', help='Value to encode')

args = parser.parse_args()

def dshot_encode(value):
    packet = (value << 1)

    # compute checksum
    csum = 0
    csum_data = packet
    for i in range(3):
        csum ^= csum_data
        csum_data >>= 4

    csum = ~csum
    csum &= 0xF
    packet = (packet << 4) | csum

    return packet

def rll_encode(value):
    old_bit = 0
    rll_value = 0

    for i in range(1, 21):
        if value & 1:
            new_bit = (1 ^ old_bit)
        else:
            new_bit = old_bit
        value >>= 1
        rll_value |= new_bit << i
        old_bit = new_bit
    return rll_value

def gcr_encode(value):
    expo = 0

    if value:
        while value & 1 == 0:
            value >>= 1
            expo = expo + 1
    
    value = (value & 0x1FF) | (expo << 9)
    value = dshot_encode(value)

    nibble_map = [0x19, 0x1b, 0x12, 0x13, 0x1d, 0x15, 0x16, 0x17, 0x1a, 0x09, 0x0a, 0x0b, 0x1e, 0x0d, 0x0e, 0x0f ]

    new_value = 0
    for i in range(4):
        new_value |= (nibble_map[value & 0xF] << ((3-i) * 5))
        value >> 4

    return rll_encode(new_value)

def print_signal(value):
    old_bit = 0
    print("_ ", sep='', end='')
    for i in range(21):
        bit = (value>>(20-i)) & 1
        if bit != old_bit:
            print(' ', sep='', end='')
        if bit:
            print('_', sep='', end='')
        else:
            print(' ', sep='', end='')
        old_bit = bit
    print(" _", sep='', end='')
    print('')

    print(" |", sep='', end='')
    for i in range(21):
        bit = (value>>(20-i)) & 1
        if bit != old_bit:
            print('|', sep='', end='')
        print(' ', sep='', end='')
        old_bit = bit
    print("| ", sep='', end='')
    print('')

    print(" |", sep='', end='')
    for i in range(21):
        bit = (value>>(20-i)) & 1
        if bit != old_bit:
            print('|', sep='', end='')
        if bit == 0:
            print("_", sep='', end='')
        else:
            print(' ', sep='', end='')
        old_bit = bit
    print("| ", sep='', end='')
    print('')

if args.value.startswith("0b"):
    value = int(args.value[2:], 2)
elif args.value.startswith("0x"):
    value = int(args.value[2:], 16)
else:
    value = int(args.value)

encoded_value = gcr_encode(value)
print("0b{:020b}".format(encoded_value))
print_signal(encoded_value)

