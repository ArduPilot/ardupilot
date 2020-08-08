#!/usr/bin/env python
'''
decode an stm32 ICSR register value
'''

import sys
import optparse


def num(s):
    try:
        return int(s)
    except ValueError:
        return int(s, 16)


parser = optparse.OptionParser(__file__)

opts, args = parser.parse_args()

if len(args) == 0:
    print(parser.usage)
    sys.exit(0)

ICSR = num(args[0])

# https://www.st.com/content/ccc/resource/technical/document/programming_manual/6c/3a/cb/e7/e4/ea/44/9b/DM00046982.pdf/files/DM00046982.pdf/jcr:content/translations/en.DM00046982.pdf
# page 225


def decoder_m4_vectactive(value):
    exceptions = {
        0: "Thread mode",
        1: "Reserved",
        2: "NMI",
        3: "Hard fault",
        4: "Memory management fault",
        5: "Bus fault",
        6: "Usage fault",
        7: "Reserved....",
        10: "Reserved",
        11: "SVCall",
        12: "Reserved for Debug",
        13: "Reserved",
        14: "PendSV",
        15: "SysTick",
    }
    if value in exceptions:
        exception = "%s" % str(exceptions[value])
    else:
        exception = "IRQ%u" % (value - 16)

    sys.stdout.write(" (%s)" % exception)


M4_BITS = [
    ("0-8", "VECTACTIVE", decoder_m4_vectactive),
    ("9-10", "RESERVED1", None),
    ("11", "RETOBASE", None),
    ("12-18", "VECTPENDING", None),
    ("19-21", "RESERVED2", None),
    ("22", "ISRPENDING", None),
    ("23-24", "RESERVED3", None),
    ("25", "PENDSTCLR", None),
    ("27", "PENDSVCLR", None),
    ("28", "PENDSVSET", None),
    ("29-30", "RESERVED4", None),
    ("31", "NMIPENDSET", None),
]

for bit in M4_BITS:
    (bits, name, decoder) = bit
    if "-" in bits:
        (start_bit, stop_bit) = bits.split("-")
        start_bit = int(start_bit)
        stop_bit = int(stop_bit)
    else:
        start_bit = int(bits)
        stop_bit = int(bits)
    mask = 0
    for i in range(start_bit, stop_bit+1):
        mask |= (1 << i)
    value = (ICSR & mask) >> start_bit
    sys.stdout.write("%s: %u" % (name, value)),
    if decoder is not None:
        decoder(value)
    print("")
