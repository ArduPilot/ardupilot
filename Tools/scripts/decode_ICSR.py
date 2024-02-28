#!/usr/bin/env python
'''
decode an stm32 ICSR register value

AP_FLAKE8_CLEAN
'''

import sys
import optparse


class DecodeICSR(object):
    # https://www.st.com/content/ccc/resource/technical/document/programming_manual/6c/3a/cb/e7/e4/ea/44/9b/DM00046982.pdf/files/DM00046982.pdf/jcr:content/translations/en.DM00046982.pdf
    # page 225

    def __init__(self):
        # this ICSR-bit-assignment-table table also looks valid for M7
        # - page 195 of
        # dm00237416-stm32f7-series-and-stm32h7-series-cortexm7-processor-programming-manual-stmicroelectronics.pdf
        self.M4_BITS = [
            ("0-8", "VECTACTIVE", self.decoder_m4_vectactive),
            ("9-10", "RESERVED1", None),
            ("11", "RETOBASE", self.decoder_m4_retobase),
            ("12-18", "VECTPENDING", self.decoder_m4_vectpending),
            ("19-21", "RESERVED2", None),
            ("22", "ISRPENDING", self.decoder_m4_isrpending),
            ("23-24", "RESERVED3", None),
            ("25", "PENDSTCLR", self.decoder_m4_pendstclr),
            ("26", "PENDSTSET", self.decoder_m4_pendstset),
            ("27", "PENDSVCLR", self.decoder_m4_pendsvclr),
            ("28", "PENDSVSET", self.decoder_m4_pendsvset),
            ("29-30", "RESERVED4", None),
            ("31", "NMIPENDSET", self.decoder_m4_nmipendset),
        ]

        self.longest_name_length = 0
        for bit in self.M4_BITS:
            (bits, name, decoder) = bit
            length = len(name)
            if length > self.longest_name_length:
                self.longest_name_length = length

    def decoder_m4_vectactive(self, value):
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

        return " (%s)" % exception

    def decoder_m4_retobase(self, value):
        if value:
            out = "no (or no more) active exceptions"
        else:
            out = "preempted active exceptions"
        return (" (%s)" % out)

    def decoder_m4_vectpending(self, value):
        return self.decoder_m4_vectactive(value)

    def decoder_m4_isrpending(self, value):
        if value:
            out = "Interrupt pending"
        else:
            out = "No pending interupt"
        return (" (%s)" % out)

    def decoder_m4_pendstclr(self, value):
        return (" (WO clears SysTick exception)")

    def decoder_m4_pendstset(self, value):
        if value:
            out = "SysTick pending"
        else:
            out = "SysTick not pending"
        return (" (%s)" % out)

    def decoder_m4_pendsvclr(self, value):
        return (" (WO clears pendsv exception)")

    def decoder_m4_pendsvset(self, value):
        if value:
            out = "PendSV pending"
        else:
            out = "PendSV not pending"
        return (" (%s)" % out)

    def decoder_m4_nmipendset(self, value):
        if value:
            out = "NMI pending"
        else:
            out = "NMI not pending"
        return (" (%s)" % out)

    def string(self, ICSR):
        ret = ""

        complete_mask = 0

        for bit in self.M4_BITS:
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
                complete_mask |= mask
            value = (ICSR & mask) >> start_bit
            _format = "%%%us: %%3u " % (self.longest_name_length,)
            ret += (_format % (name, value))
            if decoder is not None:
                ret += decoder(value)
            ret += "\n"
        return ret

        if complete_mask != 0b11111111111111111111111111111111:
            raise Exception("Mask incomplete")

    def run(self, ICSR):
        sys.stdout.write(self.string(ICSR))


if __name__ == '__main__':
    parser = optparse.OptionParser(__file__)

    opts, args = parser.parse_args()

    if len(args) == 0:
        print(parser.usage)
        sys.exit(0)

    def num(s):
        try:
            return int(s)
        except ValueError:
            return int(s, 16)

    ICSR = num(args[0])
    decoder = DecodeICSR()
    decoder.run(ICSR)
