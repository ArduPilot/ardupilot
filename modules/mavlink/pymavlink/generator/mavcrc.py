'''
MAVLink CRC-16/MCRF4XX code

Copyright Andrew Tridgell
Released under GNU LGPL version 3 or later
'''
import sys

try:
    import fastcrc
    mcrf4xx = fastcrc.crc16.mcrf4xx
except Exception:
    mcrf4xx = None

class x25crc_slow(object):
    """CRC-16/MCRF4XX - based on checksum.h from mavlink library"""

    def __init__(self, buf=None):
        self.crc = 0xFFFF
        if buf is not None:
            self.accumulate(buf)

    def accumulate(self, buf):
        """add in some more bytes (it also accepts strings)"""
        if type(buf) is str:
            buf = buf.encode()

        accum = self.crc
        for b in buf:
            tmp = b ^ (accum & 0xFF)
            tmp = (tmp ^ (tmp << 4)) & 0xFF
            accum = (accum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        self.crc = accum

    def accumulate_str(self, buf):
        """
        Provided for backwards compatibility. accumulate now also works on strings.
        """
        return self.accumulate(buf)

class x25crc_fast(object):
    """CRC-16/MCRF4XX - based on checksum.h from mavlink library"""
    def __init__(self, buf=None):
        self.crc = 0xFFFF
        if buf is not None:
            self.accumulate(buf)

    def accumulate(self, buf):
        """add in some more bytes (it also accepts strings)"""
        if type(buf) is str:
            buf = bytes(buf.encode())
        elif type(buf) is list:
            buf = bytes(buf)
        self.crc = mcrf4xx(buf, self.crc)

    def accumulate_str(self, buf):
        """
        Provided for backwards compatibility. accumulate now also works on strings.
        """
        return self.accumulate(buf)

if mcrf4xx is not None:
    x25crc = x25crc_fast
else:
    x25crc = x25crc_slow

if __name__ == "__main__":
    import random, time

    for i in range(100):
        nbytes = random.randint(1000,5000)
        b = random.randbytes(nbytes)

        t1 = time.time()
        slow = x25crc_slow()
        t2 = time.time()
        slow_t = t2 - t1

        t1 = time.time()
        c = x25crc()
        t2 = time.time()
        fast_t = t2 - t1

        slow.accumulate(b)
        c.accumulate(b)
        assert slow.crc == c.crc
        print("Speedup %.2f" % (slow_t/fast_t))
