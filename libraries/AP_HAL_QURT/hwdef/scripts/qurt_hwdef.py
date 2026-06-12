#!/usr/bin/env python3
'''
setup board.h for QURT

AP_FLAKE8_CLEAN

'''

import argparse
import os
import shlex
import sys

from dataclasses import dataclass

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../../libraries/AP_HAL/hwdef/scripts'))
import hwdef  # noqa:E402


@dataclass
class QURTSPIDev():
    name : str


@dataclass
class QURTI2CBus():
    logical_idx : int
    physical_id : int
    internal : bool


class QURTHWDef(hwdef.HWDef):

    def __init__(self, **kwargs):
        super(QURTHWDef, self).__init__(**kwargs)
        # a list of QURT_SPIDEV devices
        self.qurt_spidev = []
        # a list of QURTI2CBus entries, ordered by logical index
        self.qurt_i2c_buses = []

    def write_hwdef_header_content(self, f):
        for d in self.alllines:
            if d.startswith('define '):
                f.write('#define %s\n' % d[7:])

        self.write_SPI_config(f)
        self.write_I2C_config(f)
        self.write_IMU_config(f)
        self.write_MAG_config(f)
        self.write_BARO_config(f)

    def process_line(self, line, depth):
        '''process one line of pin definition file'''
        # keep all config lines for later use
        self.all_lines.append(line)
        self.alllines.append(line)

        a = shlex.split(line, posix=False)
        if a[0] == 'QURT_SPIDEV':
            self.process_line_qurt_spidev(line, depth, a)
        elif a[0] == 'I2C_BUS':
            self.process_line_i2c_bus(line, depth, a)

        super(QURTHWDef, self).process_line(line, depth)

    def process_line_undef(self, line, depth, a):
        for u in a[1:]:
            for dev in self.qurt_spidev:
                if u == dev.name:
                    self.qurt_spidev.remove(dev)

        super(QURTHWDef, self).process_line_undef(line, depth, a)

    def process_line_qurt_spidev(self, line, depth, a):
        (cmd, devname) = a
        self.qurt_spidev.append(QURTSPIDev(devname))

    def process_line_i2c_bus(self, line, depth, a):
        if len(a) != 4:
            self.error("I2C_BUS expects: I2C_BUS <logical_idx> <physical_id> <true|false>")
        logical_idx = int(a[1], 0)
        physical_id = int(a[2], 0)
        internal = a[3].lower() in ('true', '1', 'internal')
        self.qurt_i2c_buses.append(QURTI2CBus(logical_idx, physical_id, internal))

    def write_SPI_device_table(self, f):
        '''write SPI device table'''

        devlist = []
        for dev in self.qurt_spidev:
            f.write(
                f'#define HAL_SPI_DEVICE{len(devlist)+1} {dev.name}'
            )
            devlist.append('HAL_SPI_DEVICE%u' % (len(devlist)+1))

        self.write_device_table(f, "spi devices", "HAL_SPI_DEVICE_LIST", devlist)

    def write_SPI_config(self, f):
        '''write SPI config defines'''

        self.write_SPI_device_table(f)

    def write_I2C_config(self, f):
        '''write I2C bus table and internal-bus mask'''
        buses = sorted(self.qurt_i2c_buses, key=lambda b: b.logical_idx)
        for i, b in enumerate(buses):
            if b.logical_idx != i:
                self.error(f"I2C_BUS logical indices must be contiguous starting at 0 (got {b.logical_idx} at position {i})")

        f.write('\n// I2C buses\n')
        if len(buses) == 0:
            f.write('#define HAL_QURT_I2C_BUS_IDS { }\n')
            f.write('#define HAL_QURT_I2C_INTERNAL_MASK 0\n')
            return

        ids = ', '.join(str(b.physical_id) for b in buses)
        mask = 0
        for b in buses:
            if b.internal:
                mask |= (1 << b.logical_idx)
        f.write(f'#define HAL_QURT_I2C_BUS_IDS {{ {ids} }}\n')
        f.write(f'#define HAL_QURT_I2C_INTERNAL_MASK 0x{mask:x}\n')


if __name__ == '__main__':

    parser = argparse.ArgumentParser("qurt_hwdef.py")
    parser.add_argument(
        '-D', '--outdir', type=str, default="/tmp", help='Output directory')
    parser.add_argument(
        'hwdef', type=str, nargs='+', default=None, help='hardware definition file')
    parser.add_argument(
        '--quiet', action='store_true', default=False, help='quiet running')

    args = parser.parse_args()

    c = QURTHWDef(
        outdir=args.outdir,
        hwdef=args.hwdef,
        quiet=args.quiet,
    )
    c.run()
