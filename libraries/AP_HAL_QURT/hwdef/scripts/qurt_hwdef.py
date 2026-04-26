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


class QURTHWDef(hwdef.HWDef):

    def __init__(self, **kwargs):
        super(QURTHWDef, self).__init__(**kwargs)
        # a list of QURT_SPIDEV devices
        self.qurt_spidev = []

    def write_hwdef_header_content(self, f):
        for d in self.alllines:
            if d.startswith('define '):
                f.write('#define %s\n' % d[7:])

        self.write_SPI_config(f)
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
