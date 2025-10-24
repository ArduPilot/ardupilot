#!/usr/bin/env python3
'''
setup board.h for Linux

AP_FLAKE8_CLEAN

'''

import argparse
import shlex
import sys
import os

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../../libraries/AP_HAL/hwdef/scripts'))
import hwdef  # noqa:E402


class LinuxHWDef(hwdef.HWDef):

    def __init__(self, **kwargs):
        super(LinuxHWDef, self).__init__(**kwargs)
        # a list of LINUX_SPIDEV devices
        self.linux_spidev = []

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
        if a[0] == 'LINUX_SPIDEV':
            self.process_line_linux_spidev(line, depth, a)

        super(LinuxHWDef, self).process_line(line, depth)

    def process_line_undef(self, line, depth, a):
        for u in a[1:]:
            for dev in self.linux_spidev:
                if u == dev[0]:
                    self.linux_spidev.remove(dev)

        super(LinuxHWDef, self).process_line_undef(line, depth, a)

    def process_line_linux_spidev(self, line, depth, a):
        self.linux_spidev.append(a[1:])

    def write_SPI_device_table(self, f):
        '''write SPI device table'''

        devlist = []
        for dev in self.linux_spidev:
            if len(dev) != 8:
                self.error(f"Badly formed LINUX_SPIDEV line {dev} {len(dev)=}")
            (name, bus, subdev, mode, bits_per_word, cs_pin, lowspeed, highspeed) = dev
            # if not bus.startswith('SPI') or bus not in self.spi_list:
            #     self.error(f"Bad SPI {bus=} in SPIDEV line {dev}")
            # if mode not in ['MODE0', 'MODE1', 'MODE2', 'MODE3']:
            #     self.error("Bad MODE in SPIDEV line %s" % dev)
            if not lowspeed.endswith('*MHZ') and not lowspeed.endswith('*KHZ'):
                self.error("Bad lowspeed value %s in SPIDEV line %s" % (lowspeed, dev))
            if not highspeed.endswith('*MHZ') and not highspeed.endswith('*KHZ'):
                self.error("Bad highspeed value %s in SPIDEV line %s" %
                           (highspeed, dev))
            f.write(
                f'#define HAL_SPI_DEVICE{len(devlist)+1} SPIDesc({name:12s}, {bus}, {subdev}, {mode}, {bits_per_word}, {cs_pin}, {lowspeed:6s}, {highspeed:6s})\n'  # noqa
            )
            devlist.append('HAL_SPI_DEVICE%u' % (len(devlist)+1))

        self.write_device_table(f, "spi devices", "HAL_SPI_DEVICE_LIST", devlist)

    def write_SPI_config(self, f):
        '''write SPI config defines'''

        self.write_SPI_device_table(f)


if __name__ == '__main__':

    parser = argparse.ArgumentParser("linux_hwdef.py")
    parser.add_argument(
        '-D', '--outdir', type=str, default="/tmp", help='Output directory')
    parser.add_argument(
        'hwdef', type=str, nargs='+', default=None, help='hardware definition file')
    parser.add_argument(
        '--quiet', action='store_true', default=False, help='quiet running')

    args = parser.parse_args()

    c = LinuxHWDef(
        outdir=args.outdir,
        hwdef=args.hwdef,
        quiet=args.quiet,
    )
    c.run()
