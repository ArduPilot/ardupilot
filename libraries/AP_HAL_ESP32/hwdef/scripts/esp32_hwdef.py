#!/usr/bin/env python3
'''
setup board.h for ESP32

AP_FLAKE8_CLEAN

'''

import argparse
import shlex
import re
import sys
import os

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../../libraries/AP_HAL/hwdef/scripts'))
import hwdef  # noqa:E402


class ESP32HWDef(hwdef.HWDef):

    def __init__(self, quiet=False, outdir=None, hwdef=[]):
        super(ESP32HWDef, self).__init__(quiet=quiet, outdir=outdir, hwdef=hwdef)
        # lists of ESP32_SPIDEV buses and devices
        self.esp32_spibus = []
        self.esp32_spidev = []

    def write_hwdef_header_content(self, f):
        for d in self.alllines:
            if d.startswith('define '):
                f.write('#define %s\n' % d[7:])

        self.write_SPI_config(f)
        self.write_IMU_config(f)
        self.write_MAG_config(f)
        self.write_BARO_config(f)

        self.write_env_py(os.path.join(self.outdir, "env.py"))

    def process_line(self, line, depth):
        '''process one line of pin definition file'''
        # keep all config lines for later use
        self.all_lines.append(line)
        self.alllines.append(line)

        a = shlex.split(line, posix=False)
        if a[0] == 'ESP32_SPIBUS':
            self.process_line_esp32_spibus(line, depth, a)
        if a[0] == 'ESP32_SPIDEV':
            self.process_line_esp32_spidev(line, depth, a)

        super(ESP32HWDef, self).process_line(line, depth)

    # SPI_BUS support:
    def process_line_esp32_spibus(self, line, depth, a):
        self.esp32_spibus.append(a[1:])

    def SPI_config_define_line_for_dev(self, define_name, dev, n):
        '''return a #define line for a ESP32_SPIBUS config line'''
        (host, dma_ch, mosi, miso, sclk) = dev
        return "{{ .host={host}, .dma_ch={dma_ch}, .mosi={mosi}, .miso={miso}, .sclk={sclk} }},"

    def write_SPI_bus_table(self, f):
        '''write SPI bus table'''
        if len(self.esp32_spibus) == 0:
            f.write('\n// No SPI buses\n')
            return

        f.write('\n// SPI bus table\n')
        buslist = []
        for bus in self.esp32_spibus:
            if len(bus) != 5:
                self.error(f"Badly formed ESP32_SPIBUS line {bus} {len(bus)=}")
            (host, dma_ch, mosi, miso, sclk) = bus
            buslist.append(f"{{ .host={host}, .dma_ch={dma_ch}, .mosi={mosi}, .miso={miso}, .sclk={sclk} }},")
        f.write('#define HAL_ESP32_SPI_BUSES \\\n')
        f.write(',\\\n'.join([f"   {x}" for x in buslist]))
        f.write("\n")

    def process_line_esp32_spidev(self, line, depth, a):
        self.esp32_spidev.append(a[1:])

    def write_SPI_device_table(self, f):
        '''write SPI device table'''
        if len(self.esp32_spidev) == 0:
            f.write('\n// No SPI devices\n')
            return

        f.write('\n// SPI device table\n')
        devlist = []
        for dev in self.esp32_spidev:
            if len(dev) != 7:
                self.error(f"Badly formed ESP32_SPIDEV line {dev} {len(dev)=}")
            (name, bus, device, cs, mode, lowspeed, highspeed) = dev
            if not re.match(r"^\d+$", bus):
                raise ValueError(f"Bad ESP32_SPIDEV bus ({bus}) (must be digits)")
            if not re.match(r"^\d+$", device):
                raise ValueError(f"Bad ESP32_SPIDEV device ({device}) (must be digits)")
            if not re.match(r"^\w+$", cs):
                raise ValueError(f"Bad ESP32_SPIDEV cs ({cs}) (must be word characters)")
            if not re.match(r"^\d+$", mode):
                raise ValueError(f"Bad ESP32_SPIDEV mode ({mode}) (must be digits)")
            if not lowspeed.endswith('*MHZ') and not lowspeed.endswith('*KHZ'):
                self.error("Bad lowspeed value %s in ESP32_SPIDEV line %s" % (lowspeed, dev))
            if not highspeed.endswith('*MHZ') and not highspeed.endswith('*KHZ'):
                self.error("Bad highspeed value %s in ESP32_SPIDEV line %s" %
                           (highspeed, dev))
            devlist.append(f"{{.name= \"{name}\", .bus={bus}, .device={device}, .cs={cs}, .mode={mode}, .lspeed={lowspeed}, .hspeed={highspeed}}}")  # noqa:E501
        f.write('#define HAL_ESP32_SPI_DEVICES \\\n')
        f.write(',\\\n'.join([f"   {x}" for x in devlist]))
        f.write("\n")

    def write_SPI_config(self, f):
        '''write SPI config defines'''

        if len(self.esp32_spibus):
            self.write_SPI_bus_table(f)

        if len(self.esp32_spidev):
            self.write_SPI_device_table(f)


if __name__ == '__main__':

    parser = argparse.ArgumentParser("esp32_hwdef.py")
    parser.add_argument(
        '-D', '--outdir', type=str, default="/tmp", help='Output directory')
    parser.add_argument(
        'hwdef', type=str, nargs='+', default=None, help='hardware definition file')
    parser.add_argument(
        '--quiet', action='store_true', default=False, help='quiet running')

    args = parser.parse_args()

    c = ESP32HWDef(
        outdir=args.outdir,
        hwdef=args.hwdef,
        quiet=args.quiet,
    )
    c.run()
