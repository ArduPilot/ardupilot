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

from dataclasses import dataclass

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../../libraries/AP_HAL/hwdef/scripts'))
import hwdef  # noqa:E402


class ESP32HWDef(hwdef.HWDef):

    def __init__(self, **kwargs):
        super(ESP32HWDef, self).__init__(**kwargs)
        # lists of ESP32_SPIBUS buses and ESP32_SPIDEV devices
        self.esp32_spibus = []
        self.esp32_spidev = []

        # lists of ESP32_I2CBUS buses
        self.esp32_i2cbus = []

        # list of ESP32_SERIAL devices
        self.esp32_serials = []

        # list of ESP32_RCOUT declarations
        self.esp32_rcout = []

        # list of ADC pin configurations
        self.esp32_adcs = []

        # list of SDSPI pin configurations
        self.esp32_sdspi = []

    def write_hwdef_header_content(self, f):
        for d in self.alllines:
            if d.startswith('define '):
                f.write('#define %s\n' % d[7:])

        self.write_SPI_config(f)
        self.write_I2C_config(f)
        self.write_IMU_config(f)
        self.write_MAG_config(f)
        self.write_BARO_config(f)
        self.write_SERIAL_config(f)
        self.write_RCOUT_config(f)
        self.write_ADC_config(f)
        self.write_SDSPI_config(f)

    def process_line(self, line, depth):
        '''process one line of pin definition file'''
        # keep all config lines for later use
        self.all_lines.append(line)
        self.alllines.append(line)

        a = shlex.split(line, posix=False)
        if a[0] == 'ESP32_I2CBUS':
            self.process_line_esp32_i2cbus(line, depth, a)

        if a[0] == 'ESP32_SPIBUS':
            self.process_line_esp32_spibus(line, depth, a)
        if a[0] == 'ESP32_SPIDEV':
            self.process_line_esp32_spidev(line, depth, a)

        if a[0] == 'ESP32_SERIAL':
            self.process_line_esp32_serial(line, depth, a)

        if a[0] == 'ESP32_RCOUT':
            self.process_line_esp32_rcout(line, depth, a)

        if a[0] == 'ESP32_ADC_PIN':
            self.process_line_esp32_adc(line, depth, a)

        if a[0] == 'ESP32_SDSPI':
            self.process_line_esp32_sdspi(line, depth, a)

        super(ESP32HWDef, self).process_line(line, depth)

    # ESP32_I2CBUS support:
    def process_line_esp32_i2cbus(self, line, depth, a):
        self.esp32_i2cbus.append(a[1:])

    def write_I2C_bus_table(self, f):
        '''write I2C bus table'''
        buslist = []
        for bus in self.esp32_i2cbus:
            if len(bus) != 6:
                self.error(f"Badly formed ESP32_I2CBUS line {bus} {len(bus)=} want=6")
            (port, sda, scl, speed, internal, soft) = bus
            buslist.append(f"{{ .port={port}, .sda={sda}, .scl={scl}, .speed={speed}, .internal={internal}, .soft={soft} }}")

        self.write_device_table(f, "i2c buses", "HAL_ESP32_I2C_BUSES", buslist)

    # ESP32_SPI_BUS support:
    def process_line_esp32_spibus(self, line, depth, a):
        self.esp32_spibus.append(a[1:])

    def SPI_config_define_line_for_dev(self, define_name, dev, n):
        '''return a #define line for a ESP32_SPIBUS config line'''
        (host, dma_ch, mosi, miso, sclk) = dev
        return "{{ .host={host}, .dma_ch={dma_ch}, .mosi={mosi}, .miso={miso}, .sclk={sclk} }},"

    def write_SPI_bus_table(self, f):
        '''write SPI bus table'''
        buslist = []
        for bus in self.esp32_spibus:
            if len(bus) != 5:
                self.error(f"Badly formed ESP32_SPIBUS line {bus} {len(bus)=}")
            (host, dma_ch, mosi, miso, sclk) = bus
            buslist.append(f"{{ .host={host}, .dma_ch={dma_ch}, .mosi={mosi}, .miso={miso}, .sclk={sclk} }},")

        self.write_device_table(f, "SPI buses", "HAL_ESP32_SPI_BUSES", buslist)

    def process_line_esp32_spidev(self, line, depth, a):
        self.esp32_spidev.append(a[1:])

    def process_line_esp32_serial(self, line, depth, a):
        self.esp32_serials.append(a[1:])

    # ESP32_RCOUT support:
    def process_line_esp32_rcout(self, line, depth, a):
        self.esp32_rcout.append(a[1:])

    @dataclass
    class SDSPI():
        host : str
        dma_ch : int
        mosi : str
        miso : str
        sclk : str
        cs : str

    # ESP32_SDSPI support:
    def process_line_esp32_sdspi(self, line, depth, a):
        (host, dma_ch, mosi, miso, sclk, cs) = a[1:]
        self.esp32_sdspi.append(self.SDSPI(host, dma_ch, mosi, miso, sclk, cs))

    @dataclass
    class ADCPin():
        pin : int
        gain : float
        ardupilotpin : int

    # ESP32_ADC support:
    def process_line_esp32_adc(self, line, depth, a):
        (pin, gain, ardupilotpin) = a[1:]
        self.esp32_adcs.append(self.ADCPin(pin, gain, ardupilotpin))

    def write_SPI_device_table(self, f):
        '''write SPI device table'''
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

        self.write_device_table(f, 'SPI devices', 'HAL_ESP32_SPI_DEVICES', devlist)

    def write_I2C_config(self, f):
        '''write I2C config defines'''

        self.write_I2C_bus_table(f)

    def write_SPI_config(self, f):
        '''write SPI config defines'''

        self.write_SPI_bus_table(f)

        self.write_SPI_device_table(f)

    def write_SERIAL_config(self, f):
        '''write serial config defines'''

        seriallist = []
        for serial in self.esp32_serials:
            if len(serial) != 3:
                self.error(f"Badly formed ESP32_SERIALS line {serial} {len(serial)=}")
            (port, rxpin, txpin) = serial
            seriallist.append(f"{{ .port={port}, .rx={rxpin}, .tx={txpin} }}")

        self.write_device_table(f, 'serial devices', 'HAL_ESP32_UART_DEVICES', seriallist)

    def write_RCOUT_config(self, f):
        '''write rc output defines'''
        rcout_list = []
        for rcout in self.esp32_rcout:
            if len(rcout) != 1:
                self.error(f"Badly formed ESP32_RCOUT line {rcout} {len(rcout)=}")
            (gpio_num, ) = rcout
            rcout_list.append(gpio_num)

        if len(rcout_list) == 0:
            f.write("// No rc outputs\n")
            return
        f.write(f"#define HAL_ESP32_RCOUT {{ {', '.join(rcout_list)} }}\n")

    def write_ADC_config(self, f):
        '''write adc output defines'''
        if len(self.esp32_adcs) == 0:
            return

        outlist = []
        for e in self.esp32_adcs:
            outlist.append(f"{{ .channel={e.pin}, .scaling={e.gain}, .ardupin={e.ardupilotpin} }}")

        self.write_device_table(f, 'ADC pins', 'HAL_ESP32_ADC_PINS', outlist)

    def write_SDSPI_config(self, f):
        '''write sdspi output defines'''
        outlist = []
        for e in self.esp32_sdspi:
            outlist.append(f"{{.host={e.host}, .dma_ch={e.dma_ch}, .mosi={e.mosi}, .miso={e.miso}, .sclk={e.sclk}, .cs={e.cs}}}")  # NOQA:E501

        self.write_device_table(f, 'SDSPI configuration', 'HAL_ESP32_SDSPI', outlist)


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
