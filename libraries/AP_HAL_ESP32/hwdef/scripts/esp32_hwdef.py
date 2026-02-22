#!/usr/bin/env python3
'''
setup board.h for ESP32
AP_FLAKE8_CLEAN
'''

import os
import sys
import shlex

# Module level import not at top of file fix: move sys.path adjustment
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                             '../../../../libraries/AP_HAL/hwdef/scripts'))
import hwdef  # noqa: E402


class ESP32HWDef(hwdef.HWDef):
    # Hardware capability database for ESP32 variants
    CHIP_DATA = {
        'ESP32': {
            'reserved': {6, 7, 8, 9, 10, 11},
            'adc': {0, 2, 4, 12, 13, 14, 15, 25, 26, 27, 32, 33, 34, 35, 36, 37, 38, 39},
            'input_only': {34, 35, 36, 37, 38, 39},
            'strapping': {0, 2, 12, 15},
            'features': ['HAL_ESP32_HAS_MCPWM', 'HAL_ESP32_HAS_DAC']
        },
        'ESP32S2': {
            'reserved': {26, 27, 28, 29, 30, 31, 32},
            'adc': set(range(1, 21)),
            'input_only': {46},
            'strapping': {0, 45, 46},
            'features': ['HAL_ESP32_HAS_USB_OTG']
        },
        'ESP32S3': {
            'reserved': {26, 27, 28, 29, 30, 31, 32},
            'adc': set(range(1, 21)),
            'input_only': set(),
            'strapping': {0, 3, 45, 46},
            'features': ['HAL_ESP32_HAS_MCPWM', 'HAL_ESP32_HAS_USB_SERIAL_JTAG',
                         'HAL_ESP32_LARGE_BUFFERS']
        },
        'ESP32C3': {
            'reserved': {12, 13, 14, 15, 16, 17},
            'adc': {0, 1, 2, 3, 4, 5},
            'input_only': set(),
            'strapping': {2, 8, 9},
            'features': ['HAL_ESP32_HAS_USB_SERIAL_JTAG']
        },
        'ESP32C6': {
            'reserved': set(),
            'adc': set(),
            'input_only': set(),
            'strapping': set(),
            'features': ['HAL_ESP32_HAS_USB_SERIAL_JTAG', 'HAL_ESP32_HAS_MCPWM']
        },
        'ESP32P4': {
            'reserved': set(),
            'adc': set(),
            'input_only': set(),
            'strapping': set(),
            'features': ['HAL_ESP32_HAS_MCPWM', 'HAL_ESP32_HAS_MIPI',
                         'HAL_ESP32_HAS_USB_SERIAL_JTAG', 'HAL_ESP32_LARGE_BUFFERS']
        }
    }

    def __init__(self, outdir, hwdef, quiet=False, mcu='esp32', **kwargs):
        super(ESP32HWDef, self).__init__(outdir=outdir, hwdef=hwdef, quiet=quiet, **kwargs)
        self.board = os.path.basename(os.path.dirname(hwdef[0]))
        self.mcu = mcu
        self.advanced_build = False
        self.reserved_pins = set()
        self.input_only_pins = set()
        self.strapping_pins = set()
        self.init_hardware_constraints()
        self.serial_pins = {}
        self.rcout_pins = []
        self.spi_buses = []
        self.spi_devices = []
        self.i2c_buses = []
        self.adc_pins = []
        self.pin_assignments = {}
        self.sdspi = None
        self.flash_size_mb = None
        self.psram_size = None
        self.partition_table_filename = None

    def init_hardware_constraints(self):
        chip = self.CHIP_DATA.get(self.mcu.upper())
        if chip:
            self.reserved_pins.update(chip.get('reserved', set()))
            self.adc_capable_pins = chip.get('adc', set())
            self.input_only_pins = chip.get('input_only', set())
            self.strapping_pins = chip.get('strapping', set())
            self.progress(f"{self.mcu.upper()} constraints loaded")

    def validate_pin_assignment(self, pin_num, function, pin_name):
        try:
            pstr = pin_num.replace('GPIO_NUM_', '')
            if pstr.startswith('ADC1_GPIO'):
                pstr = pstr.replace('ADC1_GPIO', '').replace('_CHANNEL', '')
            if 'ADC_CHANNEL_' in pstr:
                return True
            pin_num_int = int(pstr)
        except Exception:
            return False

        if self.advanced_build:
            if pin_num_int in self.reserved_pins:
                self.error(f"Pin {pin_num_int} is reserved for system use")
            if pin_num_int in self.pin_assignments:
                existing = self.pin_assignments[pin_num_int]
                self.error(f"Pin conflict: GPIO{pin_num_int} assigned to "
                           f"'{existing}' and '{function} {pin_name}'")
            if pin_num_int in self.input_only_pins and function in \
               ['UART_TX', 'RCOUT', 'SPI_SCK', 'SPI_MOSI', 'I2C_SDA', 'CAN_TX']:
                self.error(f"Pin {pin_num_int} is input-only - "
                           f"cannot be used for output {function}")
            if pin_num_int in self.strapping_pins:
                self.progress(f"WARNING: GPIO{pin_num_int} is a strapping pin")

        self.pin_assignments[pin_num_int] = f"{function} {pin_name}"
        return True

    def process_line(self, line, depth=0):
        line = line.strip()
        if not line or line.startswith('#'):
            return

        # Keep track of all lines for output, like ChibiOS
        self.alllines.append(line)

        # MCU directive enables advanced logic and dynamic sdkconfig
        if line.startswith("MCU"):
            self.advanced_build = True
            self.mcu = line.split()[1]
            self.env_vars['MCU'] = self.mcu
            self.init_hardware_constraints()
            super(ESP32HWDef, self).process_line(line, depth)
            return

        if line.startswith("ESP32_SERIAL"):
            p = shlex.split(line)
            if len(p) == 4:
                num, tx, rx = p[1].replace('UART_NUM_', ''), p[2], p[3]
                self.validate_pin_assignment(tx, 'UART_TX', f'SERIAL{num}')
                self.validate_pin_assignment(rx, 'UART_RX', f'SERIAL{num}')
                self.serial_pins[f'tx_{num}'] = tx.replace('GPIO_NUM_', '')
                self.serial_pins[f'rx_{num}'] = rx.replace('GPIO_NUM_', '')
            return
        elif line.startswith("ESP32_RCOUT"):
            p = shlex.split(line)
            for pin in p[1:]:
                self.validate_pin_assignment(pin, 'RCOUT',
                                             f'CH{len(self.rcout_pins)+1}')
                self.rcout_pins.append(pin.replace('GPIO_NUM_', ''))
            return
        elif line.startswith("ESP32_SPIBUS"):
            p = shlex.split(line)
            if len(p) == 6:
                host_str = p[1].upper()
                if 'VSPI' in host_str or 'SPI3' in host_str:
                    num = 3
                elif 'HSPI' in host_str or 'SPI2' in host_str:
                    num = 2
                else:
                    num = 1
                dma, mosi, miso, sck = p[2], p[3], p[4], p[5]
                self.validate_pin_assignment(mosi, 'SPI_MOSI', f'BUS{num}')
                self.validate_pin_assignment(miso, 'SPI_MISO', f'BUS{num}')
                self.validate_pin_assignment(sck, 'SPI_SCK', f'BUS{num}')
                self.spi_buses.append({
                    'num': num,
                    'dma': dma,
                    'sck': sck.replace('GPIO_NUM_', ''),
                    'miso': miso.replace('GPIO_NUM_', ''),
                    'mosi': mosi.replace('GPIO_NUM_', '')
                })
            return
        elif line.startswith("ESP32_SPIDEV"):
            p = shlex.split(line)
            if len(p) == 8:
                name, bus, dev, cs, mode, low, high = p[1], p[2], p[3], p[4], p[5], p[6], p[7]
                self.validate_pin_assignment(cs, 'SPI_CS', name)
                self.spi_devices.append({
                    'name': name,
                    'bus': bus,
                    'device': dev,
                    'cs': cs.replace('GPIO_NUM_', ''),
                    'mode': mode,
                    'lspeed': low,
                    'hspeed': high
                })
            return
        elif line.startswith("ESP32_I2CBUS"):
            p = shlex.split(line)
            if len(p) >= 4:
                num = int(p[1].replace('I2C_NUM_', ''))
                sda, scl = p[2], p[3]
                self.validate_pin_assignment(sda, 'I2C_SDA', f'BUS{num}')
                self.validate_pin_assignment(scl, 'I2C_SCL', f'BUS{num}')
                self.i2c_buses.append({
                    'num': num,
                    'sda': sda.replace('GPIO_NUM_', ''),
                    'scl': scl.replace('GPIO_NUM_', '')
                })
            return
        elif line.startswith("ESP32_ADC_PIN") or line.startswith("ADC_PIN"):
            p = shlex.split(line)
            if len(p) == 4:
                pin, ardupilot_pin = p[1], p[3]
                self.validate_pin_assignment(pin, 'ADC', f'CH{ardupilot_pin}')
                self.adc_pins.append({'channel': int(ardupilot_pin), 'pin': pin})
            return
        elif line.startswith("ESP32_SDSPI"):
            p = shlex.split(line)
            if len(p) == 7:
                self.sdspi = {
                    'host': p[1],
                    'dma': p[2],
                    'mosi': p[3],
                    'miso': p[4],
                    'sclk': p[5],
                    'cs': p[6]
                }
            return
        elif line.startswith("FLASH_SIZE_MB"):
            self.flash_size_mb = int(line.split()[1])
            return
        elif line.startswith("PSRAM_SIZE"):
            self.psram_size = line.split()[1]
            return

        super(ESP32HWDef, self).process_line(line, depth)

        if line.startswith("RESERVED_PINS"):
            for pin in line.split()[1:]:
                self.reserved_pins.add(int(pin.replace('GPIO_NUM_', '')))

    def write_hwdef_header_content(self, f):
        self.advanced_build = ('MCU' in self.env_vars)

        f.write("#include <AP_HAL/board/esp32.h>\n")

        # Write all definitions from alllines, preserving strings
        for d in self.alllines:
            if d.startswith('define '):
                name = d.split()[1].split('(')[0]
                f.write("#undef %s\n#define %s\n" % (name, d[7:]))
            elif d.startswith('undef '):
                f.write("#undef %s\n" % d[6:])

        if self.advanced_build:
            f.write("#ifndef HAL_ESP32_HWDEF_V2\n#define "
                    "HAL_ESP32_HWDEF_V2 1\n#endif\n")
            chip = self.CHIP_DATA.get(self.mcu.upper())
            if chip:
                for feat in chip['features']:
                    f.write("#ifndef %s\n#define %s 1\n#endif\n" % (feat, feat))

        self.write_IMU_config(f)
        self.write_BARO_config(f)
        self.write_esp32_config(f)

    def write_esp32_config(self, f):
        f.write("\n/* UART Configuration */\n"
                "#ifndef HAL_ESP32_UART_DEVICES\n"
                "#define HAL_ESP32_UART_DEVICES \\\n")
        entries = []
        for key in sorted(self.serial_pins.keys()):
            if key.startswith('tx_'):
                num = key.split('_')[1]
                if f'rx_{num}' in self.serial_pins:
                    entries.append(f'    {{ .rx = GPIO_NUM_{self.serial_pins[f"rx_{num}"]}, '
                                   f'.tx = GPIO_NUM_{self.serial_pins[key]} }}')
        f.write(',\\\n'.join(entries) + '\n#endif\n')

        if self.rcout_pins:
            valid = [p for p in self.rcout_pins if p is not None]
            f.write(f"\n#ifndef HAL_ESP32_RCOUT_MAX\n#define "
                    f"HAL_ESP32_RCOUT_MAX {len(valid)}\n#endif\n")
            f.write(f"#ifndef HAL_ESP32_RCOUT\n#define HAL_ESP32_RCOUT "
                    f"{{{','.join([f'GPIO_NUM_{p}' for p in valid])}}}\n"
                    "#endif\n")

        if self.spi_buses:
            f.write("\n/* SPI Bus Configuration */\n#ifndef "
                    "HAL_ESP32_SPI_BUSES\n#define HAL_ESP32_SPI_BUSES \\\n")
            entries = []
            for b in sorted(self.spi_buses, key=lambda x: x['num']):
                # Map bus number to host name for ESP32
                host = "SPI2_HOST" if b['num'] == 2 else "SPI3_HOST" if \
                       b['num'] == 3 else "SPI1_HOST"
                entries.append(f"    {{.host={host}, .dma_ch={b['dma']}, "
                               f".mosi=GPIO_NUM_{b['mosi']}, "
                               f".miso=GPIO_NUM_{b['miso']}, "
                               f".sclk=GPIO_NUM_{b['sck']}}}")
            f.write(',\\\n'.join(entries) + '\n#endif\n')

        if self.spi_devices:
            f.write("\n/* SPI Device Configuration */\n#ifndef "
                    "HAL_ESP32_SPI_DEVICES\n#define HAL_ESP32_SPI_DEVICES \\\n")
            entries = []
            for i, d in enumerate(self.spi_devices):
                entries.append(f'    {{.name="{d["name"]}", .bus={d["bus"]}, '
                               f'.device={d["device"]}, .cs=GPIO_NUM_{d["cs"]}, '
                               f'.mode={d["mode"]}, .lspeed={d["lspeed"]}, '
                               f'.hspeed={d["hspeed"]}}}')
            f.write(',\\\n'.join(entries) + '\n#endif\n')

        if self.i2c_buses:
            f.write("\n/* I2C Configuration */\n#ifndef "
                    "HAL_ESP32_I2C_BUSES\n#define HAL_ESP32_I2C_BUSES \\\n")
            entries = [f"    {{.port=I2C_NUM_{b['num']}, "
                       f".sda=GPIO_NUM_{b['sda']}, .scl=GPIO_NUM_{b['scl']}, "
                       ".speed=400*KHZ, .internal=true}" for b in
                       sorted(self.i2c_buses, key=lambda x: x['num'])]
            f.write(',\\\n'.join(entries) + '\n#endif\n')

        if self.adc_pins:
            f.write("\n/* ADC Configuration */\n#ifndef HAL_ESP32_ADC_PINS\n"
                    "#define HAL_ESP32_ADC_PINS \\\n")
            entries = []
            for p in sorted(self.adc_pins, key=lambda x: x['channel']):
                entries.append(f"    {{{p['pin']}, 11, {p['channel']}}}")
            f.write(',\\\n'.join(entries) + '\n#endif\n')

        if self.sdspi:
            f.write(f"\n#ifndef HAL_ESP32_SDSPI\n"
                    f"#define HAL_ESP32_SDSPI {{.host={self.sdspi['host']}, "
                    f".dma_ch={self.sdspi['dma']}, .mosi={self.sdspi['mosi']}, "
                    f".miso={self.sdspi['miso']}, .sclk={self.sdspi['sclk']}, "
                    f".cs={self.sdspi['cs']}}}\n#endif\n")

    def generate_esp_idf_config(self):
        config_lines = []
        if self.flash_size_mb:
            # Set the selected flash size and its corresponding string value
            config_lines.append(f"CONFIG_ESPTOOLPY_FLASHSIZE_{self.flash_size_mb}MB=y")
            config_lines.append(f'CONFIG_ESPTOOLPY_FLASHSIZE="{self.flash_size_mb}MB"')
            # Explicitly disable other common flash sizes to ensure override
            for size in [1, 2, 4, 8, 16, 32, 64, 128]:
                if size != self.flash_size_mb:
                    config_lines.append(f"# CONFIG_ESPTOOLPY_FLASHSIZE_{size}MB is not set")
        if self.psram_size:
            config_lines.append("CONFIG_SPIRAM=y")
            config_lines.append("CONFIG_SPIRAM_TYPE_AUTO=y")
            config_lines.append("CONFIG_SPIRAM_MODE_QUAD=y")
        if self.partition_table_filename:
            config_lines.append("CONFIG_PARTITION_TABLE_CUSTOM=y")
            config_lines.append("CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="
                                f'"{self.partition_table_filename}"')

        hal_with_wifi = self.intdefines.get('HAL_WITH_WIFI', '1')
        if str(hal_with_wifi) == '0':
            config_lines.append("# CONFIG_ESP_WIFI_ENABLED is not set")
        else:
            config_lines.append("CONFIG_ESP_WIFI_ENABLED=y")

        # Essential coredump and panic behavior for ArduPilot on ESP32
        config_lines.extend([
            "CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH=y",
            "CONFIG_ESP_SYSTEM_PANIC_PRINT_HALT=y",
            "CONFIG_ESP_COREDUMP_MAX_TASKS_NUM=64",
        ])
        
        return config_lines

    def write_esp_idf_config(self, filename="sdkconfig.board"):
        config_lines = self.generate_esp_idf_config()
        if not config_lines:
            return
        fname = os.path.join(self.outdir, filename)
        with open(fname, "w") as f:
            f.write(f"# Auto-generated ESP-IDF configuration for {self.board}\n")
            for line in config_lines:
                f.write(line + "\n")

    def copy_partition_table(self):
        if not self.partition_table_filename:
            return
        hwdef_dir = os.path.dirname(self.hwdef[0])
        src = os.path.join(hwdef_dir, self.partition_table_filename)
        if os.path.exists(src):
            import shutil
            shutil.copy2(src, os.path.join(self.outdir,
                                           self.partition_table_filename))


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-D', '--outdir', required=True)
    parser.add_argument('--quiet', action='store_true')
    parser.add_argument('--mcu', default='esp32')
    parser.add_argument('hwdef', nargs='+')
    args = parser.parse_args()
    eh = ESP32HWDef(args.outdir, args.hwdef, quiet=args.quiet, mcu=args.mcu)
    eh.run()
    eh.write_esp_idf_config()
    eh.copy_partition_table()
