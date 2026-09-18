#!/usr/bin/env python3
# encoding: utf-8
"""
Zephyr hwdef.dat processor.

Parses a simplified hwdef.dat (ArduPilot sensor/vehicle configuration) and
generates hwdef.h for inclusion via <hwdef.h> in libraries/AP_HAL/board/zephyr.h.

This is the Zephyr equivalent of the ChibiOS hwdef.py processor.  Pin mux,
peripheral enables, and clock configuration are handled by the Zephyr DTS
file; this script covers only the ArduPilot-layer sensor and vehicle
configuration that sits on top of DTS.

Supported directives (case-sensitive):
    BOARD_NAME <name>
    MCU <type>            (informational only; ignored by generator)
    OSCILLATOR_HZ <n>    (informational only; ignored by generator)
    SERIAL_ORDER <ports> (SERIALn -> nth port; emits HAL_UART_DT_DEVICE_LOOKUP)
    CAN_ORDER <buses>    (informational only; logged for reference)
    SPIDEV <args>        (informational only; DTS handles SPI bus config)
    IMU <driver> SPI:<device_name> [<rotation>]
    IMU <driver> SPI:<accel_dev> SPI:<gyro_dev> [<rotation>]  (BMI088/BMI055)
    BARO <driver> SPI:<device_name>
    BARO <driver> I2C:<bus>:<hex_addr>
    COMPASS <driver> I2C:<bus>:<hex_addr> [<external: true|false>] [<rotation>]
    COMPASS <driver>:<probe_method> <instance> <rotation>
        (e.g. COMPASS AK09916:probe_ICM20948 0 ROTATION_ROLL_180_YAW_90)
    define <NAME> [<value>]     (any freeform define passed through verbatim)

Output format follows the real ChibiOS hwdef.py:
    HAL_INS_PROBE{n}   / HAL_INS_PROBE_LIST
    HAL_BARO_PROBE{n}  / HAL_BARO_PROBE_LIST
    HAL_MAG_PROBE{n}   / HAL_MAG_PROBE_LIST

AP_FLAKE8_CLEAN
"""

import os
import re
import sys

# Driver name aliases: hwdef.dat driver name → AP_Baro_<class> suffix.
# Only needed when the class name differs from the driver name.
_BARO_CLASS_ALIAS = {
    'ICP20100': 'ICP201XX',  # ICP20100 is the same silicon as ICP201XX
}


def _baro_class(driver):
    return _BARO_CLASS_ALIAS.get(driver, driver)


class ZephyrHWDef:
    def __init__(self, hwdef_path, is_bootloader=False):
        self.hwdef_path = hwdef_path
        # Bootloader builds must define HAL_BOOTLOADER_BUILD, as chibios_hwdef.py
        # does. Without it GCS_MAVLink.h is still pulled in and fails, because
        # wscript skips the mavgen target - and its export_includes - for bootloaders.
        self.is_bootloader = is_bootloader
        self.board_name = 'Zephyr'
        self.mcu = ''          # raw MCU string from "MCU <type>" directive
        self.defines = []   # list of (name, value) preserving order
        self.imus = []
        self.baros = []
        self.compasses = []
        self.spidevs = []  # list of SPI device definitions from hwdef.dat
        self.i2c_order = []  # ArduPilot I2C bus index -> peripheral, from I2C_ORDER
        self.serial_order = []  # ArduPilot SERIALn -> peripheral, from SERIAL_ORDER
        self.iomcu_uart = None  # peripheral wired to an IOMCU, from IOMCU_UART
        self.romfs = []         # (embedded name, source path), from ROMFS
        self.can_order = []  # CAN bus indices, from CAN_ORDER (class generator)
        self.can_pin_buses = set()  # CAN bus numbers seen on pin directives
        self.gen_lines = []  # PIN/DMA directive token lists (class generator)
        self._parse()

    def _get_config(self, name, default):
        """
        Value of a hwdef.dat 'define <name> <value>' directive, or default.
        The equivalent of chibios_hwdef.py's get_config() for the small subset
        of config this parser needs.
        """
        for dname, dvalue in self.defines:
            if dname == name:
                return dvalue
        return default

    @staticmethod
    def _serial_dt_labels(periph):
        # DTS nodelabel candidates for one SERIAL_ORDER token, most
        # specific first. USARTn/UARTn also try lpuartN so the same
        # token works on NXP SoCs, whose UARTs are all named lpuartN.
        p = periph.upper()
        if p in ('EMPTY', 'NONE'):
            return []
        if p in ('WIFI_TCP', 'WIFI_UDP'):
            # WiFi virtual serial ports (WiFiDriver.cpp) - no devicetree
            # node; the SERIAL_ORDER generator below emits
            # HAL_ZEPHYR_WIFI_{TCP,UDP}_SERIAL index defines instead.
            return []
        if p.startswith('OTG'):
            idx = int(p[3:]) - 1
            return ['usb_cdc_acm%d' % idx, 'cdc_acm%d' % idx]
        if p.startswith('LPUART'):
            return [p.lower()]
        if p.startswith('USART'):
            return ['usart%s' % p[5:], 'lpuart%s' % p[5:]]
        if p.startswith('UART'):
            return ['uart%s' % p[4:], 'lpuart%s' % p[4:]]
        return [p.lower()]

    @staticmethod
    def _spi_dt_labels(bus_num):
        # DTS nodelabel candidates for an SPI bus index. STM32 labels its
        # controllers spiN; NXP labels them lpspiN — try both so the same
        # SPIDEV bus number works on either SoC.
        return ['spi%d' % bus_num, 'lpspi%d' % bus_num]

    @staticmethod
    def _i2c_dt_labels(periph):
        # DTS nodelabel candidates for an I2C_ORDER token. STM32 = i2cN,
        # NXP = lpi2cN; try both. An explicit LPI2Cn token pins NXP only.
        p = periph.upper()
        if p.startswith('LPI2C'):
            return ['lpi2c%s' % p[5:]]
        if p.startswith('I2C'):
            n = p[3:]
            return ['i2c%s' % n, 'lpi2c%s' % n]
        return [p.lower()]

    # Manufacturer prefix rules: longest match wins.
    # Each entry is (mcu_prefix, manufacturer, soc_family_fn).
    # soc_family_fn(mcu) → short soc-family name for the Layer-3 conf file.
    _MCU_MFR_RULES = [
        # Espressif: ESP32S3 → esp32 / esp32s3
        #            ESP32C3 → esp32 / esp32c3   etc.
        ('ESP32', 'esp32', lambda m: m.lower().split('/')[0]),
        # STMicro:  STM32H743xx → stm32 / stm32h7
        #           STM32F4xx   → stm32 / stm32f4   etc.
        ('STM32', 'stm32', lambda m: 'stm32' + m[5:7].lower()),
        # NXP/i.MX RT: MIMXRT1176 → nxp / nxprt1176
        #              MIMXRT1064 → nxp / nxprt1064
        ('MIMXRT', 'nxp', lambda m: 'nxprt' + m[6:10]),
        # NXP Kinetis: MK66FN → nxp / mk6x
        ('MK', 'nxp', lambda m: m[:3].lower() + 'x'),
        # Renesas RA: RA6M5 → renesas / ra6
        ('RA', 'renesas', lambda m: m[:3].lower()),
    ]

    def get_mfr_soc(self):
        """Derive (manufacturer, soc_family) from the MCU name in hwdef.dat.

        Returns ('', '') when no rule matches.  The result is used as the
        Layer-2 and Layer-3 Kconfig fragment names, e.g.:
            ('espressif', 'esp32s3') → prj.espressif.conf + prj.esp32s3.conf
        """
        mcu = self.mcu.upper()
        for prefix, mfr, soc_fn in self._MCU_MFR_RULES:
            if mcu.startswith(prefix.upper()):
                try:
                    soc = soc_fn(self.mcu)
                except Exception:  # noqa: BLE001
                    soc = ''
                return (mfr, soc)
        return ('', '')

    # ------------------------------------------------------------------
    # Parser
    # ------------------------------------------------------------------

    def _parse(self):
        self._parse_file(self.hwdef_path)
        self._auto_wifi_serials()

    def _auto_wifi_serials(self):
        """WiFi MAVLink serials on EVERY ESP32-family board (maintainer
        decision 2026-08-15): place WIFI_TCP (TCP server :5760) and
        WIFI_UDP (:14550) into the first two EMPTY SERIAL_ORDER slots, or
        append them, unless the hwdef already names them or opts out with
        `define AP_ZEPHYR_WIFI_DISABLE 1`. The Kconfig side lives in the
        family fragment prj.esp32.conf; drivers in WiFiDriver.cpp."""
        if not self.mcu.upper().startswith('ESP32') or self.is_bootloader:
            return
        for name, value in self.defines:
            if name == 'AP_ZEPHYR_WIFI_DISABLE' and str(value).strip() == '1':
                return
        upper = [p.upper() for p in self.serial_order]
        if 'WIFI_TCP' in upper or 'WIFI_UDP' in upper:
            return
        for token in ('WIFI_TCP', 'WIFI_UDP'):
            placed = False
            for i, p in enumerate(self.serial_order):
                if p.upper() in ('EMPTY', 'NONE'):
                    self.serial_order[i] = token
                    placed = True
                    break
            if not placed:
                self.serial_order.append(token)

    def _parse_file(self, path):
        base_dir = os.path.dirname(path)
        with open(path, 'r') as f:
            for raw in f:
                line = raw.strip()
                if not line or line.startswith('#'):
                    continue
                tokens = line.split()
                if tokens[0] == 'include':
                    inc_path = os.path.join(base_dir, tokens[1])
                    if os.path.exists(inc_path):
                        self._parse_file(inc_path)
                else:
                    self._dispatch(line)

    def _dispatch(self, line):
        tokens = line.split()
        cmd = tokens[0]

        if cmd == 'BOARD_NAME':
            self.board_name = ' '.join(tokens[1:]).strip('"\'')

        elif cmd == 'MCU':
            self.mcu = tokens[1] if len(tokens) > 1 else ''

        elif cmd in ('define',):
            name = tokens[1]
            value = ' '.join(tokens[2:]) if len(tokens) > 2 else '1'
            # Later defines win, as in chibios_hwdef.py: a board hwdef.dat can
            # override a define inherited from an earlier 'include' (e.g.
            # hwdef.inc) without emitting two conflicting #defines into hwdef.h.
            self.defines = [(n, v) for n, v in self.defines if n != name]
            self.defines.append((name, value))

        elif cmd == 'IMU':
            # IMU <driver> SPI:<zephyr_device_name> [<rotation>]
            if len(tokens) < 3:
                return
            driver = tokens[1]
            # Two SPI: connections are allowed, for parts with SEPARATE accel
            # and gyro chip selects whose probe() takes two devices - BMI088,
            # BMI055, SCHA63T. Order is (accel, gyro), matching those drivers.
            conns = [t for t in tokens[2:] if t.startswith('SPI:')]
            if not conns:
                print('zephyr_hwdef: WARNING IMU connection must be SPI: — got %s' % tokens[2])
                return
            if len(conns) > 2:
                print('zephyr_hwdef: WARNING IMU %s: max 2 SPI devices, got %d' % (driver, len(conns)))
                return
            rotation = 'ROTATION_NONE'
            for t in tokens[2 + len(conns):]:
                if not t.startswith('SPI:'):
                    rotation = t
                    break
            self.imus.append(dict(driver=driver, devices=[c[4:] for c in conns], rotation=rotation))

        elif cmd == 'BARO':
            # BARO <driver> SPI:<device_name>
            # BARO <driver> I2C:<bus>:<hex_addr>
            if len(tokens) < 3:
                return
            driver = tokens[1]
            conn = tokens[2]
            if conn.startswith('SPI:'):
                device_name = conn[4:]
                self.baros.append(dict(driver=driver, conn='SPI', device=device_name))
            else:
                bus, addr = self._parse_i2c(conn)
                if bus is None:
                    return
                self.baros.append(dict(driver=driver, conn='I2C', bus=bus, addr=addr))

        elif cmd == 'COMPASS':
            # COMPASS <driver>:<probe_method> <instance> <rotation>
            #   e.g.  COMPASS AK09916:probe_ICM20948 0 ROTATION_ROLL_180_YAW_90
            # COMPASS <driver> I2C:<bus>:<hex_addr> [external] [rotation]
            if len(tokens) < 3:
                return
            driver_field = tokens[1]
            conn = tokens[2]

            if ':' in driver_field:
                # Special probe-method form: <driver>:<method>
                driver, probe_method = driver_field.split(':', 1)
                instance = conn   # tokens[2] is the instance index
                rotation = tokens[3] if len(tokens) > 3 else 'ROTATION_NONE'
                self.compasses.append(dict(driver=driver, conn='PROBE_METHOD',
                                           probe_method=probe_method,
                                           instance=instance, rotation=rotation))
            elif conn.startswith('I2C:'):
                bus, addr = self._parse_i2c(conn)
                if bus is None:
                    return
                external = tokens[3] if len(tokens) > 3 else 'false'
                rotation = tokens[4] if len(tokens) > 4 else 'ROTATION_NONE'
                self.compasses.append(dict(driver=driver_field, conn='I2C',
                                           bus=bus, addr=addr,
                                           external=external, rotation=rotation))
            else:
                print('zephyr_hwdef: WARNING COMPASS: unsupported connection %s' % conn)
                return

        elif cmd == 'I2C_ORDER':
            # I2C_ORDER I2C2 I2C1 - ArduPilot bus index n maps to the nth
            # peripheral listed (bus 0 = I2C2, bus 1 = I2C1 in this example).
            #
            # The filter used to be startswith('I2C'), which is false for
            # 'LPI2C1' - it starts with 'LP'. NXP tokens were therefore
            # dropped even though _i2c_dt_labels() explicitly resolves them,
            # and because a dropped token is REMOVED rather than kept, every
            # bus after it silently shifted down one index. 'I2C_ORDER LPI2C1
            # I2C2' gave bus 0 = I2C2 and no bus 1 at all.
            #
            # Accept both spellings, keep EMPTY/NONE as index placeholders the
            # way SERIAL_ORDER does, and warn about anything else rather than
            # renumbering the remaining buses behind the board author's back.
            self.i2c_order = []
            for t in tokens[1:]:
                u = t.upper()
                if u in ('EMPTY', 'NONE') or u.startswith(('I2C', 'LPI2C')):
                    self.i2c_order.append(t)
                else:
                    print('zephyr_hwdef: WARNING I2C_ORDER: ignoring '
                          'unrecognised token %s (bus indices after it would '
                          'shift)' % t)

        elif cmd == 'SERIAL_ORDER':
            # SERIAL_ORDER OTG1 USART2 USART3 UART4 UART8 UART7 OTG2 —
            # ArduPilot SERIALn maps to the nth peripheral listed, same
            # semantics as the ChibiOS hwdef directive. EMPTY reserves an
            # index with no device.
            self.serial_order = tokens[1:]

        elif cmd == 'ROMFS':
            # ROMFS <name-in-romfs> <path-in-tree> - files to embed via
            # AP_ROMFS. Same directive as the ChibiOS hwdef. AP_IOMCU needs
            # its io_firmware.bin here to CRC-check the IO co-processor.
            if len(tokens) >= 3:
                self.romfs.append((tokens[1], tokens[2]))

        elif cmd == 'IOMCU_UART':
            # IOMCU_UART USART6 - the port wired to an IO co-processor. Same
            # directive as the ChibiOS hwdef. It is not a user-facing SERIALn:
            # it is appended after SERIAL_ORDER so AP_SerialManager never
            # offers it, and only AP_IOMCU talks to it.
            self.iomcu_uart = tokens[1]

        elif cmd == 'CAN_ORDER':
            # CAN_ORDER 1 2 — CAN bus indices, consumed by the class
            # generator (zephyr_class_generator.py) for flexcan/fdcan
            # enablement + Kconfig inference.
            self.can_order = tokens[1:]

        elif re.match(r'^P[A-Z]\d+$', cmd) and len(tokens) >= 3:
            # A pin directive: P<port><n> <signal> <peripheral>, e.g.
            # "PD0 CAN1_RX CAN1". chibios_hwdef.py derives the CAN bus count
            # from exactly these when no CAN_ORDER is given, so do the same -
            # otherwise a board that describes its CAN pins still gets
            # HAL_NUM_CAN_IFACES 0 and no CAN at all.
            m = re.match(r'^CAN(\d+)$', tokens[2])
            if m:
                self.can_pin_buses.add(int(m.group(1)))
                # Hand the class generator the same pin as a PIN directive, so
                # a board that declares its CAN pins the ChibiOS way gets the
                # controller enabled with its pinctrl in the generated overlay
                # - no second, hand-written description of the same wiring.
                # tokens[1] is the signal (CAN1_RX), cmd is the pad (PD0).
                self.gen_lines.append(['PIN', cmd, tokens[1]])

        elif cmd in ('PIN', 'DMA', 'PERIPH'):
            # Class-generator directives (zephyr_class_generator.py): PIN <PAD> <FUNC>,
            # DMA <PERIPH> <rx> <tx>, PERIPH <node> <prop>. Collected verbatim with
            # include-chain order preserved; this parser does nothing with them.
            self.gen_lines.append(tokens)

        elif cmd == 'SPIDEV':
            # SPIDEV <name> <bus> <devid> <cs_pin> <mode> <low_speed> <high_speed>
            # e.g. SPIDEV ms5611 SPI1 DEVID3 BARO_CS MODE3 20*MHZ 20*MHZ
            if len(tokens) < 8:
                return
            name = tokens[1]
            bus_str = tokens[2]  # e.g. "SPI1", "SPI4"
            # tokens[3] is DEVIDn, e.g. "DEVID1", "DEVID3" (not used here)
            cs_pin = tokens[4]   # e.g. "BARO_CS", "MPU_CS" (mapped later in SPIDevice.cpp)
            mode = tokens[5]     # e.g. "MODE3" (not used here, Zephyr handles SPI modes)
            low_speed = tokens[6]   # e.g. "20*MHZ"
            high_speed = tokens[7]  # e.g. "20*MHZ"

            # Extract bus number: "SPI1" → 1, "SPI4" → 4
            try:
                bus_num = int(bus_str[3:])
            except (ValueError, IndexError):
                return

            # Parse speed: "20*MHZ" -> 20000000, "1.5*MHZ" -> 1500000
            #
            # The old version textually replaced the unit with its zeros and
            # then stripped the '*', so "1.5*MHZ" became "1.5000000", which
            # contains '.' and so went through eval() as the NUMBER 1.5 and
            # truncated to 1 Hz. Every fractional speed in a hwdef was silently
            # reduced to 1 Hz - a bus that then ran at the driver's minimum
            # with nothing reported.
            #
            # Multiply by the unit instead of pasting digits onto the mantissa.
            UNITS = (('MHZ', 1000000), ('KHZ', 1000), ('HZ', 1))

            def parse_speed(s):
                text = s.upper().strip()
                mult = 1
                for suffix, scale in UNITS:
                    if text.endswith(suffix):
                        mult = scale
                        text = text[:-len(suffix)].rstrip()
                        break
                text = text.rstrip('*').strip()
                if not text:
                    text = '1'
                try:
                    return int(round(float(text) * mult))
                except ValueError:
                    return 1000000  # default 1 MHz

            low_hz = parse_speed(low_speed)
            high_hz = parse_speed(high_speed)

            # "MODE3" → 3 (SPI CPOL/CPHA mode number)
            try:
                mode_num = int(mode[4:]) if mode.upper().startswith('MODE') else 0
            except ValueError:
                mode_num = 0

            self.spidevs.append(dict(name=name, bus=bus_num, cs_pin=cs_pin,
                                     mode=mode_num,
                                     low_hz=low_hz, high_hz=high_hz))

        # MCU, OSCILLATOR_HZ, CAN_ORDER: parsed but not used by the
        # generator — DTS/waf handle those aspects.

    @staticmethod
    def _parse_i2c(conn):
        if not conn.startswith('I2C:'):
            print('zephyr_hwdef: WARNING expected I2C: connection, got %s' % conn)
            return None, None
        parts = conn[4:].split(':')
        if len(parts) != 2:
            print('zephyr_hwdef: WARNING malformed I2C spec: %s' % conn)
            return None, None
        bus = int(parts[0])
        addr_str = parts[1]
        addr = int(addr_str, 16) if addr_str.startswith('0x') or addr_str.startswith('0X') else int(addr_str)
        return bus, addr

    # ------------------------------------------------------------------
    # Generator
    # ------------------------------------------------------------------

    def generate_hwdef_h(self, output_path):
        lines = [
            '#pragma once',
            '',
            '/* Generated by Tools/ardupilotwaf/zephyr_hwdef.py',
            ' * Source: %s' % os.path.basename(self.hwdef_path),
            ' * Do not edit — re-run ./waf configure to regenerate.',
            ' */',
            '',
        ]

        lines.append('#define HAL_BOARD_NAME "%s"' % self.board_name)
        lines.append('')

        # USB identity strings, mirroring chibios_hwdef.py's write_USB_config():
        # the product string carries a "-BL" suffix on bootloader builds so the
        # host can tell a running bootloader from a running app - they share a
        # VID:PID otherwise. hwdef.dat can override either with
        #   define USB_STRING_PRODUCT "..."
        lines.append('// USB configuration')
        lines.append('#define HAL_USB_STRING_MANUFACTURER %s' %
                     self._get_config('USB_STRING_MANUFACTURER', '"ArduPilot"'))
        # chibios_hwdef.py appends "-BL" unconditionally because its %BOARD%
        # is the board DIRECTORY name. Ours is BOARD_NAME from the hwdef, and
        # this tree's hwdef-bl.dat already declares it with the suffix - so
        # only add it when it is not there, or the product string ends up
        # "..-BL-BL".
        default_product = self.board_name
        if self.is_bootloader and not default_product.endswith('-BL'):
            default_product += '-BL'
        lines.append('#define HAL_USB_STRING_PRODUCT %s' %
                     self._get_config('USB_STRING_PRODUCT', '"%s"' % default_product))
        lines.append('')

        # setup for bootloader build (mirrors chibios_hwdef.py)
        if self.is_bootloader:
            lines.append('#define HAL_BOOTLOADER_BUILD TRUE')
            # TRUE/FALSE come from ChibiOS's hal.h and are undefined under Zephyr, so
            # `#if HAL_USE_CAN == TRUE` evaluates as 0 == 0 and wrongly pulls in
            # can_start()/stm32_watchdog_init(). Define them so the guard resolves.
            lines.append('')
            lines.append('#ifndef TRUE')
            lines.append('#define TRUE 1')
            lines.append('#endif')
            lines.append('#ifndef FALSE')
            lines.append('#define FALSE 0')
            lines.append('#endif')
            lines.append('#define HAL_USE_CAN FALSE')
            lines.append('#define HAL_NUM_CAN_IFACES 0')
            lines.append('')

        # --- SPI device list (from SPIDEV directives) ---
        if self.spidevs:
            # CS index: position of the device's CS pin name within the
            # per-bus ordered-unique list of CS names, in SPIDEV appearance
            # order (shared with the spi_dt_spec generation below).
            bus_cs_order = {}   # bus_num -> [cs_pin_name, ...]
            for spi in self.spidevs:
                order = bus_cs_order.setdefault(spi['bus'], [])
                if spi['cs_pin'] not in order:
                    order.append(spi['cs_pin'])

            lines.append('/* SPI device list — generated from hwdef.dat SPIDEV directives */')
            lines.append('#define HAL_SPI_DEVICES_LIST \\')
            for n, spi in enumerate(self.spidevs):
                # Map device name to device type (IMU0, IMU1, IMU2, BARO, FRAM)
                device_type = 'SPIDevice::DeviceId::IMU0'  # default
                if 'icm20602_ext' in spi['name'] or 'gyro_ext' in spi['name'].lower():
                    device_type = 'SPIDevice::DeviceId::IMU0'
                elif 'icm20948_ext' in spi['name'] or 'mpu_ext' in spi['name'].lower():
                    device_type = 'SPIDevice::DeviceId::IMU1'
                elif 'icm20948' in spi['name'] or ('mpu' in spi['name'].lower() and '_ext' not in spi['name']):
                    device_type = 'SPIDevice::DeviceId::IMU2'
                elif 'ms5611' in spi['name'] or 'baro' in spi['name'].lower():
                    device_type = 'SPIDevice::DeviceId::BARO'
                elif 'fram' in spi['name'].lower() or 'ramtron' in spi['name'].lower():
                    device_type = 'SPIDevice::DeviceId::FRAM'

                cs_index = bus_cs_order[spi['bus']].index(spi['cs_pin'])

                # Add comma if not the last element (for array initialization)
                line_end = ', \\' if n < len(self.spidevs) - 1 else ''
                lines.append('  {"%s", %s, %d, %d, %dU, %dU}%s' %
                             (spi['name'], device_type, spi['bus'], cs_index,
                              spi['low_hz'], spi['high_hz'], line_end))
            lines.append('')

            # --- SPI spi_dt_spec declarations + name-based lookup ---
            # Built entirely from hwdef.dat; the DTS contributes only controllers,
            # pinctrl and cs-gpios, which MUST be listed in bus_cs_order.
            lines.append('/* Per-bus CS index order (DTS cs-gpios arrays must match):')
            for bus_num in sorted(bus_cs_order):
                lines.append(' *   spi%d: %s' % (bus_num, ' '.join(
                    '%d=%s' % (i, n) for i, n in enumerate(bus_cs_order[bus_num]))))
            lines.append(' */')

            # hwdef MODEn → Zephyr CPOL/CPHA operation flags
            _mode_flags = {
                0: '',
                1: ' | SPI_MODE_CPHA',
                2: ' | SPI_MODE_CPOL',
                3: ' | SPI_MODE_CPOL | SPI_MODE_CPHA',
            }

            lines.append('/* SPI spi_dt_spec declarations — generated from hwdef.dat SPIDEV directives */')
            lines.append('#define HAL_SPI_DT_SPEC_DECLS \\')
            decl_lines = []
            for spi in self.spidevs:
                name = spi['name']
                bus = spi['bus']
                cs_idx = bus_cs_order[bus].index(spi['cs_pin'])
                flags = _mode_flags.get(spi.get('mode', 0), '')
                # emit under both spiN and lpspiN nodelabels (STM32 vs NXP);
                # only the label that exists on the board compiles
                for label in self._spi_dt_labels(bus):
                    decl_lines.append(
                        '  IF_ENABLED(DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(%s)), '
                        '(static const struct spi_dt_spec spi_spec_%s = { '
                        '.bus = DEVICE_DT_GET(DT_NODELABEL(%s)), '
                        '.config = { '
                        '.frequency = %dU, '
                        '.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB%s, '
                        '.slave = 0, '
                        '.cs = { .gpio = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(%s), cs_gpios, %d), '
                        '.delay = 0, .cs_is_gpio = true }, '
                        '} };))'
                        % (label, name, label, spi['high_hz'], flags, label, cs_idx)
                    )
            lines.append(' \\\n'.join(decl_lines))
            lines.append('')

            lines.append('/* SPI spi_dt_spec name-based lookup — generated from hwdef.dat SPIDEV directives */')
            lines.append('#define HAL_SPI_DT_SPEC_LOOKUP(name_var) \\')
            lookup_lines = []
            for spi in self.spidevs:
                name = spi['name']
                for label in self._spi_dt_labels(spi['bus']):
                    lookup_lines.append(
                        '  IF_ENABLED(DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(%s)), '
                        '(if (strcmp(name_var, "%s") == 0) { return &spi_spec_%s; }))'
                        % (label, name, name)
                    )
            lines.append(' \\\n'.join(lookup_lines))
            lines.append('')

        # --- I2C bus lookup (from I2C_ORDER) ---
        # Maps ArduPilot I2C bus index -> Zephyr controller nodelabel, e.g.
        # "I2C_ORDER I2C2 I2C1" gives bus 0 = i2c2, bus 1 = i2c1. Used by
        # I2CDevice.cpp instead of a hardcoded per-SoC switch.
        if self.i2c_order:
            lines.append('/* I2C bus lookup — generated from hwdef.dat I2C_ORDER */')
            lines.append('#define HAL_I2C_DT_DEVICE_LOOKUP(bus_var) \\')
            i2c_lines = []
            for n, periph in enumerate(self.i2c_order):
                # try both i2cN and lpi2cN (STM32 vs NXP nodelabels)
                for label in self._i2c_dt_labels(periph):
                    i2c_lines.append(
                        '  IF_ENABLED(DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(%s)), '
                        '(if ((bus_var) == %d) { return DEVICE_DT_GET(DT_NODELABEL(%s)); }))'
                        % (label, n, label)
                    )
            lines.append(' \\\n'.join(i2c_lines))
            lines.append('#define HAL_I2C_BUS_COUNT %d' % len(self.i2c_order))
            lines.append('')

        # --- UART device lookup (from SERIAL_ORDER) ---
        # Maps ArduPilot SERIALn to a Zephyr nodelabel. Each token expands to every
        # plausible label guarded by IF_ENABLED, so only existing okay nodes emit.
        if self.serial_order:
            # The IOMCU port goes after the user-facing ones, exactly as
            # chibios_hwdef.py does it, so SERIALn numbering does not shift.
            serial_order = list(self.serial_order)
            iomcu_idx = None
            if self.iomcu_uart:
                if self.iomcu_uart in serial_order:
                    iomcu_idx = serial_order.index(self.iomcu_uart)
                else:
                    iomcu_idx = len(serial_order)
                    serial_order.append(self.iomcu_uart)
            lines.append('/* UART device lookup — generated from hwdef.dat SERIAL_ORDER */')
            lines.append('#define HAL_UART_DT_DEVICE_LOOKUP(serial_var) \\')
            uart_lines = []
            for n, periph in enumerate(serial_order):
                for label in self._serial_dt_labels(periph):
                    uart_lines.append(
                        '  IF_ENABLED(DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(%s)), '
                        '(if ((serial_var) == %d) { return DEVICE_DT_GET(DT_NODELABEL(%s)); }))'
                        % (label, n, label)
                    )
            lines.append(' \\\n'.join(uart_lines))
            # Count the user-facing ports only: the IOMCU port is not one.
            lines.append('#define HAL_UART_NUM_SERIAL_PORTS %d' % len(self.serial_order))
            # The SERIAL_ORDER token per port, so @SYS/uarts.txt can name the
            # hardware the way AP_HAL_ChibiOS does (OTG1, UART4, ...) instead
            # of printing a Zephyr device name like "serial@40004400". The
            # IOMCU port is included, because hal.serial() can be asked for it.
            lines.append('/* SERIAL_ORDER tokens, for @SYS/uarts.txt port names */')
            lines.append('#define HAL_UART_PORT_NAMES %s'
                         % ', '.join('"%s"' % p for p in serial_order))
            if iomcu_idx is not None:
                lines.append('#define HAL_WITH_IO_MCU 1')
                lines.append('#define HAL_UART_IOMCU_IDX %d' % iomcu_idx)
                lines.append('#define HAL_UART_IO_DRIVER '
                             'Zephyr::UARTDriver &uart_io = serial%dDriver;'
                             % iomcu_idx)
                # chibios_hwdef.py also emits HAL_HAVE_SERVO_VOLTAGE and
                # AP_FEATURE_SBUS_OUT here. Left out until this port exercises
                # them. Note that AP_BoardConfig declares the SBUS-out rate
                # parameter under HAL_BOARD_CHIBIOS but guards its use on
                # AP_FEATURE_SBUS_OUT: nothing has enabled that feature off
                # ChibiOS, so widen the declaration before setting it here.
            # WiFi virtual serial ports: WIFI_TCP/WIFI_UDP tokens map to
            # WiFiDriver/WiFiUdpDriver instances (no devicetree node) -
            # HAL_Zephyr_Class.cpp swaps them in by index.
            wifi_any = False
            for n, periph in enumerate(self.serial_order):
                if periph.upper() == 'WIFI_TCP':
                    lines.append('#define HAL_ZEPHYR_WIFI_TCP_SERIAL %d' % n)
                    wifi_any = True
                elif periph.upper() == 'WIFI_UDP':
                    lines.append('#define HAL_ZEPHYR_WIFI_UDP_SERIAL %d' % n)
                    wifi_any = True
            if wifi_any:
                lines.append('#define AP_ZEPHYR_WIFI_ENABLED 1')
            lines.append('')

        # --- I2C device list (from BARO and COMPASS directives) ---
        i2c_devices = []
        if self.baros:
            for baro in self.baros:
                if baro['conn'] == 'I2C':
                    device_type = 'I2CDevice::DeviceId::BARO'
                    i2c_devices.append(dict(name=baro['driver'].lower(), bus=baro['bus'],
                                            addr=baro['addr'], device_type=device_type, driver=baro['driver']))
        if self.compasses:
            for mag in self.compasses:
                if mag['conn'] == 'I2C':
                    device_type = 'I2CDevice::DeviceId::COMPASS'
                    i2c_devices.append(dict(name=mag['driver'].lower(), bus=mag['bus'],
                                            addr=mag['addr'], device_type=device_type, driver=mag['driver']))

        if i2c_devices:
            lines.append('/* I2C device list — generated from hwdef.dat BARO/COMPASS I2C directives */')
            lines.append('#define HAL_I2C_DEVICES_LIST \\')
            for n, i2c in enumerate(i2c_devices):
                line_end = ', \\' if n < len(i2c_devices) - 1 else ''
                lines.append('  {"%s", %s, %d, 0x%02x}%s' %
                             (i2c['name'], i2c['device_type'], i2c['bus'], i2c['addr'], line_end))
            lines.append('')

        # Freeform defines from hwdef.dat 'define' directives.
        for name, value in self.defines:
            lines.append('#define %s %s' % (name, value))
        if self.defines:
            lines.append('')

        defined_names = {n for n, _ in self.defines}

        # --- CAN interface count ---
        # chibios_hwdef.py takes CAN_ORDER when given and otherwise counts the
        # CANn peripherals named on pin directives; do the same, so a board
        # that describes its CAN pins gets working CAN without having to also
        # state the count by hand. An explicit define in hwdef.dat still wins.
        if 'HAL_NUM_CAN_IFACES' not in defined_names:
            if self.can_order:
                can_buses = [int(x) for x in self.can_order]
            else:
                can_buses = sorted(self.can_pin_buses)
            if can_buses:
                lines.append('/* CAN interfaces — from CAN_ORDER, else the CANn pin directives */')
                lines.append('#define HAL_NUM_CAN_IFACES %d' % len(can_buses))
                lines.append('')

        # --- crash dump to flash ---
        # Scaffolded the way chibios_hwdef.py scaffolds it: defined to 0 unless
        # the board asked otherwise, so @SYS/crash_dump.bin is a per-board
        # switch rather than permanently absent because the macro never exists.
        if 'AP_CRASHDUMP_FLASH_ENABLED' not in defined_names:
            lines.append('#ifndef AP_CRASHDUMP_FLASH_ENABLED')
            lines.append('#define AP_CRASHDUMP_FLASH_ENABLED 0')
            lines.append('#endif')
            lines.append('')

        # --- IMU probe list ---
        if self.imus:
            ins_macros = []
            for n, imu in enumerate(self.imus, 1):
                macro = 'HAL_INS_PROBE%d' % n
                ins_macros.append(macro)
                lines.append(
                    '#define %s ADD_BACKEND(AP_InertialSensor_%s::probe(*this,%s,%s))'
                    % (macro, imu['driver'],
                       ','.join('hal.spi->get_device("%s")' % d for d in imu['devices']),
                       imu['rotation'])
                )
            lines.append('#define HAL_INS_PROBE_LIST %s' % ';'.join(ins_macros))
            lines.append('')

        # --- Baro probe list ---
        if self.baros:
            baro_macros = []
            for n, baro in enumerate(self.baros, 1):
                macro = 'HAL_BARO_PROBE%d' % n
                baro_macros.append(macro)
                cls = _baro_class(baro['driver'])
                if baro['conn'] == 'SPI':
                    lines.append(
                        '#define %s probe_spi_dev(AP_Baro_%s::probe, "%s");'
                        ' RETURN_IF_NO_SPACE;'
                        % (macro, cls, baro['device'])
                    )
                else:
                    lines.append(
                        '#define %s probe_i2c_dev(AP_Baro_%s::probe, %d, %s);'
                        ' RETURN_IF_NO_SPACE;'
                        % (macro, cls, baro['bus'], hex(baro['addr']))
                    )
                lines.append('#undef AP_BARO_%s_ENABLED' % cls)
                lines.append('#define AP_BARO_%s_ENABLED 1' % cls)
            lines.append('#define HAL_BARO_PROBE_LIST %s' % ';'.join(baro_macros))
            lines.append('')

        # --- Compass probe list ---
        if self.compasses:
            mag_macros = []
            for n, mag in enumerate(self.compasses, 1):
                macro = 'HAL_MAG_PROBE%d' % n
                mag_macros.append(macro)
                driver = mag['driver']
                if mag['conn'] == 'PROBE_METHOD':
                    # e.g. AK09916:probe_ICM20948 → probe_ICM20948(instance, rotation)
                    lines.append(
                        '#define %s {add_backend(DRIVER_%s,'
                        'AP_Compass_%s::%s(%s,%s));RETURN_IF_NO_SPACE;}'
                        % (macro, driver, driver, mag['probe_method'],
                           mag['instance'], mag['rotation'])
                    )
                else:
                    lines.append(
                        '#define %s {add_backend(DRIVER_%s,'
                        'AP_Compass_%s::probe(GET_I2C_DEVICE(%d,%s),%s,%s));'
                        'RETURN_IF_NO_SPACE;}'
                        % (macro, driver, driver,
                           mag['bus'], hex(mag['addr']),
                           mag['external'], mag['rotation'])
                    )
                lines.append('#undef AP_COMPASS_%s_ENABLED' % driver)
                lines.append('#define AP_COMPASS_%s_ENABLED 1' % driver)
                if mag.get('probe_method', '').startswith('probe_ICM20948'):
                    lines.append('#undef AP_COMPASS_ICM20948_ENABLED')
                    lines.append('#define AP_COMPASS_ICM20948_ENABLED 1')
            lines.append('#define HAL_MAG_PROBE_LIST %s' % ';'.join(mag_macros))
            lines.append('')

        output = '\n'.join(lines) + '\n'

        # Only write if content changed to avoid triggering spurious rebuilds.
        existing = None
        if os.path.exists(output_path):
            with open(output_path, 'r') as fh:
                existing = fh.read()
        if output != existing:
            os.makedirs(os.path.dirname(output_path), exist_ok=True)
            with open(output_path, 'w') as fh:
                fh.write(output)
            print('zephyr_hwdef: wrote %s' % output_path)
        else:
            print('zephyr_hwdef: %s unchanged' % output_path)


def main():
    if len(sys.argv) != 3:
        print('Usage: zephyr_hwdef.py <hwdef.dat> <output/hwdef.h>', file=sys.stderr)
        sys.exit(1)
    hwdef = ZephyrHWDef(sys.argv[1])
    hwdef.generate_hwdef_h(sys.argv[2])


if __name__ == '__main__':
    main()
