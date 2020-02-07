#!/usr/bin/env python
'''
setup board.h for chibios
'''

import argparse
import sys
import fnmatch
import os
import dma_resolver
import shlex
import pickle
import re
import shutil

parser = argparse.ArgumentParser("chibios_pins.py")
parser.add_argument(
    '-D', '--outdir', type=str, default=None, help='Output directory')
parser.add_argument(
    '--bootloader', action='store_true', default=False, help='configure for bootloader')
parser.add_argument(
    'hwdef', type=str, default=None, help='hardware definition file')
parser.add_argument(
    '--params', type=str, default=None, help='user default params path')

args = parser.parse_args()

# output variables for each pin
f4f7_vtypes = ['MODER', 'OTYPER', 'OSPEEDR', 'PUPDR', 'ODR', 'AFRL', 'AFRH']
f1_vtypes = ['CRL', 'CRH', 'ODR']
f1_input_sigs = ['RX', 'MISO', 'CTS']
f1_output_sigs = ['TX', 'MOSI', 'SCK', 'RTS', 'CH1', 'CH2', 'CH3', 'CH4']
af_labels = ['USART', 'UART', 'SPI', 'I2C', 'SDIO', 'SDMMC', 'OTG', 'JT', 'TIM', 'CAN']

vtypes = []

# number of pins in each port
pincount = {
    'A': 16,
    'B': 16,
    'C': 16,
    'D': 16,
    'E': 16,
    'F': 16,
    'G': 16,
    'H': 2,
    'I': 0,
    'J': 0,
    'K': 0
}

ports = pincount.keys()

portmap = {}

# dictionary of all config lines, indexed by first word
config = {}

# alternate pin mappings
altmap = {}

# list of all pins in config file order
allpins = []

# list of configs by type
bytype = {}

# list of alt configs by type
alttype = {}

# list of configs by label
bylabel = {}

# list of alt configs by label
altlabel = {}

# list of SPI devices
spidev = []

# dictionary of ROMFS files
romfs = {}

# SPI bus list
spi_list = []

# all config lines in order
alllines = []

# allow for extra env vars
env_vars = {}

# build flags for ChibiOS makefiles
build_flags = []

# sensor lists
imu_list = []
compass_list = []
baro_list = []

mcu_type = None
dual_USB_enabled = False

def is_int(str):
    '''check if a string is an integer'''
    try:
        int(str)
    except Exception:
        return False
    return True


def error(str):
    '''show an error and exit'''
    print("Error: " + str)
    sys.exit(1)


def get_mcu_lib(mcu):
    '''get library file for the chosen MCU'''
    import importlib
    try:
        return importlib.import_module(mcu)
    except ImportError:
        error("Unable to find module for MCU %s" % mcu)


def setup_mcu_type_defaults():
    '''setup defaults for given mcu type'''
    global pincount, ports, portmap, vtypes, mcu_type
    lib = get_mcu_lib(mcu_type)
    if hasattr(lib, 'pincount'):
        pincount = lib.pincount
    if mcu_series.startswith("STM32F1"):
        vtypes = f1_vtypes
    else:
        vtypes = f4f7_vtypes
    ports = pincount.keys()
    # setup default as input pins
    for port in ports:
        portmap[port] = []
        for pin in range(pincount[port]):
            portmap[port].append(generic_pin(port, pin, None, 'INPUT', []))


def get_alt_function(mcu, pin, function):
    '''return alternative function number for a pin'''
    lib = get_mcu_lib(mcu)

    if function.endswith('_TXINV') or function.endswith('_RXINV'):
        # RXINV and TXINV are special labels for inversion pins, not alt-functions
        return None

    if hasattr(lib, "AltFunction_map"):
        alt_map = lib.AltFunction_map
    else:
        # just check if Alt Func is available or not
        for l in af_labels:
            if function.startswith(l):
                return 0
        return None

    if function and function.endswith("_RTS") and (
            function.startswith('USART') or function.startswith('UART')):
        # we do software RTS
        return None

    for l in af_labels:
        if function.startswith(l):
            s = pin + ":" + function
            if s not in alt_map:
                error("Unknown pin function %s for MCU %s" % (s, mcu))
            return alt_map[s]
    return None


def have_type_prefix(ptype):
    '''return True if we have a peripheral starting with the given peripheral type'''
    for t in list(bytype.keys()) + list(alttype.keys()):
        if t.startswith(ptype):
            return True
    return False


def get_ADC1_chan(mcu, pin):
    '''return ADC1 channel for an analog pin'''
    import importlib
    try:
        lib = importlib.import_module(mcu)
        ADC1_map = lib.ADC1_map
    except ImportError:
        error("Unable to find ADC1_Map for MCU %s" % mcu)

    if pin not in ADC1_map:
        error("Unable to find ADC1 channel for pin %s" % pin)
    return ADC1_map[pin]


class generic_pin(object):
    '''class to hold pin definition'''

    def __init__(self, port, pin, label, type, extra):
        global mcu_series
        self.portpin = "P%s%u" % (port, pin)
        self.port = port
        self.pin = pin
        self.label = label
        self.type = type
        self.extra = extra
        self.af = None
        if type == 'OUTPUT':
            self.sig_dir = 'OUTPUT'
        else:
            self.sig_dir = 'INPUT'
        if mcu_series.startswith("STM32F1") and self.label is not None:
            self.f1_pin_setup()

        # check that labels and pin types are consistent
        for prefix in ['USART', 'UART', 'TIM']:
            if label is None or type is None:
                continue
            if type.startswith(prefix):
                a1 = label.split('_')
                a2 = type.split('_')
                if a1[0] != a2[0]:
                    error("Peripheral prefix mismatch for %s %s %s" % (self.portpin, label, type))

    def f1_pin_setup(self):
        for label in af_labels:
            if self.label.startswith(label):
                if self.label.endswith(tuple(f1_input_sigs)):
                    self.sig_dir = 'INPUT'
                    self.extra.append('FLOATING')
                elif self.label.endswith(tuple(f1_output_sigs)):
                    self.sig_dir = 'OUTPUT'
                elif label == 'I2C':
                    self.sig_dir = 'OUTPUT'
                elif label == 'OTG':
                    self.sig_dir = 'OUTPUT'
                else:
                    error("Unknown signal type %s:%s for %s!" % (self.portpin, self.label, mcu_type))

    def has_extra(self, v):
        '''return true if we have the given extra token'''
        return v in self.extra

    def extra_prefix(self, prefix):
        '''find an extra token starting with the given prefix'''
        for e in self.extra:
            if e.startswith(prefix):
                return e
        return None

    def extra_value(self, name, type=None, default=None):
        '''find an extra value of given type'''
        v = self.extra_prefix(name)
        if v is None:
            return default
        if v[len(name)] != '(' or v[-1] != ')':
            error("Badly formed value for %s: %s\n" % (name, v))
        ret = v[len(name) + 1:-1]
        if type is not None:
            try:
                ret = type(ret)
            except Exception:
                error("Badly formed value for %s: %s\n" % (name, ret))
        return ret

    def is_RTS(self):
        '''return true if this is a RTS pin'''
        if self.label and self.label.endswith("_RTS") and (
                self.type.startswith('USART') or self.type.startswith('UART')):
            return True
        return False

    def is_CS(self):
        '''return true if this is a CS pin'''
        return self.has_extra("CS") or self.type == "CS"

    def get_MODER_value(self):
        '''return one of ALTERNATE, OUTPUT, ANALOG, INPUT'''
        if self.af is not None:
            v = "ALTERNATE"
        elif self.type == 'OUTPUT':
            v = "OUTPUT"
        elif self.type.startswith('ADC'):
            v = "ANALOG"
        elif self.is_CS():
            v = "OUTPUT"
        elif self.is_RTS():
            v = "OUTPUT"
        else:
            v = "INPUT"
        return v

    def get_MODER(self):
        '''return one of ALTERNATE, OUTPUT, ANALOG, INPUT'''
        return "PIN_MODE_%s(%uU)" % (self.get_MODER_value(), self.pin)

    def get_OTYPER_value(self):
        '''return one of PUSHPULL, OPENDRAIN'''
        v = 'PUSHPULL'
        if self.type.startswith('I2C'):
            # default I2C to OPENDRAIN
            v = 'OPENDRAIN'
        values = ['PUSHPULL', 'OPENDRAIN']
        for e in self.extra:
            if e in values:
                v = e
        return v

    def get_OTYPER(self):
        '''return one of PUSHPULL, OPENDRAIN'''
        return "PIN_OTYPE_%s(%uU)" % (self.get_OTYPER_value(), self.pin)

    def get_OSPEEDR_value(self):
        '''return one of SPEED_VERYLOW, SPEED_LOW, SPEED_MEDIUM, SPEED_HIGH'''
        # on STM32F4 these speeds correspond to 2MHz, 25MHz, 50MHz and 100MHz
        values = ['SPEED_VERYLOW', 'SPEED_LOW', 'SPEED_MEDIUM', 'SPEED_HIGH']
        v = 'SPEED_MEDIUM'
        for e in self.extra:
            if e in values:
                v = e
        return v

    def get_OSPEEDR_int(self):
        '''return value from 0 to 3 for speed'''
        values = ['SPEED_VERYLOW', 'SPEED_LOW', 'SPEED_MEDIUM', 'SPEED_HIGH']
        v = self.get_OSPEEDR_value()
        if v not in values:
            error("Bad OSPEED %s" % v)
        return values.index(v)

    def get_OSPEEDR(self):
        '''return one of SPEED_VERYLOW, SPEED_LOW, SPEED_MEDIUM, SPEED_HIGH'''
        return "PIN_O%s(%uU)" % (self.get_OSPEEDR_value(), self.pin)

    def get_PUPDR_value(self):
        '''return one of FLOATING, PULLUP, PULLDOWN'''
        values = ['FLOATING', 'PULLUP', 'PULLDOWN']
        v = 'FLOATING'
        if self.is_CS():
            v = "PULLUP"
        # generate pullups for UARTs
        if (self.type.startswith('USART') or
            self.type.startswith('UART')) and (
            (self.label.endswith('_TX') or
             self.label.endswith('_RX') or
             self.label.endswith('_CTS') or
             self.label.endswith('_RTS'))):
            v = "PULLUP"
        # generate pullups for SDIO and SDMMC
        if (self.type.startswith('SDIO') or
            self.type.startswith('SDMMC')) and (
            (self.label.endswith('_D0') or
             self.label.endswith('_D1') or
             self.label.endswith('_D2') or
             self.label.endswith('_D3') or
             self.label.endswith('_CMD'))):
            v = "PULLUP"
        for e in self.extra:
            if e in values:
                v = e
        return v

    def get_PUPDR(self):
        '''return one of FLOATING, PULLUP, PULLDOWN wrapped in PIN_PUPDR_ macro'''
        return "PIN_PUPDR_%s(%uU)" % (self.get_PUPDR_value(), self.pin)

    def get_ODR_F1_value(self):
        '''return one of LOW, HIGH'''
        values = ['LOW', 'HIGH']
        v = 'HIGH'
        if self.type == 'OUTPUT':
            v = 'LOW'
        elif self.label is not None and self.label.startswith('I2C'):
            v = 'LOW'
        for e in self.extra:
            if e in values:
                v = e
        # for some controllers input pull up down is selected by ODR
        if self.type == "INPUT":
            v = 'LOW'
            if 'PULLUP' in self.extra:
                v = "HIGH"
        return v

    def get_ODR_value(self):
        '''return one of LOW, HIGH'''
        if mcu_series.startswith("STM32F1"):
            return self.get_ODR_F1_value()
        values = ['LOW', 'HIGH']
        v = 'HIGH'
        for e in self.extra:
            if e in values:
                v = e
        return v

    def get_ODR(self):
        '''return one of LOW, HIGH wrapped in PIN_ODR macro'''
        return "PIN_ODR_%s(%uU)" % (self.get_ODR_value(), self.pin)

    def get_AFIO_value(self):
        '''return AFIO'''
        af = self.af
        if af is None:
            af = 0
        return af

    def get_AFIO(self):
        '''return AFIO wrapped in PIN_AFIO_AF macro'''
        return "PIN_AFIO_AF(%uU, %uU)" % (self.pin, self.get_AFIO_value())

    def get_AFRL(self):
        '''return AFIO low 8'''
        if self.pin >= 8:
            return None
        return self.get_AFIO()

    def get_AFRH(self):
        '''return AFIO high 8'''
        if self.pin < 8:
            return None
        return self.get_AFIO()

    def get_CR_F1(self):
        '''return CR FLAGS for STM32F1xx'''
        # Check Speed
        if self.sig_dir != "INPUT" or self.af is not None:
            speed_values = ['SPEED_LOW', 'SPEED_MEDIUM', 'SPEED_HIGH']
            v = 'SPEED_MEDIUM'
            for e in self.extra:
                if e in speed_values:
                    v = e
            speed_str = "PIN_%s(%uU) |" % (v, self.pin)
        elif self.is_CS():
            speed_str = "PIN_SPEED_LOW(%uU) |" % (self.pin)
        else:
            speed_str = ""
        if self.af is not None:
            if self.label.endswith('_RX'):
                # uart RX is configured as a input, and can be pullup, pulldown or float
                if 'PULLUP' in self.extra or 'PULLDOWN' in self.extra:
                    v = 'PUD'
                else:
                    v = "NOPULL"
            elif self.label.startswith('I2C'):
                v = "AF_OD"
            else:
                v = "AF_PP"
        elif self.is_CS():
            v = "OUTPUT_PP"
        elif self.sig_dir == 'OUTPUT':
            if 'OPENDRAIN' in self.extra:
                v = 'OUTPUT_OD'
            else:
                v = "OUTPUT_PP"
        elif self.type.startswith('ADC'):
            v = "ANALOG"
        else:
            v = "PUD"
            if 'FLOATING' in self.extra:
                v = "NOPULL"
        mode_str = "PIN_MODE_%s(%uU)" % (v, self.pin)
        return "%s %s" % (speed_str, mode_str)

    def get_CR(self):
        '''return CR FLAGS'''
        if mcu_series.startswith("STM32F1"):
            return self.get_CR_F1()
        if self.sig_dir != "INPUT":
            speed_values = ['SPEED_LOW', 'SPEED_MEDIUM', 'SPEED_HIGH']
            v = 'SPEED_MEDIUM'
            for e in self.extra:
                if e in speed_values:
                    v = e
            speed_str = "PIN_%s(%uU) |" % (v, self.pin)
        else:
            speed_str = ""
        # Check Alternate function
        if self.type.startswith('I2C'):
            v = "AF_OD"
        elif self.sig_dir == 'OUTPUT':
            if self.af is not None:
                v = "AF_PP"
            else:
                v = "OUTPUT_PP"
        elif self.type.startswith('ADC'):
            v = "ANALOG"
        elif self.is_CS():
            v = "OUTPUT_PP"
        elif self.is_RTS():
            v = "OUTPUT_PP"
        else:
            v = "PUD"
            if 'FLOATING' in self.extra:
                v = "NOPULL"
        mode_str = "PIN_MODE_%s(%uU)" % (v, self.pin)
        return "%s %s" % (speed_str, mode_str)

    def get_CRH(self):
        if self.pin < 8:
            return None
        return self.get_CR()

    def get_CRL(self):
        if self.pin >= 8:
            return None
        return self.get_CR()

    def pal_modeline(self):
        '''return a mode line suitable for palSetModeLine()'''
        # MODER, OTYPER, OSPEEDR, PUPDR, ODR, AFRL, AFRH
        ret = 'PAL_STM32_MODE_' + self.get_MODER_value()
        ret += '|PAL_STM32_OTYPE_' + self.get_OTYPER_value()
        ret += '|PAL_STM32_SPEED(%u)' % self.get_OSPEEDR_int()
        ret += '|PAL_STM32_PUPDR_' + self.get_PUPDR_value()
        af = self.get_AFIO_value()
        if af != 0:
            ret += '|PAL_STM32_ALTERNATE(%u)' % af

        return ret

    def __str__(self):
        str = ''
        if self.af is not None:
            str += " AF%u" % self.af
        if self.type.startswith('ADC1'):
            str += " ADC1_IN%u" % get_ADC1_chan(mcu_type, self.portpin)
        if self.extra_value('PWM', type=int):
            str += " PWM%u" % self.extra_value('PWM', type=int)
        return "P%s%u %s %s%s" % (self.port, self.pin, self.label, self.type,
                                  str)


def get_config(name, column=0, required=True, default=None, type=None, spaces=False):
    '''get a value from config dictionary'''
    if name not in config:
        if required and default is None:
            error("missing required value %s in hwdef.dat" % name)
        return default
    if len(config[name]) < column + 1:
        if not required:
            return None
        error("missing required value %s in hwdef.dat (column %u)" % (name,
                                                                      column))
    if spaces:
        ret = ' '.join(config[name][column:])
    else:
        ret = config[name][column]

    if type is not None:
        if type == int and ret.startswith('0x'):
            try:
                ret = int(ret, 16)
            except Exception:
                error("Badly formed config value %s (got %s)" % (name, ret))
        else:
            try:
                ret = type(ret)
            except Exception:
                error("Badly formed config value %s (got %s)" % (name, ret))
    return ret


def get_mcu_config(name, required=False):
    '''get a value from the mcu dictionary'''
    lib = get_mcu_lib(mcu_type)
    if not hasattr(lib, 'mcu'):
        error("Missing mcu config for %s" % mcu_type)
    if name not in lib.mcu:
        if required:
            error("Missing required mcu config %s for %s" % (name, mcu_type))
        return None
    return lib.mcu[name]


def make_line(label):
    '''return a line for a label'''
    if label in bylabel:
        p = bylabel[label]
        line = 'PAL_LINE(GPIO%s,%uU)' % (p.port, p.pin)
    else:
        line = "0"
    return line


def enable_can(f):
    '''setup for a CAN enabled board'''
    f.write('#define HAL_WITH_UAVCAN 1\n')
    env_vars['HAL_WITH_UAVCAN'] = '1'


def has_sdcard_spi():
    '''check for sdcard connected to spi bus'''
    for dev in spidev:
        if(dev[0] == 'sdcard'):
            return True
    return False


def write_mcu_config(f):
    '''write MCU config defines'''
    f.write('// MCU type (ChibiOS define)\n')
    f.write('#define %s_MCUCONF\n' % get_config('MCU'))
    mcu_subtype = get_config('MCU', 1)
    if mcu_subtype.endswith('xx'):
        f.write('#define %s_MCUCONF\n\n' % mcu_subtype[:-2])
    f.write('#define %s\n\n' % mcu_subtype)
    f.write('// crystal frequency\n')
    f.write('#define STM32_HSECLK %sU\n\n' % get_config('OSCILLATOR_HZ'))
    f.write('// UART used for stdout (printf)\n')
    if get_config('STDOUT_SERIAL', required=False):
        f.write('#define HAL_STDOUT_SERIAL %s\n\n' % get_config('STDOUT_SERIAL'))
        f.write('// baudrate used for stdout (printf)\n')
        f.write('#define HAL_STDOUT_BAUDRATE %u\n\n' % get_config('STDOUT_BAUDRATE', type=int))
    if have_type_prefix('SDIO'):
        f.write('// SDIO available, enable POSIX filesystem support\n')
        f.write('#define USE_POSIX\n\n')
        f.write('#define HAL_USE_SDC TRUE\n')
        build_flags.append('USE_FATFS=yes')
    elif have_type_prefix('SDMMC'):
        f.write('// SDMMC available, enable POSIX filesystem support\n')
        f.write('#define USE_POSIX\n\n')
        f.write('#define HAL_USE_SDC TRUE\n')
        f.write('#define STM32_SDC_USE_SDMMC1 TRUE\n')
        build_flags.append('USE_FATFS=yes')
    elif has_sdcard_spi():
        f.write('// MMC via SPI available, enable POSIX filesystem support\n')
        f.write('#define USE_POSIX\n\n')
        f.write('#define HAL_USE_MMC_SPI TRUE\n')
        f.write('#define HAL_USE_SDC FALSE\n')
        f.write('#define HAL_SDCARD_SPI_HOOK TRUE\n')
        build_flags.append('USE_FATFS=yes')
    else:
        f.write('#define HAL_USE_SDC FALSE\n')
        build_flags.append('USE_FATFS=no')
        env_vars['DISABLE_SCRIPTING'] = True
    if 'OTG1' in bytype:
        f.write('#define STM32_USB_USE_OTG1                  TRUE\n')
        f.write('#define HAL_USE_USB TRUE\n')
        f.write('#define HAL_USE_SERIAL_USB TRUE\n')
    if 'OTG2' in bytype:
        f.write('#define STM32_USB_USE_OTG2                  TRUE\n')
    if have_type_prefix('CAN') and 'AP_PERIPH' not in env_vars:
        enable_can(f)

    if get_config('PROCESS_STACK', required=False):
        env_vars['PROCESS_STACK'] = get_config('PROCESS_STACK')
    else:
        env_vars['PROCESS_STACK'] = "0x2000"

    if get_config('MAIN_STACK', required=False):
        env_vars['MAIN_STACK'] = get_config('MAIN_STACK')
    else:
        env_vars['MAIN_STACK'] = "0x400"

    if get_config('IOMCU_FW', required=False):
        env_vars['IOMCU_FW'] = get_config('IOMCU_FW')
    else:
        env_vars['IOMCU_FW'] = 0

    if get_config('PERIPH_FW', required=False):
        env_vars['PERIPH_FW'] = get_config('PERIPH_FW')
    else:
        env_vars['PERIPH_FW'] = 0

    # write any custom STM32 defines
    for d in alllines:
        if d.startswith('STM32_'):
            f.write('#define %s\n' % d)
        if d.startswith('define '):
            f.write('#define %s\n' % d[7:])
    flash_size = get_config('FLASH_SIZE_KB', type=int)
    f.write('#define BOARD_FLASH_SIZE %u\n' % flash_size)
    env_vars['BOARD_FLASH_SIZE'] = flash_size
    f.write('#define CRT1_AREAS_NUMBER 1\n')

    flash_reserve_start = get_config(
        'FLASH_RESERVE_START_KB', default=16, type=int)
    f.write('\n// location of loaded firmware\n')
    f.write('#define FLASH_LOAD_ADDRESS 0x%08x\n' % (0x08000000 + flash_reserve_start*1024))
    if args.bootloader:
        f.write('#define FLASH_BOOTLOADER_LOAD_KB %u\n' % get_config('FLASH_BOOTLOADER_LOAD_KB', type=int))
    f.write('\n')

    ram_map = get_mcu_config('RAM_MAP', True)
    f.write('// memory regions\n')
    regions = []
    total_memory = 0
    for (address, size, flags) in ram_map:
        regions.append('{(void*)0x%08x, 0x%08x, 0x%02x }' % (address, size*1024, flags))
        total_memory += size
    f.write('#define HAL_MEMORY_REGIONS %s\n' % ', '.join(regions))
    f.write('#define HAL_MEMORY_TOTAL_KB %u\n' % total_memory)

    f.write('#define HAL_RAM0_START 0x%08x\n' % ram_map[0][0])
    ram_reserve_start = get_config('RAM_RESERVE_START', default=0, type=int)
    if ram_reserve_start > 0:
        f.write('#define HAL_RAM_RESERVE_START 0x%08x\n' % ram_reserve_start)

    f.write('\n// CPU serial number (12 bytes)\n')
    f.write('#define UDID_START 0x%08x\n\n' % get_mcu_config('UDID_START', True))

    f.write('\n// APJ board ID (for bootloaders)\n')
    f.write('#define APJ_BOARD_ID %s\n' % get_config('APJ_BOARD_ID'))

    lib = get_mcu_lib(mcu_type)
    build_info = lib.build

    if get_mcu_config('CPU_FLAGS') and get_mcu_config('CORTEX'):
        # CPU flags specified in mcu file
        cortex = get_mcu_config('CORTEX')
        env_vars['CPU_FLAGS'] = get_mcu_config('CPU_FLAGS').split()
        build_info['MCU'] = cortex
        print("MCU Flags: %s %s" % (cortex, env_vars['CPU_FLAGS']))
    elif mcu_series.startswith("STM32F1"):
        cortex = "cortex-m3"
        env_vars['CPU_FLAGS'] = ["-mcpu=%s" % cortex]
        build_info['MCU'] = cortex
    else:
        # default to F4
        cortex = "cortex-m4"
        env_vars['CPU_FLAGS'] = ["-mcpu=%s" % cortex, "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"]
        build_info['MCU'] = cortex

    if not mcu_series.startswith("STM32F1") and not args.bootloader:
        env_vars['CPU_FLAGS'].append('-u_printf_float')
        build_info['ENV_UDEFS'] = "-DCHPRINTF_USE_FLOAT=1"

    # setup build variables
    for v in build_info.keys():
        build_flags.append('%s=%s' % (v, build_info[v]))

    # setup for bootloader build
    if args.bootloader:
        f.write('''
#define HAL_BOOTLOADER_BUILD TRUE
#define HAL_USE_ADC FALSE
#define HAL_USE_EXT FALSE
#define HAL_NO_UARTDRIVER
#define HAL_NO_PRINTF
#define HAL_NO_CCM
#define CH_DBG_STATISTICS FALSE
#define CH_CFG_USE_TM FALSE
#define CH_CFG_USE_REGISTRY FALSE
#define CH_CFG_USE_WAITEXIT FALSE
#define CH_CFG_USE_DYNAMIC FALSE
#define CH_CFG_USE_MEMPOOLS FALSE
#define CH_CFG_USE_OBJ_FIFOS FALSE
#define CH_DBG_FILL_THREADS FALSE
#define CH_CFG_USE_SEMAPHORES FALSE
#define CH_CFG_USE_HEAP FALSE
#define CH_CFG_USE_MUTEXES FALSE
#define CH_CFG_USE_CONDVARS FALSE
#define CH_CFG_USE_CONDVARS_TIMEOUT FALSE
#define CH_CFG_USE_EVENTS FALSE
#define CH_CFG_USE_EVENTS_TIMEOUT FALSE
#define CH_CFG_USE_MESSAGES FALSE
#define CH_CFG_USE_MAILBOXES FALSE
#define CH_CFG_USE_FACTORY FALSE
#define CH_CFG_USE_MEMCORE FALSE
#define HAL_USE_I2C FALSE
#define HAL_USE_PWM FALSE
''')
    if env_vars.get('ROMFS_UNCOMPRESSED', False):
        f.write('#define HAL_ROMFS_UNCOMPRESSED\n')


def write_ldscript(fname):
    '''write ldscript.ld for this board'''
    flash_size = get_config('FLASH_USE_MAX_KB', type=int, default=0)
    if flash_size == 0:
        flash_size = get_config('FLASH_SIZE_KB', type=int)

    # space to reserve for bootloader and storage at start of flash
    flash_reserve_start = get_config(
        'FLASH_RESERVE_START_KB', default=16, type=int)

    env_vars['FLASH_RESERVE_START_KB'] = str(flash_reserve_start)

    # space to reserve for storage at end of flash
    flash_reserve_end = get_config('FLASH_RESERVE_END_KB', default=0, type=int)

    # ram layout
    ram_map = get_mcu_config('RAM_MAP', True)

    flash_base = 0x08000000 + flash_reserve_start * 1024

    if not args.bootloader:
        flash_length = flash_size - (flash_reserve_start + flash_reserve_end)
    else:
        flash_length = get_config('FLASH_BOOTLOADER_LOAD_KB', type=int)

    print("Generating ldscript.ld")
    f = open(fname, 'w')
    ram0_start = ram_map[0][0]
    ram0_len = ram_map[0][1] * 1024

    # possibly reserve some memory for app/bootloader comms
    ram_reserve_start = get_config('RAM_RESERVE_START', default=0, type=int)
    ram0_start += ram_reserve_start
    ram0_len -= ram_reserve_start

    f.write('''/* generated ldscript.ld */
MEMORY
{
    flash : org = 0x%08x, len = %uK
    ram0  : org = 0x%08x, len = %u
}

INCLUDE common.ld
''' % (flash_base, flash_length, ram0_start, ram0_len))


def copy_common_linkerscript(outdir, hwdef):
    dirpath = os.path.dirname(hwdef)
    shutil.copy(os.path.join(dirpath, "../common/common.ld"),
                os.path.join(outdir, "common.ld"))

def get_USB_IDs():
    '''return tuple of USB VID/PID'''

    global dual_USB_enabled
    if dual_USB_enabled:
        # use pidcodes allocated ID
        default_vid = 0x1209
        default_pid = 0x5740
    else:
        default_vid = 0x1209
        default_pid = 0x5741
    return (get_config('USB_VENDOR', type=int, default=default_vid), get_config('USB_PRODUCT', type=int, default=default_pid))

def write_USB_config(f):
    '''write USB config defines'''
    if not have_type_prefix('OTG'):
        return
    f.write('// USB configuration\n')
    (USB_VID, USB_PID) = get_USB_IDs()
    f.write('#define HAL_USB_VENDOR_ID 0x%04x\n' % int(USB_VID))
    f.write('#define HAL_USB_PRODUCT_ID 0x%04x\n' % int(USB_PID))
    f.write('#define HAL_USB_STRING_MANUFACTURER "%s"\n' % get_config("USB_STRING_MANUFACTURER", default="ArduPilot"))
    default_product = "%BOARD%"
    if args.bootloader:
        default_product += "-BL"
    f.write('#define HAL_USB_STRING_PRODUCT "%s"\n' % get_config("USB_STRING_PRODUCT", default=default_product))
    f.write('#define HAL_USB_STRING_SERIAL "%s"\n' % get_config("USB_STRING_SERIAL", default="%SERIAL%"))

    f.write('\n\n')


def write_SPI_table(f):
    '''write SPI device table'''
    f.write('\n// SPI device table\n')
    devlist = []
    for dev in spidev:
        if len(dev) != 7:
            print("Badly formed SPIDEV line %s" % dev)
        name = '"' + dev[0] + '"'
        bus = dev[1]
        devid = dev[2]
        cs = dev[3]
        mode = dev[4]
        lowspeed = dev[5]
        highspeed = dev[6]
        if not bus.startswith('SPI') or bus not in spi_list:
            error("Bad SPI bus in SPIDEV line %s" % dev)
        if not devid.startswith('DEVID') or not is_int(devid[5:]):
            error("Bad DEVID in SPIDEV line %s" % dev)
        if cs not in bylabel or not bylabel[cs].is_CS():
            error("Bad CS pin in SPIDEV line %s" % dev)
        if mode not in ['MODE0', 'MODE1', 'MODE2', 'MODE3']:
            error("Bad MODE in SPIDEV line %s" % dev)
        if not lowspeed.endswith('*MHZ') and not lowspeed.endswith('*KHZ'):
            error("Bad lowspeed value %s in SPIDEV line %s" % (lowspeed, dev))
        if not highspeed.endswith('*MHZ') and not highspeed.endswith('*KHZ'):
            error("Bad highspeed value %s in SPIDEV line %s" % (highspeed,
                                                                dev))
        cs_pin = bylabel[cs]
        pal_line = 'PAL_LINE(GPIO%s,%uU)' % (cs_pin.port, cs_pin.pin)
        devidx = len(devlist)
        f.write(
            '#define HAL_SPI_DEVICE%-2u SPIDesc(%-17s, %2u, %2u, %-19s, SPIDEV_%s, %7s, %7s)\n'
            % (devidx, name, spi_list.index(bus), int(devid[5:]), pal_line,
               mode, lowspeed, highspeed))
        devlist.append('HAL_SPI_DEVICE%u' % devidx)
    f.write('#define HAL_SPI_DEVICE_LIST %s\n\n' % ','.join(devlist))


def write_SPI_config(f):
    '''write SPI config defines'''
    global spi_list
    for t in list(bytype.keys()) + list(alttype.keys()):
        if t.startswith('SPI'):
            spi_list.append(t)
    spi_list = sorted(spi_list)
    if len(spi_list) == 0:
        f.write('#define HAL_USE_SPI FALSE\n')
        return
    devlist = []
    for dev in spi_list:
        n = int(dev[3:])
        devlist.append('HAL_SPI%u_CONFIG' % n)
        f.write(
            '#define HAL_SPI%u_CONFIG { &SPID%u, %u, STM32_SPI_SPI%u_DMA_STREAMS }\n'
            % (n, n, n, n))
    f.write('#define HAL_SPI_BUS_LIST %s\n\n' % ','.join(devlist))
    write_SPI_table(f)


def parse_spi_device(dev):
    '''parse a SPI:xxx device item'''
    a = dev.split(':')
    if len(a) != 2:
        error("Bad SPI device: %s" % dev)
    return 'hal.spi->get_device("%s")' % a[1]


def parse_i2c_device(dev):
    '''parse a I2C:xxx:xxx device item'''
    a = dev.split(':')
    if len(a) != 3:
        error("Bad I2C device: %s" % dev)
    busaddr = int(a[2], base=0)
    if a[1] == 'ALL_EXTERNAL':
        return ('FOREACH_I2C_EXTERNAL(b)', 'GET_I2C_DEVICE(b,0x%02x)' % (busaddr))
    elif a[1] == 'ALL_INTERNAL':
        return ('FOREACH_I2C_INTERNAL(b)', 'GET_I2C_DEVICE(b,0x%02x)' % (busaddr))
    elif a[1] == 'ALL':
        return ('FOREACH_I2C(b)', 'GET_I2C_DEVICE(b,0x%02x)' % (busaddr))
    busnum = int(a[1])
    return ('', 'GET_I2C_DEVICE(%u,0x%02x)' % (busnum, busaddr))


def seen_str(dev):
    '''return string representation of device for checking for duplicates'''
    return str(dev[:2])


def write_IMU_config(f):
    '''write IMU config defines'''
    global imu_list
    devlist = []
    wrapper = ''
    seen = set()
    for dev in imu_list:
        if seen_str(dev) in seen:
            error("Duplicate IMU: %s" % seen_str(dev))
        seen.add(seen_str(dev))
        driver = dev[0]
        for i in range(1, len(dev)):
            if dev[i].startswith("SPI:"):
                dev[i] = parse_spi_device(dev[i])
            elif dev[i].startswith("I2C:"):
                (wrapper, dev[i]) = parse_i2c_device(dev[i])
        n = len(devlist)+1
        devlist.append('HAL_INS_PROBE%u' % n)
        f.write(
            '#define HAL_INS_PROBE%u %s ADD_BACKEND(AP_InertialSensor_%s::probe(*this,%s))\n'
            % (n, wrapper, driver, ','.join(dev[1:])))
    if len(devlist) > 0:
        f.write('#define HAL_INS_PROBE_LIST %s\n\n' % ';'.join(devlist))


def write_MAG_config(f):
    '''write MAG config defines'''
    global compass_list
    devlist = []
    seen = set()
    for dev in compass_list:
        if seen_str(dev) in seen:
            error("Duplicate MAG: %s" % seen_str(dev))
        seen.add(seen_str(dev))
        driver = dev[0]
        probe = 'probe'
        wrapper = ''
        a = driver.split(':')
        driver = a[0]
        if len(a) > 1 and a[1].startswith('probe'):
            probe = a[1]
        for i in range(1, len(dev)):
            if dev[i].startswith("SPI:"):
                dev[i] = parse_spi_device(dev[i])
            elif dev[i].startswith("I2C:"):
                (wrapper, dev[i]) = parse_i2c_device(dev[i])
        n = len(devlist)+1
        devlist.append('HAL_MAG_PROBE%u' % n)
        f.write(
            '#define HAL_MAG_PROBE%u %s ADD_BACKEND(DRIVER_%s, AP_Compass_%s::%s(%s))\n'
            % (n, wrapper, driver, driver, probe, ','.join(dev[1:])))
    if len(devlist) > 0:
        f.write('#define HAL_MAG_PROBE_LIST %s\n\n' % ';'.join(devlist))


def write_BARO_config(f):
    '''write barometer config defines'''
    global baro_list
    devlist = []
    seen = set()
    for dev in baro_list:
        if seen_str(dev) in seen:
            error("Duplicate BARO: %s" % seen_str(dev))
        seen.add(seen_str(dev))
        driver = dev[0]
        probe = 'probe'
        wrapper = ''
        a = driver.split(':')
        driver = a[0]
        if len(a) > 1 and a[1].startswith('probe'):
            probe = a[1]
        for i in range(1, len(dev)):
            if dev[i].startswith("SPI:"):
                dev[i] = parse_spi_device(dev[i])
            elif dev[i].startswith("I2C:"):
                (wrapper, dev[i]) = parse_i2c_device(dev[i])
                if dev[i].startswith('hal.i2c_mgr'):
                    dev[i] = 'std::move(%s)' % dev[i]
        n = len(devlist)+1
        devlist.append('HAL_BARO_PROBE%u' % n)
        args = ['*this'] + dev[1:]
        f.write(
            '#define HAL_BARO_PROBE%u %s ADD_BACKEND(AP_Baro_%s::%s(%s))\n'
            % (n, wrapper, driver, probe, ','.join(args)))
    if len(devlist) > 0:
        f.write('#define HAL_BARO_PROBE_LIST %s\n\n' % ';'.join(devlist))


def get_gpio_bylabel(label):
    '''get GPIO(n) setting on a pin label, or -1'''
    p = bylabel.get(label)
    if p is None:
        return -1
    return p.extra_value('GPIO', type=int, default=-1)


def get_extra_bylabel(label, name, default=None):
    '''get extra setting for a label by name'''
    p = bylabel.get(label)
    if p is None:
        return default
    return p.extra_value(name, type=str, default=default)


def write_UART_config(f):
    '''write UART config defines'''
    global dual_USB_enabled
    if get_config('UART_ORDER', required=False) is None:
        return
    uart_list = config['UART_ORDER']
    f.write('\n// UART configuration\n')

    # write out driver declarations for HAL_ChibOS_Class.cpp
    devnames = "ABCDEFGH"
    sdev = 0
    idx = 0
    num_empty_uarts = 0
    for dev in uart_list:
        if dev == 'EMPTY':
            f.write('#define HAL_UART%s_DRIVER Empty::UARTDriver uart%sDriver\n' %
                    (devnames[idx], devnames[idx]))
            num_empty_uarts += 1
        else:
            f.write(
                '#define HAL_UART%s_DRIVER ChibiOS::UARTDriver uart%sDriver(%u)\n'
                % (devnames[idx], devnames[idx], sdev))
            sdev += 1
        idx += 1
    for idx in range(len(uart_list), len(devnames)):
        f.write('#define HAL_UART%s_DRIVER Empty::UARTDriver uart%sDriver\n' %
                (devnames[idx], devnames[idx]))

    if 'IOMCU_UART' in config:
        f.write('#define HAL_WITH_IO_MCU 1\n')
        idx = len(uart_list)
        f.write('#define HAL_UART_IOMCU_IDX %u\n' % idx)
        f.write(
            '#define HAL_UART_IO_DRIVER ChibiOS::UARTDriver uart_io(HAL_UART_IOMCU_IDX)\n'
        )
        uart_list.append(config['IOMCU_UART'][0])
        f.write('#define HAL_HAVE_SERVO_VOLTAGE 1\n') # make the assumption that IO gurantees servo monitoring
        # all IOMCU capable boards have SBUS out
        f.write('#define AP_FEATURE_SBUS_OUT 1\n')
    else:
        f.write('#define HAL_WITH_IO_MCU 0\n')
    f.write('\n')

    need_uart_driver = False
    OTG2_index = None
    devlist = []
    have_rts_cts = False
    for dev in uart_list:
        if dev.startswith('UART'):
            n = int(dev[4:])
        elif dev.startswith('USART'):
            n = int(dev[5:])
        elif dev.startswith('OTG'):
            n = int(dev[3:])
        elif dev.startswith('EMPTY'):
            continue
        else:
            error("Invalid element %s in UART_ORDER" % dev)
        devlist.append('HAL_%s_CONFIG' % dev)
        tx_line = make_line(dev + '_TX')
        rx_line = make_line(dev + '_RX')
        rts_line = make_line(dev + '_RTS')
        if rts_line != "0":
            have_rts_cts = True
        if dev.startswith('OTG2'):
            f.write(
                '#define HAL_%s_CONFIG {(BaseSequentialStream*) &SDU2, true, false, 0, 0, false, 0, 0}\n'
                % dev)
            OTG2_index = uart_list.index(dev)
            dual_USB_enabled = True
        elif dev.startswith('OTG'):
            f.write(
                '#define HAL_%s_CONFIG {(BaseSequentialStream*) &SDU1, true, false, 0, 0, false, 0, 0}\n'
                % dev)
        else:
            need_uart_driver = True
            f.write(
                "#define HAL_%s_CONFIG { (BaseSequentialStream*) &SD%u, false, "
                % (dev, n))
            if mcu_series.startswith("STM32F1"):
                f.write("%s, %s, %s, " % (tx_line, rx_line, rts_line))
            else:
                f.write("STM32_%s_RX_DMA_CONFIG, STM32_%s_TX_DMA_CONFIG, %s, %s, %s, " %
                        (dev, dev, tx_line, rx_line, rts_line))

            # add inversion pins, if any
            f.write("%d, " % get_gpio_bylabel(dev + "_RXINV"))
            f.write("%s, " % get_extra_bylabel(dev + "_RXINV", "POL", "0"))
            f.write("%d, " % get_gpio_bylabel(dev + "_TXINV"))
            f.write("%s}\n" % get_extra_bylabel(dev + "_TXINV", "POL", "0"))
    if have_rts_cts:
        f.write('#define AP_FEATURE_RTSCTS 1\n')
    if OTG2_index is not None:
        f.write('#define HAL_OTG2_UART_INDEX %d\n' % OTG2_index)
        f.write('''
#if HAL_WITH_UAVCAN
#ifndef HAL_OTG2_PROTOCOL
#define HAL_OTG2_PROTOCOL SerialProtocol_SLCAN
#endif
#define HAL_SERIAL%d_PROTOCOL HAL_OTG2_PROTOCOL
#define HAL_SERIAL%d_BAUD 115200
#endif
''' % (OTG2_index, OTG2_index))
        f.write('#define HAL_HAVE_DUAL_USB_CDC 1\n')

    f.write('#define HAL_UART_DEVICE_LIST %s\n\n' % ','.join(devlist))
    if not need_uart_driver and not args.bootloader:
        f.write('''
#ifndef HAL_USE_SERIAL
#define HAL_USE_SERIAL HAL_USE_SERIAL_USB
#endif
''')
    num_uarts = len(devlist)
    if 'IOMCU_UART' in config:
        num_uarts -= 1
    if num_uarts > 8:
        error("Exceeded max num UARTs of 8 (%u)" % num_uarts)
    f.write('#define HAL_UART_NUM_SERIAL_PORTS %u\n' % (num_uarts+num_empty_uarts))


def write_UART_config_bootloader(f):
    '''write UART config defines'''
    if get_config('UART_ORDER', required=False) is None:
        return
    uart_list = config['UART_ORDER']
    f.write('\n// UART configuration\n')
    devlist = []
    have_uart = False
    OTG2_index = None
    for u in uart_list:
        if u.startswith('OTG2'):
            devlist.append('(BaseChannel *)&SDU2')
            OTG2_index = uart_list.index(u)
        elif u.startswith('OTG'):
            devlist.append('(BaseChannel *)&SDU1')
        else:
            unum = int(u[-1])
            devlist.append('(BaseChannel *)&SD%u' % unum)
            have_uart = True
    f.write('#define BOOTLOADER_DEV_LIST %s\n' % ','.join(devlist))
    if OTG2_index is not None:
        f.write('#define HAL_OTG2_UART_INDEX %d\n' % OTG2_index)
    if not have_uart:
        f.write('''
#ifndef HAL_USE_SERIAL
#define HAL_USE_SERIAL FALSE
#endif
''')


def write_I2C_config(f):
    '''write I2C config defines'''
    if not have_type_prefix('I2C'):
        print("No I2C peripherals")
        f.write('''
#ifndef HAL_USE_I2C
#define HAL_USE_I2C FALSE
#endif
''')
        return
    if 'I2C_ORDER' not in config:
        print("Missing I2C_ORDER config")
        return
    i2c_list = config['I2C_ORDER']
    f.write('// I2C configuration\n')
    if len(i2c_list) == 0:
        error("I2C_ORDER invalid")
    devlist = []

    # write out config structures
    for dev in i2c_list:
        if not dev.startswith('I2C') or dev[3] not in "1234":
            error("Bad I2C_ORDER element %s" % dev)
        n = int(dev[3:])
        devlist.append('HAL_I2C%u_CONFIG' % n)
        sda_line = make_line('I2C%u_SDA' % n)
        scl_line = make_line('I2C%u_SCL' % n)
        f.write('''
#if defined(STM32_I2C_I2C%u_RX_DMA_STREAM) && defined(STM32_I2C_I2C%u_TX_DMA_STREAM)
#define HAL_I2C%u_CONFIG { &I2CD%u, STM32_I2C_I2C%u_RX_DMA_STREAM, STM32_I2C_I2C%u_TX_DMA_STREAM, %s, %s }
#else
#define HAL_I2C%u_CONFIG { &I2CD%u, SHARED_DMA_NONE, SHARED_DMA_NONE, %s, %s }
#endif
'''
                % (n, n, n, n, n, n, scl_line, sda_line, n, n, scl_line, sda_line))
    f.write('\n#define HAL_I2C_DEVICE_LIST %s\n\n' % ','.join(devlist))


def parse_timer(str):
    '''parse timer channel string, i.e TIM8_CH2N'''
    result = re.match(r'TIM([0-9]*)_CH([1234])(N?)', str)
    if result:
        tim = int(result.group(1))
        chan = int(result.group(2))
        compl = result.group(3) == 'N'
        if tim < 1 or tim > 17:
            error("Bad timer number %s in %s" % (tim, str))
        return (tim, chan, compl)
    else:
        error("Bad timer definition %s" % str)


def write_PWM_config(f):
    '''write PWM config defines'''
    rc_in = None
    rc_in_int = None
    alarm = None
    pwm_out = []
    pwm_timers = []
    for l in bylabel.keys():
        p = bylabel[l]
        if p.type.startswith('TIM'):
            if p.has_extra('RCIN'):
                rc_in = p
            elif p.has_extra('RCININT'):
                rc_in_int = p
            elif p.has_extra('ALARM'):
                alarm = p
            else:
                if p.extra_value('PWM', type=int) is not None:
                    pwm_out.append(p)
                if p.type not in pwm_timers:
                    pwm_timers.append(p.type)

    if not pwm_out and not alarm:
        print("No PWM output defined")
        f.write('''
#ifndef HAL_USE_PWM
#define HAL_USE_PWM FALSE
#endif
''')

    if rc_in is not None:
        (n, chan, compl) = parse_timer(rc_in.label)
        if compl:
            # it is an inverted channel
            f.write('#define HAL_RCIN_IS_INVERTED\n')
        if chan not in [1, 2]:
            error(
                "Bad channel number, only channel 1 and 2 supported for RCIN")
        f.write('// RC input config\n')
        f.write('#define HAL_USE_ICU TRUE\n')
        f.write('#define STM32_ICU_USE_TIM%u TRUE\n' % n)
        f.write('#define RCIN_ICU_TIMER ICUD%u\n' % n)
        f.write('#define RCIN_ICU_CHANNEL ICU_CHANNEL_%u\n' % chan)
        f.write('#define STM32_RCIN_DMA_STREAM STM32_TIM_TIM%u_CH%u_DMA_STREAM\n' % (n, chan))
        f.write('#define STM32_RCIN_DMA_CHANNEL STM32_TIM_TIM%u_CH%u_DMA_CHAN\n' % (n, chan))
        f.write('\n')

    if rc_in_int is not None:
        (n, chan, compl) = parse_timer(rc_in_int.label)
        if compl:
            error('Complementary channel is not supported for RCININT %s' % rc_in_int.label)
        f.write('// RC input config\n')
        f.write('#define HAL_USE_EICU TRUE\n')
        f.write('#define STM32_EICU_USE_TIM%u TRUE\n' % n)
        f.write('#define RCININT_EICU_TIMER EICUD%u\n' % n)
        f.write('#define RCININT_EICU_CHANNEL EICU_CHANNEL_%u\n' % chan)
        f.write('\n')

    if alarm is not None:
        (n, chan, compl) = parse_timer(alarm.label)
        if compl:
            error("Complementary channel is not supported for ALARM %s" % alarm.label)
        f.write('\n')
        f.write('// Alarm PWM output config\n')
        f.write('#define STM32_PWM_USE_TIM%u TRUE\n' % n)
        f.write('#define STM32_TIM%u_SUPPRESS_ISR\n' % n)

        chan_mode = [
            'PWM_OUTPUT_DISABLED', 'PWM_OUTPUT_DISABLED',
            'PWM_OUTPUT_DISABLED', 'PWM_OUTPUT_DISABLED'
        ]
        chan_mode[chan - 1] = 'PWM_OUTPUT_ACTIVE_HIGH'

        pwm_clock = 1000000
        period = 1000

        f.write('''#define HAL_PWM_ALARM \\
        { /* pwmGroup */ \\
          %u,  /* Timer channel */ \\
          { /* PWMConfig */ \\
            %u,    /* PWM clock frequency. */ \\
            %u,    /* Initial PWM period 20ms. */ \\
            NULL,  /* no callback */ \\
            { /* Channel Config */ \\
             {%s, NULL}, \\
             {%s, NULL}, \\
             {%s, NULL}, \\
             {%s, NULL}  \\
            }, \\
            0, 0 \\
          }, \\
          &PWMD%u /* PWMDriver* */ \\
        }\n''' %
                (chan-1, pwm_clock, period, chan_mode[0],
                 chan_mode[1], chan_mode[2], chan_mode[3], n))
    else:
        f.write('\n')
        f.write('// No Alarm output pin defined\n')
        f.write('#undef HAL_PWM_ALARM\n')
    f.write('\n')

    f.write('// PWM timer config\n')
    for t in sorted(pwm_timers):
        n = int(t[3:])
        f.write('#define STM32_PWM_USE_TIM%u TRUE\n' % n)
        f.write('#define STM32_TIM%u_SUPPRESS_ISR\n' % n)
    f.write('\n')
    f.write('// PWM output config\n')
    groups = []
    have_complementary = False
    for t in sorted(pwm_timers):
        group = len(groups) + 1
        n = int(t[3:])
        chan_list = [255, 255, 255, 255]
        chan_mode = [
            'PWM_OUTPUT_DISABLED', 'PWM_OUTPUT_DISABLED',
            'PWM_OUTPUT_DISABLED', 'PWM_OUTPUT_DISABLED'
        ]
        alt_functions = [0, 0, 0, 0]
        pal_lines = ['0', '0', '0', '0']
        for p in pwm_out:
            if p.type != t:
                continue
            (n, chan, compl) = parse_timer(p.label)
            pwm = p.extra_value('PWM', type=int)
            chan_list[chan - 1] = pwm - 1
            if compl:
                chan_mode[chan - 1] = 'PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH'
                have_complementary = True
            else:
                chan_mode[chan - 1] = 'PWM_OUTPUT_ACTIVE_HIGH'
            alt_functions[chan - 1] = p.af
            pal_lines[chan - 1] = 'PAL_LINE(GPIO%s, %uU)' % (p.port, p.pin)
        groups.append('HAL_PWM_GROUP%u' % group)
        if n in [1, 8]:
            # only the advanced timers do 8MHz clocks
            advanced_timer = 'true'
        else:
            advanced_timer = 'false'
        pwm_clock = 1000000
        period = 20000 * pwm_clock / 1000000
        f.write('''#if defined(STM32_TIM_TIM%u_UP_DMA_STREAM) && defined(STM32_TIM_TIM%u_UP_DMA_CHAN)
# define HAL_PWM%u_DMA_CONFIG true, STM32_TIM_TIM%u_UP_DMA_STREAM, STM32_TIM_TIM%u_UP_DMA_CHAN
#else
# define HAL_PWM%u_DMA_CONFIG false, 0, 0
#endif\n''' % (n, n, n, n, n, n))
        f.write('''#define HAL_PWM_GROUP%u { %s, \\
        {%u, %u, %u, %u}, \\
        /* Group Initial Config */ \\
        { \\
          %u,  /* PWM clock frequency. */ \\
          %u,   /* Initial PWM period 20ms. */ \\
          NULL,     /* no callback */ \\
          { \\
           /* Channel Config */ \\
           {%s, NULL}, \\
           {%s, NULL}, \\
           {%s, NULL}, \\
           {%s, NULL}  \\
          }, 0, 0}, &PWMD%u, \\
          HAL_PWM%u_DMA_CONFIG, \\
          { %u, %u, %u, %u }, \\
          { %s, %s, %s, %s }}\n''' %
                (group, advanced_timer,
                 chan_list[0], chan_list[1], chan_list[2], chan_list[3],
                 pwm_clock, period,
                 chan_mode[0], chan_mode[1], chan_mode[2], chan_mode[3],
                 n, n,
                 alt_functions[0], alt_functions[1], alt_functions[2], alt_functions[3],
                 pal_lines[0], pal_lines[1], pal_lines[2], pal_lines[3]))
    f.write('#define HAL_PWM_GROUPS %s\n\n' % ','.join(groups))
    if have_complementary:
        f.write('#define STM32_PWM_USE_ADVANCED TRUE\n')


def write_ADC_config(f):
    '''write ADC config defines'''
    f.write('// ADC config\n')
    adc_chans = []
    for l in bylabel:
        p = bylabel[l]
        if not p.type.startswith('ADC'):
            continue
        chan = get_ADC1_chan(mcu_type, p.portpin)
        scale = p.extra_value('SCALE', default=None)
        if p.label == 'VDD_5V_SENS':
            f.write('#define ANALOG_VCC_5V_PIN %u\n' % chan)
            f.write('#define HAL_HAVE_BOARD_VOLTAGE 1\n')
        if p.label == 'FMU_SERVORAIL_VCC_SENS':
            f.write('#define FMU_SERVORAIL_ADC_CHAN %u\n' % chan)
            f.write('#define HAL_HAVE_SERVO_VOLTAGE 1\n')
        adc_chans.append((chan, scale, p.label, p.portpin))
    adc_chans = sorted(adc_chans)
    vdd = get_config('STM32_VDD')
    if vdd[-1] == 'U':
        vdd = vdd[:-1]
    vdd = float(vdd) * 0.01
    f.write('#define HAL_ANALOG_PINS { \\\n')
    for (chan, scale, label, portpin) in adc_chans:
        scale_str = '%.2f/4096' % vdd
        if scale is not None and scale != '1':
            scale_str = scale + '*' + scale_str
        f.write('{ %2u, %12s }, /* %s %s */ \\\n' % (chan, scale_str, portpin,
                                                     label))
    f.write('}\n\n')


def write_GPIO_config(f):
    '''write GPIO config defines'''
    f.write('// GPIO config\n')
    gpios = []
    gpioset = set()
    for l in bylabel:
        p = bylabel[l]
        gpio = p.extra_value('GPIO', type=int)
        if gpio is None:
            continue
        if gpio in gpioset:
            error("Duplicate GPIO value %u" % gpio)
        gpioset.add(gpio)
        # see if it is also a PWM pin
        pwm = p.extra_value('PWM', type=int, default=0)
        port = p.port
        pin = p.pin
        gpios.append((gpio, pwm, port, pin, p))
    gpios = sorted(gpios)
    for (gpio, pwm, port, pin, p) in gpios:
        f.write('#define HAL_GPIO_LINE_GPIO%u PAL_LINE(GPIO%s, %2uU)\n' % (gpio, port, pin))
    f.write('#define HAL_GPIO_PINS { \\\n')
    for (gpio, pwm, port, pin, p) in gpios:
        f.write('{ %3u, true, %2u, PAL_LINE(GPIO%s, %2uU)}, /* %s */ \\\n' %
                (gpio, pwm, port, pin, p))
    # and write #defines for use by config code
    f.write('}\n\n')
    f.write('// full pin define list\n')
    last_label = None
    for l in sorted(list(set(bylabel.keys()))):
        p = bylabel[l]
        label = p.label
        label = label.replace('-', '_')
        if label == last_label:
            continue
        last_label = label
        f.write('#define HAL_GPIO_PIN_%-20s PAL_LINE(GPIO%s,%uU)\n' %
                (label, p.port, p.pin))
    f.write('\n')


def bootloader_path():
    # always embed a bootloader if it is available
    this_dir = os.path.realpath(__file__)
    rootdir = os.path.relpath(os.path.join(this_dir, "../../../../.."))
    hwdef_dirname = os.path.basename(os.path.dirname(args.hwdef))
    bootloader_filename = "%s_bl.bin" % (hwdef_dirname,)
    bootloader_path = os.path.join(rootdir,
                                   "Tools",
                                   "bootloaders",
                                   bootloader_filename)
    if os.path.exists(bootloader_path):
        return os.path.realpath(bootloader_path)

    return None


def add_bootloader():
    '''added bootloader to ROMFS'''
    bp = bootloader_path()
    if bp is not None:
        romfs["bootloader.bin"] = bp


def write_ROMFS(outdir):
    '''create ROMFS embedded header'''
    romfs_list = []
    for k in romfs.keys():
        romfs_list.append((k, romfs[k]))
    env_vars['ROMFS_FILES'] = romfs_list


def setup_apj_IDs():
    '''setup the APJ board IDs'''
    env_vars['APJ_BOARD_ID'] = get_config('APJ_BOARD_ID')
    env_vars['APJ_BOARD_TYPE'] = get_config('APJ_BOARD_TYPE', default=mcu_type)
    (USB_VID, USB_PID) = get_USB_IDs()
    env_vars['USBID'] = '0x%04x/0x%04x' % (USB_VID, USB_PID)


def write_peripheral_enable(f):
    '''write peripheral enable lines'''
    f.write('// peripherals enabled\n')
    for type in sorted(list(bytype.keys()) + list(alttype.keys())):
        if type.startswith('USART') or type.startswith('UART'):
            dstr = 'STM32_SERIAL_USE_%-6s' % type
            f.write('#ifndef %s\n' % dstr)
            f.write('#define %s TRUE\n' % dstr)
            f.write('#endif\n')
        if type.startswith('SPI'):
            f.write('#define STM32_SPI_USE_%s                  TRUE\n' % type)
        if type.startswith('OTG'):
            f.write('#define STM32_USB_USE_%s                  TRUE\n' % type)
        if type.startswith('I2C'):
            f.write('#define STM32_I2C_USE_%s                  TRUE\n' % type)


def get_dma_exclude(periph_list):
    '''return list of DMA devices to exclude from DMA'''
    dma_exclude = []
    for periph in periph_list:
        if periph in bylabel:
            p = bylabel[periph]
            if p.has_extra('NODMA'):
                dma_exclude.append(periph)
        if periph in altlabel:
            p = altlabel[periph]
            if p.has_extra('NODMA'):
                dma_exclude.append(periph)
    return dma_exclude


def write_alt_config(f):
    '''write out alternate config settings'''
    if len(altmap.keys()) == 0:
        # no alt configs
        return
    f.write('''
/* alternative configurations */
#define PAL_STM32_SPEED(n) ((n&3U)<<3U)
#define PAL_STM32_HIGH     0x8000U

#define HAL_PIN_ALT_CONFIG { \\
''')
    for alt in altmap.keys():
        for pp in altmap[alt].keys():
            p = altmap[alt][pp]
            f.write("    { %u, %s, PAL_LINE(GPIO%s,%uU)}, /* %s */ \\\n" % (alt, p.pal_modeline(), p.port, p.pin, str(p)))
    f.write('}\n\n')


def write_hwdef_header(outfilename):
    '''write hwdef header file'''
    print("Writing hwdef setup in %s" % outfilename)
    f = open(outfilename, 'w')

    f.write('''/*
 generated hardware definitions from hwdef.dat - DO NOT EDIT
*/

#pragma once

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

''')

    write_mcu_config(f)
    write_SPI_config(f)
    write_ADC_config(f)
    write_GPIO_config(f)
    write_IMU_config(f)
    write_MAG_config(f)
    write_BARO_config(f)

    write_peripheral_enable(f)

    dma_unassigned = dma_resolver.write_dma_header(f, periph_list, mcu_type,
                                                   dma_exclude=get_dma_exclude(periph_list),
                                                   dma_priority=get_config('DMA_PRIORITY', default='TIM* SPI*', spaces=True),
                                                   dma_noshare=get_config('DMA_NOSHARE', default='', spaces=True))

    if not args.bootloader:
        write_PWM_config(f)
        write_I2C_config(f)
        write_UART_config(f)
    else:
        write_UART_config_bootloader(f)

    setup_apj_IDs()
    write_USB_config(f)

    add_bootloader()

    if len(romfs) > 0:
        f.write('#define HAL_HAVE_AP_ROMFS_EMBEDDED_H 1\n')

    if mcu_series.startswith('STM32F1'):
        f.write('''
/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_OUTPUT_PP(n)         (0U << (((n) & 7) * 4))
#define PIN_MODE_OUTPUT_OD(n)         (4U << (((n) & 7) * 4))
#define PIN_MODE_AF_PP(n)             (8U << (((n) & 7) * 4))
#define PIN_MODE_AF_OD(n)             (12U << (((n) & 7) * 4))
#define PIN_MODE_ANALOG(n)            (0U << (((n) & 7) * 4))
#define PIN_MODE_NOPULL(n)            (4U << (((n) & 7) * 4))
#define PIN_MODE_PUD(n)               (8U << (((n) & 7) * 4))
#define PIN_SPEED_MEDIUM(n)           (1U << (((n) & 7) * 4))
#define PIN_SPEED_LOW(n)              (2U << (((n) & 7) * 4))
#define PIN_SPEED_HIGH(n)             (3U << (((n) & 7) * 4))
#define PIN_ODR_HIGH(n)               (1U << (((n) & 15)))
#define PIN_ODR_LOW(n)                (0U << (((n) & 15)))
#define PIN_PULLUP(n)                 (1U << (((n) & 15)))
#define PIN_PULLDOWN(n)               (0U << (((n) & 15)))
#define PIN_UNDEFINED(n)                PIN_INPUT_PUD(n)
''')
    else:
        f.write('''
/*
* I/O ports initial setup, this configuration is established soon after reset
* in the initialization code.
* Please refer to the STM32 Reference Manual for details.
*/
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

''')

    for port in sorted(ports):
        f.write("/* PORT%s:\n" % port)
        for pin in range(pincount[port]):
            p = portmap[port][pin]
            if p.label is not None:
                f.write(" %s\n" % p)
        f.write("*/\n\n")

        if pincount[port] == 0:
            # handle blank ports
            for vtype in vtypes:
                f.write("#define VAL_GPIO%s_%-7s             0x0\n" % (port,
                                                                       vtype))
            f.write("\n\n\n")
            continue

        for vtype in vtypes:
            f.write("#define VAL_GPIO%s_%-7s (" % (p.port, vtype))
            first = True
            for pin in range(pincount[port]):
                p = portmap[port][pin]
                modefunc = getattr(p, "get_" + vtype)
                v = modefunc()
                if v is None:
                    continue
                if not first:
                    f.write(" | \\\n                           ")
                f.write(v)
                first = False
            if first:
                # there were no pin definitions, use 0
                f.write("0")
            f.write(")\n\n")
    write_alt_config(f)

    if not mcu_series.startswith("STM32F1"):
        dma_required = ['SPI*', 'ADC*']
        if 'IOMCU_UART' in config:
            dma_required.append(config['IOMCU_UART'][0] + '*')
        for d in dma_unassigned:
            for r in dma_required:
                if fnmatch.fnmatch(d, r):
                    error("Missing required DMA for %s" % d)


def build_peripheral_list():
    '''build a list of peripherals for DMA resolver to work on'''
    peripherals = []
    done = set()
    prefixes = ['SPI', 'USART', 'UART', 'I2C']
    periph_pins = allpins[:]
    for alt in altmap.keys():
        for p in altmap[alt].keys():
            periph_pins.append(altmap[alt][p])
    for p in periph_pins:
        type = p.type
        if type in done:
            continue
        for prefix in prefixes:
            if type.startswith(prefix):
                ptx = type + "_TX"
                prx = type + "_RX"
                if prefix in ['SPI', 'I2C']:
                    # in DMA map I2C and SPI has RX and TX suffix
                    if ptx not in bylabel:
                        bylabel[ptx] = p
                    if prx not in bylabel:
                        bylabel[prx] = p
                if prx in bylabel or prx in altlabel:
                    peripherals.append(prx)
                if ptx in bylabel or ptx in altlabel:
                    peripherals.append(ptx)

        if type.startswith('ADC'):
            peripherals.append(type)
        if type.startswith('SDIO') or type.startswith('SDMMC'):
            if not mcu_series.startswith("STM32H7"):
                peripherals.append(type)
        if type.startswith('TIM'):
            if p.has_extra('RCIN'):
                label = p.label
                if label[-1] == 'N':
                    label = label[:-1]
                peripherals.append(label)
            elif not p.has_extra('ALARM') and not p.has_extra('RCININT'):
                # get the TIMn_UP DMA channels for DShot
                label = type + '_UP'
                if label not in peripherals and not p.has_extra('NODMA'):
                    peripherals.append(label)
        done.add(type)
    return peripherals


def write_env_py(filename):
    '''write out env.py for environment variables to control the build process'''

    # see if board has a defaults.parm file or a --default-parameters file was specified
    defaults_filename = os.path.join(os.path.dirname(args.hwdef), 'defaults.parm')
    defaults_path = os.path.join(os.path.dirname(args.hwdef), args.params)

    if not args.bootloader:
        if os.path.exists(defaults_path):
            env_vars['DEFAULT_PARAMETERS'] = os.path.abspath(defaults_path)
            print("Default parameters path from command line: %s" % defaults_path)
        elif os.path.exists(defaults_filename):
            env_vars['DEFAULT_PARAMETERS'] = os.path.abspath(defaults_filename)
            print("Default parameters path from hwdef: %s" % defaults_filename)
        else:
            print("No default parameter file found")

    # CHIBIOS_BUILD_FLAGS is passed to the ChibiOS makefile
    env_vars['CHIBIOS_BUILD_FLAGS'] = ' '.join(build_flags)
    pickle.dump(env_vars, open(filename, "wb"))


def romfs_add(romfs_filename, filename):
    '''add a file to ROMFS'''
    romfs[romfs_filename] = filename


def romfs_wildcard(pattern):
    '''add a set of files to ROMFS by wildcard'''
    base_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..')
    (pattern_dir, pattern) = os.path.split(pattern)
    for f in os.listdir(os.path.join(base_path, pattern_dir)):
        if fnmatch.fnmatch(f, pattern):
            romfs[f] = os.path.join(pattern_dir, f)


def process_line(line):
    '''process one line of pin definition file'''
    global allpins, imu_list, compass_list, baro_list
    global mcu_type, mcu_series
    a = shlex.split(line)
    # keep all config lines for later use
    alllines.append(line)

    p = None
    if a[0].startswith('P') and a[0][1] in ports:
        # it is a port/pin definition
        try:
            port = a[0][1]
            pin = int(a[0][2:])
            label = a[1]
            type = a[2]
            extra = a[3:]
        except Exception:
            error("Bad pin line: %s" % a)
            return

        p = generic_pin(port, pin, label, type, extra)
        af = get_alt_function(mcu_type, a[0], label)
        if af is not None:
            p.af = af

        alt = p.extra_value("ALT", type=int, default=0)
        if alt != 0:
            if mcu_series.startswith("STM32F1"):
                error("Alt config not allowed for F1 MCU")
            if alt not in altmap:
                altmap[alt] = {}
            if p.portpin in altmap[alt]:
                error("Pin %s ALT(%u) redefined" % (p.portpin, alt))
            altmap[alt][p.portpin] = p
            # we need to add alt pins into bytype[] so they are enabled in chibios config
            if type not in alttype:
                alttype[type] = []
            alttype[type].append(p)
            altlabel[label] = p
            return

        if a[0] in config:
            error("Pin %s redefined" % a[0])

    if p is None and line.find('ALT(') != -1:
        error("ALT() invalid for %s" % a[0])

    config[a[0]] = a[1:]
    if p is not None:
        # add to set of pins for primary config
        portmap[port][pin] = p
        allpins.append(p)
        if type not in bytype:
            bytype[type] = []
        bytype[type].append(p)
        bylabel[label] = p
    elif a[0] == 'MCU':
        mcu_type = a[2]
        mcu_series = a[1]
        setup_mcu_type_defaults()
    elif a[0] == 'SPIDEV':
        spidev.append(a[1:])
    elif a[0] == 'IMU':
        imu_list.append(a[1:])
    elif a[0] == 'COMPASS':
        compass_list.append(a[1:])
    elif a[0] == 'BARO':
        baro_list.append(a[1:])
    elif a[0] == 'ROMFS':
        romfs_add(a[1], a[2])
    elif a[0] == 'ROMFS_WILDCARD':
        romfs_wildcard(a[1])
    elif a[0] == 'undef':
        print("Removing %s" % a[1])
        config.pop(a[1], '')
        bytype.pop(a[1], '')
        bylabel.pop(a[1], '')
        # also remove all occurences of defines in previous lines if any
        for line in alllines[:]:
            if line.startswith('define') and a[1] == line.split()[1]:
                alllines.remove(line)
        newpins = []
        for pin in allpins:
            if pin.type == a[1]:
                continue
            if pin.label == a[1]:
                continue
            if pin.portpin == a[1]:
                continue
            newpins.append(pin)
        allpins = newpins
        if a[1] == 'IMU':
            imu_list = []
        if a[1] == 'COMPASS':
            compass_list = []
        if a[1] == 'BARO':
            baro_list = []
    elif a[0] == 'env':
        print("Adding environment %s" % ' '.join(a[1:]))
        if len(a[1:]) < 2:
            error("Bad env line for %s" % a[0])
        env_vars[a[1]] = ' '.join(a[2:])


def process_file(filename):
    '''process a hwdef.dat file'''
    try:
        f = open(filename, "r")
    except Exception:
        error("Unable to open file %s" % filename)
    for line in f.readlines():
        line = line.strip()
        if len(line) == 0 or line[0] == '#':
            continue
        a = shlex.split(line)
        if a[0] == "include" and len(a) > 1:
            include_file = a[1]
            if include_file[0] != '/':
                dir = os.path.dirname(filename)
                include_file = os.path.normpath(
                    os.path.join(dir, include_file))
            print("Including %s" % include_file)
            process_file(include_file)
        else:
            process_line(line)


# process input file
process_file(args.hwdef)

outdir = args.outdir
if outdir is None:
    outdir = '/tmp'

if "MCU" not in config:
    error("Missing MCU type in config")

mcu_type = get_config('MCU', 1)
print("Setup for MCU %s" % mcu_type)

# build a list for peripherals for DMA resolver
periph_list = build_peripheral_list()

# write out hwdef.h
write_hwdef_header(os.path.join(outdir, "hwdef.h"))

# write out ldscript.ld
write_ldscript(os.path.join(outdir, "ldscript.ld"))

write_ROMFS(outdir)

# copy the shared linker script into the build directory; it must
# exist in the same directory as the ldscript.ld file we generate.
copy_common_linkerscript(outdir, args.hwdef)

write_env_py(os.path.join(outdir, "env.py"))
