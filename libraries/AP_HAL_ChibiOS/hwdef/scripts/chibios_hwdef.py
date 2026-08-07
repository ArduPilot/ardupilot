#!/usr/bin/env python3
'''
setup board.h for chibios

AP_FLAKE8_CLEAN

'''

import argparse
import sys
import fnmatch
import os
import dma_resolver
import shlex
import re
import shutil

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../../libraries/AP_HAL/hwdef/scripts'))
import hwdef  # noqa:E402


class ChibiOSHWDefIncludeNotFoundException(Exception):
    def __init__(self, hwdef, includer):
        self.hwdef = hwdef
        self.includer = includer


class ChibiOSHWDef(hwdef.HWDef):

    # output variables for each pin
    f4f7_vtypes = ['MODER', 'OTYPER', 'OSPEEDR', 'PUPDR', 'ODR', 'AFRL', 'AFRH']
    f1_vtypes = ['CRL', 'CRH', 'ODR']
    af_labels = ['USART', 'UART', 'SPI', 'I2C', 'SDIO', 'SDMMC', 'OTG', 'JT', 'TIM', 'CAN', 'QUADSPI', 'OCTOSPI', 'ETH', 'MCO']

    def __init__(self, bootloader=False, signed_fw=False, default_params_filepath=None, **kwargs):
        super(ChibiOSHWDef, self).__init__(**kwargs)
        self.bootloader = bootloader
        self.signed_fw = signed_fw
        self.default_params_filepath = default_params_filepath
        self.processed_defaults_filepath = None
        self.have_defaults_file = False

        # if true then parameters will be appended in special apj-tool
        # section at end of binary:
        self.force_apj_default_parameters = False

        self.default_gpio = ['INPUT', 'FLOATING']

        self.vtypes = []

        # number of pins in each port
        self.pincount = {
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

        self.ports = self.pincount.keys()

        self.portmap = {}

        # alternate pin mappings
        self.altmap = {}

        # list of all pins in config file order
        self.allpins = []

        # list of configs by type
        self.bytype = {}

        # list of alt configs by type
        self.alttype = {}

        # list of configs by label
        self.bylabel = {}

        # list of alt configs by label
        self.altlabel = {}

        # list of SPI devices
        self.spidev = []

        # list of WSPI devices
        self.wspidev = []

        # dictionary of ROMFS files
        self.romfs = {}

        # SPI bus list
        self.spi_list = []

        # list of WSPI devices
        self.wspi_list = []

        # build flags for ChibiOS makefiles
        self.build_flags = []

        # sensor lists
        self.airspeed_list = []

        # dataflash config
        self.dataflash_list = []

        self.dma_exclude_pattern = []

        self.mcu_type = None
        self.dual_USB_enabled = False

        # list of device patterns that can't be shared
        self.dma_noshare = []

        # list of shared up timers
        self.shared_up = []

    def get_mcu_lib(self, mcu):
        '''get library file for the chosen MCU'''
        import importlib
        try:
            return importlib.import_module(mcu)
        except ImportError:
            self.error("Unable to find module for MCU %s" % mcu)

    def setup_mcu_type_defaults(self):
        '''setup defaults for given mcu type'''
        lib = self.get_mcu_lib(self.mcu_type)
        if hasattr(lib, 'pincount'):
            self.pincount = lib.pincount
        if self.mcu_series.startswith("STM32F1"):
            self.vtypes = self.f1_vtypes
        else:
            self.vtypes = self.f4f7_vtypes
        self.ports = self.pincount.keys()
        # setup default as input pins
        for port in self.ports:
            self.portmap[port] = []
            for pin in range(self.pincount[port]):
                self.portmap[port].append(self.generic_pin(port, pin, None, self.default_gpio[0], self.default_gpio[1:], self.mcu_type, self.mcu_series, self.get_ADC1_chan, self.get_ADC2_chan, self.get_ADC3_chan, self.af_labels))  # noqa

        if self.mcu_series.startswith("STM32H7") or self.mcu_series.startswith("STM32F7"):
            # default DMA off on I2C for H7, we're much better off reducing DMA sharing
            self.dma_exclude_pattern = ['I2C*']

    def get_alt_function(self, mcu, pin, function):
        '''return alternative function number for a pin'''
        lib = self.get_mcu_lib(mcu)

        if function.endswith('_TXINV') or function.endswith('_RXINV'):
            # RXINV and TXINV are special labels for inversion pins, not alt-functions
            return None

        if hasattr(lib, "AltFunction_map"):
            alt_map = lib.AltFunction_map
        else:
            # just check if Alt Func is available or not
            for label in self.af_labels:
                if function.startswith(label):
                    return 0
            return None

        if function and (function.endswith("_RTS") or function.endswith("_CTS_GPIO")) and (
                function.startswith('USART') or function.startswith('UART')):
            # we do software RTS and can do either software CTS or hardware CTS
            return None

        for label in self.af_labels:
            if function.startswith(label):
                s = pin + ":" + function
                if s not in alt_map:
                    self.error("Unknown pin function %s for MCU %s" % (s, mcu))
                return alt_map[s]
        return None

    def have_type_prefix(self, ptype):
        '''return True if we have a peripheral starting with the given peripheral type'''
        for t in list(self.bytype.keys()) + list(self.alttype.keys()):
            if t.startswith(ptype):
                return True
        return False

    def get_ADC1_chan(self, mcu, pin):
        '''return ADC1 channel for an analog pin'''
        import importlib
        try:
            lib = importlib.import_module(mcu)
            ADC1_map = lib.ADC1_map
        except ImportError:
            self.error("Unable to find ADC1_Map for MCU %s" % mcu)

        if pin not in ADC1_map:
            self.error("Unable to find ADC1 channel for pin %s" % pin)
        return ADC1_map[pin]

    def get_ADC2_chan(self, mcu, pin):
        '''return ADC2 channel for an analog pin'''
        import importlib
        try:
            lib = importlib.import_module(mcu)
            ADC2_map = lib.ADC2_map
        except ImportError:
            self.error("Unable to find ADC2_Map for MCU %s" % mcu)

        if pin not in ADC2_map:
            self.error("Unable to find ADC2 channel for pin %s" % pin)
        return ADC2_map[pin]

    def get_ADC3_chan(self, mcu, pin):
        '''return ADC3 channel for an analog pin'''
        import importlib
        try:
            lib = importlib.import_module(mcu)
            ADC3_map = lib.ADC3_map
        except ImportError:
            self.error("Unable to find ADC3_Map for MCU %s" % mcu)

        if pin not in ADC3_map:
            self.error("Unable to find ADC3 channel for pin %s" % pin)
        return ADC3_map[pin]

    class generic_pin(object):
        '''class to hold pin definition'''

        def __init__(self,
                     port,
                     pin,
                     label,
                     type,
                     extra,
                     mcu_type,
                     mcu_series,
                     get_ADC1_chan,
                     get_ADC2_chan,
                     get_ADC3_chan,
                     af_labels
                     ):
            self.portpin = "P%s%u" % (port, pin)
            self.port = port
            self.pin = pin
            self.label = label
            self.type = type
            self.extra = extra
            self.af = None
            self.mcu_type = mcu_type
            self.mcu_series = mcu_series
            # these are methods supplied to us to resolve channel numbers:
            self.get_ADC1_chan = get_ADC1_chan
            self.get_ADC2_chan = get_ADC2_chan
            self.get_ADC3_chan = get_ADC3_chan
            self.af_labels = af_labels

            if type == 'OUTPUT':
                self.sig_dir = 'OUTPUT'
            else:
                self.sig_dir = 'INPUT'
            if mcu_series.startswith("STM32F1") and self.label is not None:
                self.f1_pin_setup(mcu_type)

            # check that labels and pin types are consistent
            for prefix in ['USART', 'UART', 'TIM']:
                if label is None or type is None:
                    continue
                if type.startswith(prefix):
                    a1 = label.split('_')
                    a2 = type.split('_')
                    if a1[0] != a2[0]:
                        self.error("Peripheral prefix mismatch for %s %s %s" % (self.portpin, label, type))

        def f1_pin_setup(self, mcu_type):
            f1_input_sigs = ['RX', 'MISO', 'CTS']
            f1_output_sigs = ['TX', 'MOSI', 'SCK', 'RTS', 'CH1', 'CH2', 'CH3', 'CH4']
            for label in self.af_labels:
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
                        self.error("Unknown signal type %s:%s for %s!" % (self.portpin, self.label, mcu_type))

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
                self.error("Badly formed value for %s: %s\n" % (name, v))
            ret = v[len(name) + 1:-1]
            if type is not None:
                try:
                    ret = type(ret)
                except Exception:
                    self.error("Badly formed value for %s: %s\n" % (name, ret))
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
                self.error("Bad OSPEED %s" % v)
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
                 self.label.endswith('_CTS'))):
                v = "PULLUP"

            # pulldown on RTS to prevent radios from staying in bootloader
            if (self.type.startswith('USART') or
                self.type.startswith('UART')) and (
                 self.label.endswith('_RTS')):
                v = "PULLDOWN"

            if (self.type.startswith('SWD') and
                    'SWDIO' in self.label):
                v = "PULLUP"

            if (self.type.startswith('SWD') and
                    'SWCLK' in self.label):
                v = "PULLDOWN"

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
            if self.mcu_series.startswith("STM32F1"):
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
            if self.mcu_series.startswith("STM32F1"):
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

        def periph_type(self):
            '''return peripheral type from GPIO_PIN_TYPE class'''
            patterns = {
                'USART*RX' : 'PERIPH_TYPE::UART_RX',
                'UART*RX' : 'PERIPH_TYPE::UART_RX',
                'USART*TX' : 'PERIPH_TYPE::UART_TX',
                'UART*TX' : 'PERIPH_TYPE::UART_TX',
                'I2C*SDA' : 'PERIPH_TYPE::I2C_SDA',
                'I2C*SCL' : 'PERIPH_TYPE::I2C_SCL',
                'EXTERN_GPIO*' : 'PERIPH_TYPE::GPIO',
            }
            for k in patterns.keys():
                if fnmatch.fnmatch(self.label, k):
                    return patterns[k]
            return 'PERIPH_TYPE::OTHER'

        def periph_instance(self):
            '''return peripheral instance'''
            if self.periph_type() == 'PERIPH_TYPE::GPIO':
                result = re.match(r'[A-Z_]*([0-9]+)', self.label)
            else:
                result = re.match(r'[A-Z_]*([0-9]+)', self.type)
            if result:
                return int(result.group(1))
            return 0

        def __str__(self):
            str = ''
            if self.af is not None:
                str += " AF%u" % self.af
            if self.type.startswith('ADC1'):
                str += " ADC1_IN%u" % self.get_ADC1_chan(self.mcu_type, self.portpin)
            if self.type.startswith('ADC2'):
                str += " ADC2_IN%u" % self.get_ADC2_chan(self.mcu_type, self.portpin)
            if self.type.startswith('ADC3'):
                str += " ADC3_IN%u" % self.get_ADC3_chan(self.mcu_type, self.portpin)
            if self.extra_value('PWM', type=int):
                str += " PWM%u" % self.extra_value('PWM', type=int)
            return "P%s%u %s %s%s" % (self.port, self.pin, self.label, self.type,
                                      str)

    def get_config(self, name, column=0, required=True, default=None, type=None, spaces=False, aslist=False):
        '''get a value from config dictionary'''
        if name not in self.config:
            if required and default is None:
                self.error("missing required value %s in hwdef.dat" % name)
            return default
        if aslist:
            return self.config[name]
        if len(self.config[name]) < column + 1:
            if not required:
                return None
            self.error("missing required value %s in hwdef.dat (column %u)" %
                       (name, column))
        if spaces:
            ret = ' '.join(self.config[name][column:])
        else:
            ret = self.config[name][column]

        if type is not None:
            if type == int and ret.startswith('0x'):
                try:
                    ret = int(ret, 16)
                except Exception:
                    self.error("Badly formed config value %s (got %s)" % (name, ret))
            else:
                try:
                    ret = type(ret)
                except Exception:
                    self.error("Badly formed config value %s (got %s)" % (name, ret))
        return ret

    def get_mcu_config(self, name, required=False):
        '''get a value from the mcu dictionary'''
        lib = self.get_mcu_lib(self.mcu_type)
        if not hasattr(lib, 'mcu'):
            self.error("Missing mcu config for %s" % self.mcu_type)
        if name not in lib.mcu:
            if required:
                self.error("Missing required mcu config %s for %s" % (name, self.mcu_type))
            return None
        return lib.mcu[name]

    def get_ram_reserve_start(self):
        '''get amount of memory to reserve for bootloader comms and the address if non-zero'''
        ram_reserve_start = self.get_config('RAM_RESERVE_START', default=0, type=int)
        if ram_reserve_start == 0 and self.is_periph_fw():
            ram_reserve_start = 256
        ram_map_bootloader = self.get_ram_map(use_bootloader=True)
        ram0_start_address = ram_map_bootloader[0][0]
        return ram_reserve_start, ram0_start_address

    def make_line(self, label):
        '''return a line for a label'''
        if label in self.bylabel:
            p = self.bylabel[label]
            line = 'PAL_LINE(GPIO%s,%uU)' % (p.port, p.pin)
        else:
            line = "0"
        return line

    def disable_can(self, f):
        '''setup for a non-CAN enabled board'''
        f.write("#define HAL_NUM_CAN_IFACES 0\n")
        f.write("#undef HAL_ENABLE_DRONECAN_DRIVERS\n")
        f.write("#define HAL_ENABLE_DRONECAN_DRIVERS 0\n")

    def enable_can(self, f):
        '''setup for a CAN enabled board'''
        if self.mcu_series.startswith("STM32H7") or self.mcu_series.startswith("STM32G4"):
            prefix = "FDCAN"
            cast = "CanType"
        else:
            prefix = "CAN"
            cast = "bxcan::CanType"

        # allow for optional CAN_ORDER option giving bus order
        can_order_str = self.get_config('CAN_ORDER', required=False, aslist=True)
        if can_order_str:
            can_order = [int(s) for s in can_order_str]
        else:
            can_order = []
            for i in range(1, 3):
                if 'CAN%u' % i in self.bytype or (i == 1 and 'CAN' in self.bytype):
                    can_order.append(i)

        base_list = []
        for i in can_order:
            base_list.append("reinterpret_cast<%s*>(uintptr_t(%s%s_BASE))" % (cast, prefix, i))
            f.write("#define HAL_CAN_IFACE%u_ENABLE\n" % i)

        can_rev_order = [-1]*3
        for i in range(len(can_order)):
            can_rev_order[can_order[i]-1] = i

        f.write('#define HAL_CAN_INTERFACE_LIST %s\n' % ','.join([str(i-1) for i in can_order]))
        f.write('#define HAL_CAN_INTERFACE_REV_LIST %s\n' % ','.join([str(i) for i in can_rev_order]))
        f.write('#define HAL_CAN_BASE_LIST %s\n' % ','.join(base_list))
        f.write('#define HAL_NUM_CAN_IFACES %d\n' % len(base_list))
        if 'CAN' in self.bytype and self.mcu_type.startswith("STM32F3"):
            f.write('#define CAN1_BASE CAN_BASE\n')
        self.env_vars['HAL_NUM_CAN_IFACES'] = str(len(base_list))

        if self.mcu_series.startswith("STM32H7") and not self.is_bootloader_fw():
            # set maximum supported canfd bit rate in MBits/sec
            canfd_supported = int(self.get_config('CANFD_SUPPORTED', 0, default=4, required=False))
            f.write('#define HAL_CANFD_SUPPORTED %d\n' % canfd_supported)
            self.env_vars['HAL_CANFD_SUPPORTED'] = canfd_supported
        else:
            canfd_supported = int(self.get_config('CANFD_SUPPORTED', 0, default=0, required=False))
            f.write('#define HAL_CANFD_SUPPORTED %d\n' % canfd_supported)
            self.env_vars['HAL_CANFD_SUPPORTED'] = canfd_supported

    def has_dataflash_spi(self):
        '''check for dataflash connected to spi bus'''
        for dev in self.spidev:
            if dev[0] == 'dataflash':
                return True
        return False

    def has_sdcard_spi(self):
        '''check for sdcard connected to spi bus'''
        for dev in self.spidev:
            if dev[0] == 'sdcard':
                return True
        return False

    def get_ram_map(self, use_bootloader=False):
        '''get RAM_MAP. May be different for bootloader'''
        if self.is_bootloader_fw() or use_bootloader:
            ram_map = self.get_mcu_config('RAM_MAP_BOOTLOADER', False)
            if ram_map is not None:
                return ram_map
        elif self.env_vars['EXT_FLASH_SIZE_MB'] and not self.env_vars['INT_FLASH_PRIMARY']:
            ram_map = self.get_mcu_config('RAM_MAP_EXTERNAL_FLASH', False)
            if ram_map is not None:
                return ram_map
        elif int(self.env_vars.get('USE_ALT_RAM_MAP', 0)) == 1:
            self.progress("Using ALT_RAM_MAP")
            return self.get_mcu_config('ALT_RAM_MAP', True)
        return self.get_mcu_config('RAM_MAP', True)

    def get_flash_pages_sizes(self):
        mcu_series = self.mcu_series
        mcu_type = self.mcu_type
        if mcu_series.startswith('STM32F4') or mcu_series.startswith('CKS32F4'):
            if self.get_config('FLASH_SIZE_KB', type=int) == 512:
                return [16, 16, 16, 16, 64, 128, 128, 128]
            elif self.get_config('FLASH_SIZE_KB', type=int) == 1024:
                return [16, 16, 16, 16, 64, 128, 128, 128, 128, 128, 128, 128]
            elif self.get_config('FLASH_SIZE_KB', type=int) == 2048:
                return [
                    16, 16, 16, 16,
                    64, 128, 128, 128,
                    128, 128, 128, 128,
                    128, 128, 128, 128,
                    128, 128, 128, 128,
                    128, 128, 128, 128
                ]
            else:
                raise Exception("Unsupported flash size %u" % self.get_config('FLASH_SIZE_KB', type=int))
        elif mcu_series.startswith('STM32F7'):
            if self.get_config('FLASH_SIZE_KB', type=int) == 512:
                return [16, 16, 16, 16, 64, 128, 128, 128]
            elif self.get_config('FLASH_SIZE_KB', type=int) == 1024:
                return [32, 32, 32, 32, 128, 256, 256, 256]
            elif self.get_config('FLASH_SIZE_KB', type=int) == 2048:
                return [32, 32, 32, 32, 128, 256, 256, 256,
                        256, 256, 256, 256]
            else:
                raise Exception("Unsupported flash size %u" % self.get_config('FLASH_SIZE_KB', type=int))
        elif mcu_type.startswith('STM32H7A'):
            return [8] * (self.get_config('FLASH_SIZE_KB', type=int)//8)
        elif mcu_series.startswith('STM32H7'):
            return [128] * (self.get_config('FLASH_SIZE_KB', type=int)//128)
        elif mcu_series.startswith('STM32F100') or mcu_series.startswith('STM32F103'):
            return [1] * self.get_config('FLASH_SIZE_KB', type=int)
        elif mcu_series.startswith('STM32L4') and self.mcu_type.startswith('STM32L4R'):
            # STM32L4PLUS
            return [4] * (self.get_config('FLASH_SIZE_KB', type=int)//4)
        elif (mcu_series.startswith('STM32F105') or
              mcu_series.startswith('STM32F3') or
              mcu_series.startswith('STM32G4') or
              mcu_series.startswith('STM32L4')):
            return [2] * (self.get_config('FLASH_SIZE_KB', type=int)//2)
        else:
            raise Exception("Unsupported flash size MCU %s" % mcu_series)

    def get_flash_npages(self):
        pages = self.get_flash_pages_sizes()
        total_size = sum(pages)
        if total_size != self.get_config('FLASH_SIZE_KB', type=int):
            raise Exception("Invalid flash size MCU %s" % self.mcu_series)
        return len(pages)

    def get_flash_page_offset_kb(self, sector):
        '''return the offset in flash of a page number'''
        pages = self.get_flash_pages_sizes()
        offset = 0
        for i in range(sector):
            offset += pages[i]
        return offset

    def get_storage_flash_page(self):
        '''get STORAGE_FLASH_PAGE either from this hwdef or from hwdef.dat
           in the same directory if this is a bootloader
        '''
        storage_flash_page = self.get_config('STORAGE_FLASH_PAGE', default=None, type=int, required=False)
        if storage_flash_page is not None:
            return storage_flash_page
        if self.is_bootloader_fw() and self.hwdef[0].find("-bl") != -1:
            hwdefdat = self.hwdef[0].replace("-bl", "")
            if os.path.exists(hwdefdat):
                ret = None
                lines = self.load_file_with_include(hwdefdat)
                for line in lines:
                    result = re.match(r'STORAGE_FLASH_PAGE\s*([0-9]+)', line)
                    if result:
                        ret = int(result.group(1))
                return ret
        return None

    def validate_flash_storage_size(self):
        '''check there is room for storage with HAL_STORAGE_SIZE'''
        if self.intdefines.get('HAL_WITH_RAMTRON', 0) == 1:
            # no check for RAMTRON storage
            return
        storage_flash_page = self.get_storage_flash_page()
        pages = self.get_flash_pages_sizes()
        page_size = pages[storage_flash_page] * 1024
        if self.intdefines.get('AP_FLASH_STORAGE_DOUBLE_PAGE', 0) == 1:
            page_size *= 2
        storage_size = self.intdefines.get('HAL_STORAGE_SIZE', None)
        if storage_size is None:
            self.error('Need HAL_STORAGE_SIZE define')
        if storage_size >= page_size:
            self.error("HAL_STORAGE_SIZE too large %u %u" % (storage_size, page_size))
        if page_size == 16384 and storage_size > 15360:
            self.error("HAL_STORAGE_SIZE invalid, needs to be 15360")

    def enable_networking(self, f):
        f.write('''
#ifndef AP_NETWORKING_ENABLED
#define AP_NETWORKING_ENABLED 1
#endif
#define CH_CFG_USE_MAILBOXES 1
''')

    def write_mcu_config(self, f):
        '''write MCU config defines'''
        f.write('#define CHIBIOS_BOARD_NAME "%s"\n' % os.path.basename(os.path.dirname(self.hwdef[0])))
        f.write('// MCU type (ChibiOS define)\n')
        f.write('#define %s_MCUCONF\n' % self.get_config('MCU'))
        mcu_subtype = self.get_config('MCU', 1)
        if mcu_subtype[-1:] == 'x' or mcu_subtype[-2:-1] == 'x':
            f.write('#define %s_MCUCONF\n\n' % mcu_subtype[:-2])
        f.write('#define %s\n\n' % mcu_subtype)
        f.write('// crystal frequency\n')
        f.write('#define STM32_HSECLK %sU\n\n' % self.get_config('OSCILLATOR_HZ'))
        f.write('// UART used for stdout (printf)\n')
        if self.get_config('STDOUT_SERIAL', required=False):
            f.write('#define HAL_STDOUT_SERIAL %s\n\n' % self.get_config('STDOUT_SERIAL'))
            f.write('// baudrate used for stdout (printf)\n')
            f.write('#define HAL_STDOUT_BAUDRATE %u\n\n' % self.get_config('STDOUT_BAUDRATE', type=int))
        if len(self.dataflash_list) > 0:
            # we only support dataflash OR sdcard, so prioritize dataflash if its been explicitly configured
            f.write('#define HAL_USE_FATFS FALSE\n\n')
            f.write('#define HAL_USE_SDC FALSE\n')
            self.build_flags.append('USE_FATFS=no')
        elif self.have_type_prefix('SDIO'):
            f.write('// SDIO available, enable POSIX filesystem support\n')
            f.write('#define USE_POSIX\n')
            f.write('#define HAL_OS_POSIX_IO TRUE\n\n')
            f.write('#define HAL_USE_FATFS TRUE\n\n')
            f.write('#define HAL_USE_SDC TRUE\n')
            self.build_flags.append('USE_FATFS=yes')
            self.env_vars['WITH_FATFS'] = "1"
        elif self.have_type_prefix('SDMMC2'):
            f.write('// SDMMC2 available, enable POSIX filesystem support\n')
            f.write('#define USE_POSIX\n')
            f.write('#define HAL_OS_POSIX_IO TRUE\n\n')
            f.write('#define HAL_USE_FATFS TRUE\n\n')
            f.write('#define HAL_USE_SDC TRUE\n')
            f.write('#define STM32_SDC_USE_SDMMC2 TRUE\n')
            f.write('#define HAL_USE_SDMMC 1\n')
            self.build_flags.append('USE_FATFS=yes')
            self.env_vars['WITH_FATFS'] = "1"
        elif self.have_type_prefix('SDMMC'):
            f.write('// SDMMC available, enable POSIX filesystem support\n')
            f.write('#define USE_POSIX\n')
            f.write('#define HAL_USE_FATFS TRUE\n\n')
            f.write('#define HAL_OS_POSIX_IO TRUE\n\n')
            f.write('#define HAL_USE_SDC TRUE\n')
            f.write('#define STM32_SDC_USE_SDMMC1 TRUE\n')
            f.write('#define HAL_USE_SDMMC 1\n')
            self.build_flags.append('USE_FATFS=yes')
            self.env_vars['WITH_FATFS'] = "1"
        elif self.has_sdcard_spi():
            f.write('// MMC via SPI available, enable POSIX filesystem support\n')
            f.write('#define USE_POSIX\n')
            f.write('#define HAL_USE_FATFS TRUE\n\n')
            f.write('#define HAL_OS_POSIX_IO TRUE\n\n')
            f.write('#define HAL_USE_MMC_SPI TRUE\n')
            f.write('#define HAL_USE_SDC FALSE\n')
            f.write('#define HAL_SDCARD_SPI_HOOK TRUE\n')
            self.build_flags.append('USE_FATFS=yes')
            self.env_vars['WITH_FATFS'] = "1"
        else:
            f.write('#define HAL_USE_FATFS FALSE\n\n')
            f.write('#define HAL_USE_SDC FALSE\n')
            self.build_flags.append('USE_FATFS=no')
        if 'OTG1' in self.bytype:
            if self.get_mcu_config('STM32_OTG2_IS_OTG1', False) is not None:
                f.write('#define STM32_USB_USE_OTG2                  TRUE\n')
                f.write('#define STM32_OTG2_IS_OTG1                  TRUE\n')
            else:
                f.write('#define STM32_USB_USE_OTG1                  TRUE\n')
                f.write('#define STM32_OTG2_IS_OTG1                  FALSE\n')
            f.write('#define HAL_USE_USB TRUE\n')
            f.write('#define HAL_USE_SERIAL_USB TRUE\n')
        if 'OTG2' in self.bytype:
            f.write('#define STM32_USB_USE_OTG2                  TRUE\n')

        if 'ETH1' in self.bytype:
            self.enable_networking(f)
            f.write('''

#define HAL_USE_MAC                         TRUE
#define MAC_USE_EVENTS                      TRUE
#define STM32_ETH_BUFFERS_EXTERN

''')
        defines = self.get_mcu_config('DEFINES', False)
        if defines is not None:
            for d in defines.keys():
                v = defines[d]
                f.write("#ifndef %s\n#define %s %s\n#endif\n" % (d, d, v))
        else:
            defines = {}
        # enable RNG for all H7 chips
        if self.mcu_series.startswith("STM32H7") and 'HAL_USE_HW_RNG' not in defines.keys():
            f.write("#define HAL_USE_HW_RNG TRUE\n")
        elif 'HAL_USE_HW_RNG' not in defines.keys():
            f.write("#define HAL_USE_HW_RNG FALSE\n")

        if self.get_config('PROCESS_STACK', required=False):
            self.env_vars['PROCESS_STACK'] = self.get_config('PROCESS_STACK')
        else:
            self.env_vars['PROCESS_STACK'] = "0x1C00"

        f.write('#define HAL_PROCESS_STACK_SIZE %s\n' % self.env_vars['PROCESS_STACK'])
        # MAIN_STACK is location of initial stack on startup and is also the stack
        # used for slow interrupts. It needs to be big enough for maximum interrupt
        # nesting
        if self.get_config('MAIN_STACK', required=False):
            self.env_vars['MAIN_STACK'] = self.get_config('MAIN_STACK')
        else:
            self.env_vars['MAIN_STACK'] = "0x600"

        if self.get_config('IOMCU_FW', required=False):
            self.env_vars['IOMCU_FW'] = self.get_config('IOMCU_FW')
        else:
            self.env_vars['IOMCU_FW'] = 0

        # check if heater pin defined
        if 'HEATER' in self.bylabel.keys():
            self.env_vars['IOMCU_FW_WITH_HEATER'] = 1
        else:
            self.env_vars['IOMCU_FW_WITH_HEATER'] = 0

        if self.get_config('PERIPH_FW', required=False):
            self.env_vars['PERIPH_FW'] = self.get_config('PERIPH_FW')
        else:
            self.env_vars['PERIPH_FW'] = 0

        # write any custom STM32 defines
        using_chibios_can = False
        for d in self.alllines:
            if d.startswith('STM32_'):
                f.write('#define %s\n' % d)
            if d.startswith('define '):
                if 'HAL_USE_CAN' in d:
                    using_chibios_can = True
                f.write('#define %s\n' % d[7:])

        if self.intdefines.get('AP_NETWORKING_ENABLED', 0) == 1:
            self.enable_networking(f)

        if self.intdefines.get('HAL_USE_USB_MSD', 0) == 1:
            self.build_flags.append('USE_USB_MSD=yes')

        if self.have_type_prefix('CAN') and not using_chibios_can:
            self.enable_can(f)
        else:
            self.disable_can(f)
        flash_size = self.get_config('FLASH_SIZE_KB', type=int)
        f.write('#define BOARD_FLASH_SIZE %u\n' % flash_size)
        self.env_vars['BOARD_FLASH_SIZE'] = flash_size

        flash_reserve_start = self.get_config(
            'FLASH_RESERVE_START_KB', default=16, type=int)
        f.write('\n// location of loaded firmware\n')
        f.write('#define FLASH_LOAD_ADDRESS 0x%08x\n' % (0x08000000 + flash_reserve_start*1024))
        # can be no persistent parameters if no space allocated for them
        if not self.is_bootloader_fw() and flash_reserve_start == 0:
            f.write('#define HAL_ENABLE_SAVE_PERSISTENT_PARAMS 0\n')

        f.write('#define EXT_FLASH_SIZE_MB %u\n' % self.get_config('EXT_FLASH_SIZE_MB', default=0, type=int))
        f.write('#define EXT_FLASH_RESERVE_START_KB %u\n' % self.get_config('EXT_FLASH_RESERVE_START_KB', default=0, type=int))
        f.write('#define EXT_FLASH_RESERVE_END_KB %u\n' % self.get_config('EXT_FLASH_RESERVE_END_KB', default=0, type=int))

        self.env_vars['EXT_FLASH_SIZE_MB'] = self.get_config('EXT_FLASH_SIZE_MB', default=0, type=int)
        self.env_vars['INT_FLASH_PRIMARY'] = self.get_config('INT_FLASH_PRIMARY', default=False, type=bool)
        if self.env_vars['EXT_FLASH_SIZE_MB'] and not self.is_bootloader_fw() and not self.env_vars['INT_FLASH_PRIMARY']:
            f.write('#define CRT0_AREAS_NUMBER 4\n')
            f.write('#define __FASTRAMFUNC__ __attribute__ ((__section__(".fastramfunc")))\n')
            f.write('#define __RAMFUNC__ __attribute__ ((__section__(".ramfunc")))\n')
            f.write('#define PORT_IRQ_ATTRIBUTES __FASTRAMFUNC__\n')
        else:
            f.write('#define CRT0_AREAS_NUMBER 1\n')

        if self.env_vars['INT_FLASH_PRIMARY']:
            # this will put methods with low latency requirements into external flash
            # and save internal flash space
            f.write('#define __EXTFLASHFUNC__ __attribute__ ((__section__(".extflash")))\n')
        else:
            f.write('#define __EXTFLASHFUNC__\n')

        storage_flash_page = self.get_storage_flash_page()
        flash_reserve_end = self.get_config('FLASH_RESERVE_END_KB', default=0, type=int)
        if storage_flash_page is not None:
            if not self.is_bootloader_fw():
                f.write('#define STORAGE_FLASH_PAGE %u\n' % storage_flash_page)
                self.validate_flash_storage_size()
            elif self.get_config('FLASH_RESERVE_END_KB', type=int, required=False) is None:
                # ensure the flash page leaves room for bootloader
                offset = self.get_flash_page_offset_kb(storage_flash_page)
                bl_offset = self.get_config('FLASH_BOOTLOADER_LOAD_KB', type=int)
                # storage at end of flash - leave room
                if offset > bl_offset:
                    flash_reserve_end = flash_size - offset
            if self.is_bootloader_fw():
                f.write('#define STORAGE_FLASH_START_PAGE %u\n' % storage_flash_page)

        crashdump_enabled = bool(self.intdefines.get('AP_CRASHDUMP_ENABLED', (flash_size >= 2048 and not self.is_bootloader_fw())))  # noqa
        # lets pick a flash sector for Crash log
        f.write('#ifndef AP_CRASHDUMP_ENABLED\n')
        f.write('#define AP_CRASHDUMP_ENABLED %u\n' % crashdump_enabled)
        f.write('#endif\n')
        self.env_vars['ENABLE_CRASHDUMP'] = crashdump_enabled

        if self.is_bootloader_fw():
            if self.env_vars['EXT_FLASH_SIZE_MB'] and not self.env_vars['INT_FLASH_PRIMARY']:
                f.write('\n// location of loaded firmware in external flash\n')
                f.write('#define APP_START_ADDRESS 0x%08x\n' % (0x90000000 + self.get_config(
                    'EXT_FLASH_RESERVE_START_KB', default=0, type=int)*1024))
                f.write('#define BOOT_FROM_EXT_FLASH 1\n')
            f.write('#define FLASH_BOOTLOADER_LOAD_KB %u\n' % self.get_config('FLASH_BOOTLOADER_LOAD_KB', type=int))
            f.write('#define FLASH_RESERVE_END_KB %u\n' % flash_reserve_end)
            f.write('#define APP_START_OFFSET_KB %u\n' % self.get_config('APP_START_OFFSET_KB', default=0, type=int))
        f.write('\n')

        ram_reserve_start, ram0_start_address = self.get_ram_reserve_start()
        f.write('#define HAL_RAM0_START 0x%08x\n' % ram0_start_address)
        if ram_reserve_start > 0:
            f.write('#define HAL_RAM_RESERVE_START 0x%08x\n' % ram_reserve_start)

        ram_map = self.get_ram_map()

        f.write('// memory regions\n')
        regions = []
        cc_regions = []
        total_memory = 0
        for (address, size, flags) in ram_map:
            size *= 1024
            cc_regions.append('{0x%08x, 0x%08x, CRASH_CATCHER_BYTE }' % (address, address + size))
            if address == ram0_start_address:
                address += ram_reserve_start
                size -= ram_reserve_start
            regions.append('{(void*)0x%08x, 0x%08x, 0x%02x }' % (address, size, flags))
            total_memory += size
        f.write('#define HAL_MEMORY_REGIONS %s\n' % ', '.join(regions))
        f.write('#define HAL_CC_MEMORY_REGIONS %s\n' % ', '.join(cc_regions))
        f.write('#define HAL_MEMORY_TOTAL_KB %u\n' % (total_memory/1024))

        f.write('\n// CPU serial number (12 bytes)\n')
        udid_start = self.get_mcu_config('UDID_START')
        if udid_start is None:
            f.write('#define UDID_START UID_BASE\n\n')
        else:
            f.write('#define UDID_START 0x%08x\n\n' % udid_start)

        f.write('\n// APJ board ID (for bootloaders)\n')
        f.write('#define APJ_BOARD_ID %s\n' % self.get_numeric_board_id())

        # support ALT_BOARD_ID for px4 firmware
        alt_id = self.get_config('ALT_BOARD_ID', required=False)
        if alt_id is not None:
            f.write('#define ALT_BOARD_ID %s\n' % alt_id)

        f.write('''
#ifndef HAL_ENABLE_THREAD_STATISTICS
#define HAL_ENABLE_THREAD_STATISTICS FALSE
#endif
    ''')

        lib = self.get_mcu_lib(self.mcu_type)
        build_info = lib.build

        if self.get_mcu_config('CPU_FLAGS') and self.get_mcu_config('CORTEX'):
            # CPU flags specified in mcu file
            cortex = self.get_mcu_config('CORTEX')
            self.env_vars['CPU_FLAGS'] = self.get_mcu_config('CPU_FLAGS').split()
            build_info['MCU'] = cortex
            self.progress("MCU Flags: %s %s" % (cortex, self.env_vars['CPU_FLAGS']))
        elif self.mcu_series.startswith("STM32F1"):
            cortex = "cortex-m3"
            self.env_vars['CPU_FLAGS'] = ["-mcpu=%s" % cortex]
            build_info['MCU'] = cortex
        else:
            cortex = "cortex-m4"
            self.env_vars['CPU_FLAGS'] = ["-mcpu=%s" % cortex, "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"]
            build_info['MCU'] = cortex

        f.write('''
#ifndef HAL_HAVE_HARDWARE_DOUBLE
#define HAL_HAVE_HARDWARE_DOUBLE 0
#endif
''')

        if self.get_config('MCU_CLOCKRATE_MHZ', required=False):
            clockrate = int(self.get_config('MCU_CLOCKRATE_MHZ'))
            f.write('#define HAL_CUSTOM_MCU_CLOCKRATE %u\n' % (clockrate * 1000000))
            f.write('#define HAL_EXPECTED_SYSCLOCK %u\n' % (clockrate * 1000000))
        elif self.get_mcu_config('EXPECTED_CLOCK', required=True):
            f.write('#define HAL_EXPECTED_SYSCLOCK %u\n' % self.get_mcu_config('EXPECTED_CLOCK'))

        if self.get_mcu_config('EXPECTED_CLOCKS', required=False):
            clockrate = self.get_config('MCU_CLOCKRATE_MHZ', required=False)
            for mcu_clock, mcu_clock_speed in self.get_mcu_config('EXPECTED_CLOCKS'):
                if (mcu_clock == 'STM32_HCLK' or mcu_clock == 'STM32_SYS_CK') and clockrate:
                    f.write('#define HAL_EXPECTED_%s %u\n' % (mcu_clock, int(clockrate) * 1000000))
                else:
                    f.write('#define HAL_EXPECTED_%s %u\n' % (mcu_clock, mcu_clock_speed))

        self.env_vars['CORTEX'] = cortex

        if not self.is_bootloader_fw():
            if cortex == 'cortex-m4':
                self.env_vars['CPU_FLAGS'].append('-DARM_MATH_CM4')
            elif cortex == 'cortex-m7':
                self.env_vars['CPU_FLAGS'].append('-DARM_MATH_CM7')

        if not self.mcu_series.startswith("STM32F1") and not self.is_bootloader_fw():
            self.env_vars['CPU_FLAGS'].append('-u_printf_float')
            build_info['ENV_UDEFS'] = "-DCHPRINTF_USE_FLOAT=1"

        # setup build variables
        for v in build_info.keys():
            self.build_flags.append('%s=%s' % (v, build_info[v]))

        # setup for bootloader build
        if self.is_bootloader_fw():
            f.write('''
#define HAL_BOOTLOADER_BUILD TRUE
#define HAL_USE_ADC FALSE
#define HAL_NO_PRINTF
#define HAL_USE_I2C FALSE
#define HAL_USE_PWM FALSE
#define CH_DBG_ENABLE_STACK_CHECK FALSE
// avoid timer and RCIN threads to save memory
#define HAL_NO_TIMER_THREAD
#define HAL_NO_RCOUT_THREAD
#ifndef HAL_RCIN_THREAD_ENABLED
#define HAL_RCIN_THREAD_ENABLED 0
#endif
#ifndef AP_HAL_SHARED_DMA_ENABLED
#define AP_HAL_SHARED_DMA_ENABLED 0
#endif
#define HAL_NO_ROMFS_SUPPORT TRUE
#define CH_CFG_USE_TM FALSE
#ifndef CH_CFG_USE_REGISTRY
#define CH_CFG_USE_REGISTRY FALSE
#endif
#ifndef CH_CFG_USE_WAITEXIT
#define CH_CFG_USE_WAITEXIT FALSE
#endif
#ifndef CH_CFG_USE_MEMPOOLS
#define CH_CFG_USE_MEMPOOLS FALSE
#endif
#define CH_DBG_FILL_THREADS FALSE
#ifndef CH_CFG_USE_MUTEXES
#define CH_CFG_USE_MUTEXES FALSE
#endif
#ifndef CH_CFG_USE_EVENTS
#define CH_CFG_USE_EVENTS FALSE
#endif
#define CH_CFG_USE_EVENTS_TIMEOUT FALSE
#define CH_CFG_OPTIMIZE_SPEED FALSE
#define HAL_USE_EMPTY_STORAGE 1
#ifndef HAL_STORAGE_SIZE
#define HAL_STORAGE_SIZE 16384
#endif
#define HAL_USE_RTC FALSE
#ifndef CH_CFG_USE_DYNAMIC
#define CH_CFG_USE_DYNAMIC FALSE
#endif
#define STM32_FLASH_DISABLE_ISR 0
#ifndef PAL_USE_CALLBACKS
#define PAL_USE_CALLBACKS FALSE
#endif
''')
            # get bootloader flash space, if larger than 128k we can enable Heap
            flash_size = self.get_config('FLASH_USE_MAX_KB', type=int, default=0)
            if flash_size == 0:
                flash_size = self.get_config('FLASH_SIZE_KB', type=int)
            flash_length = min(flash_size, self.get_config('FLASH_BOOTLOADER_LOAD_KB', type=int))
            if not self.env_vars['EXT_FLASH_SIZE_MB'] and not self.signed_fw and flash_length < 128:
                f.write('''
#ifndef CH_CFG_USE_MEMCORE
#define CH_CFG_USE_MEMCORE FALSE
#endif
#ifndef CH_CFG_USE_SEMAPHORES
#define CH_CFG_USE_SEMAPHORES FALSE
#endif
#ifndef CH_CFG_USE_HEAP
#define CH_CFG_USE_HEAP FALSE
#endif
''')
        if self.env_vars.get('ROMFS_UNCOMPRESSED', False):
            f.write('#define HAL_ROMFS_UNCOMPRESSED\n')

        if not self.is_bootloader_fw():
            f.write('''#define STM32_DMA_REQUIRED TRUE\n\n''')

        if self.is_bootloader_fw():
            # do not enable flash protection in bootloader, even if hwdef
            # requests it:
            f.write('''
#undef HAL_FLASH_PROTECTION
#define HAL_FLASH_PROTECTION 0
''')
        else:
            # flash protection is off by default:
            f.write('''
#ifndef HAL_FLASH_PROTECTION
#define HAL_FLASH_PROTECTION 0
#endif
''')

    def write_ldscript(self, fname):
        '''write ldscript.ld for this board'''
        flash_size = self.get_config('FLASH_USE_MAX_KB', type=int, default=0)
        if flash_size == 0:
            flash_size = self.get_config('FLASH_SIZE_KB', type=int)

        # space to reserve for bootloader and storage at start of flash
        flash_reserve_start = self.get_config(
            'FLASH_RESERVE_START_KB', default=16, type=int)

        storage_flash_page = self.get_storage_flash_page()
        if storage_flash_page is not None:
            offset = self.get_flash_page_offset_kb(storage_flash_page)
            if offset > flash_reserve_start:
                # storage is after flash, need to ensure flash doesn't encroach on it
                flash_size = min(flash_size, offset)
            else:
                # storage is before flash, need to ensure storage fits
                offset2 = self.get_flash_page_offset_kb(storage_flash_page+2)
                if flash_reserve_start < offset2:
                    self.error("Storage overlaps flash")

        self.env_vars['FLASH_RESERVE_START_KB'] = str(flash_reserve_start)

        # space to reserve for storage at end of flash
        flash_reserve_end = self.get_config('FLASH_RESERVE_END_KB', default=0, type=int)

        # space to reserve for bootloader and storage at start of external flash
        ext_flash_reserve_start = self.get_config(
            'EXT_FLASH_RESERVE_START_KB', default=0, type=int)
        self.env_vars['EXT_FLASH_RESERVE_START_KB'] = str(ext_flash_reserve_start)

        # space to reserve for storage at end of flash
        ext_flash_reserve_end = self.get_config('EXT_FLASH_RESERVE_END_KB', default=0, type=int)

        # ram layout
        ram_map = self.get_ram_map()
        instruction_ram = self.get_mcu_config('INSTRUCTION_RAM', False)

        flash_base = 0x08000000 + flash_reserve_start * 1024
        ext_flash_base = 0x90000000 + ext_flash_reserve_start * 1024
        if instruction_ram is not None:
            instruction_ram_base = instruction_ram[0]
            instruction_ram_length = instruction_ram[1]

        ram1_start = 0
        ram1_len = 0
        flash_ram = self.get_mcu_config('FLASH_RAM', False)
        if flash_ram is not None:
            ram1_start = flash_ram[0]
            ram1_len = flash_ram[1] * 1024

        ram2_start = 0
        ram2_len = 0
        data_ram = self.get_mcu_config('DATA_RAM', False)
        if data_ram is not None:
            ram2_start = data_ram[0]
            ram2_len = data_ram[1] * 1024

        # get external flash if any
        ext_flash_size = self.get_config('EXT_FLASH_SIZE_MB', default=0, type=int)
        int_flash_primary = self.get_config('INT_FLASH_PRIMARY', default=False, type=int)

        if not self.is_bootloader_fw():
            flash_length = flash_size - (flash_reserve_start + flash_reserve_end)
            ext_flash_length = ext_flash_size * 1024 - (ext_flash_reserve_start + ext_flash_reserve_end)
        else:
            flash_length = min(flash_size, self.get_config('FLASH_BOOTLOADER_LOAD_KB', type=int))
            ext_flash_length = 0

        self.env_vars['FLASH_TOTAL'] = flash_length * 1024

        self.progress("Generating ldscript.ld")
        f = open(fname, 'w')
        ram0_start = ram_map[0][0]
        ram0_len = ram_map[0][1] * 1024
        if ext_flash_size > 32:
            self.error("We only support 24bit addressing over external flash")

        ram_reserve_start, ram0_start_address = self.get_ram_reserve_start()
        if ram_reserve_start > 0 and ram0_start_address == ram0_start:
            ram0_start += ram_reserve_start
            ram0_len -= ram_reserve_start
        if ext_flash_length == 0 or self.is_bootloader_fw():
            self.env_vars['HAS_EXTERNAL_FLASH_SECTIONS'] = 0
            f.write('''/* generated ldscript.ld */
MEMORY
{
    flash : org = 0x%08x, len = %uK
    ram0  : org = 0x%08x, len = %u
}

INCLUDE common.ld
''' % (flash_base, flash_length, ram0_start, ram0_len))
        elif int_flash_primary:
            self.env_vars['HAS_EXTERNAL_FLASH_SECTIONS'] = 1
            f.write('''/* generated ldscript.ld */
MEMORY
{
    flash : org = 0x%08x, len = %uK
    ext_flash : org = 0x%08x, len = %uK
    ram0  : org = 0x%08x, len = %u
}

INCLUDE common_mixf.ld
''' % (flash_base, flash_length, ext_flash_base, ext_flash_length, ram0_start, ram0_len))
        else:
            self.env_vars['HAS_EXTERNAL_FLASH_SECTIONS'] = 1
            self.build_flags.append('COPY_VECTORS_TO_RAM=yes')
            f.write('''/* generated ldscript.ld */
MEMORY
{
    default_flash (rx) : org = 0x%08x, len = %uK
    instram : org = 0x%08x, len = %uK
    ram0  : org = 0x%08x, len = %u
    flashram  : org = 0x%08x, len = %u
    dataram  : org = 0x%08x, len = %u
}

INCLUDE common.ld
''' % (ext_flash_base, ext_flash_length, instruction_ram_base, instruction_ram_length, ram0_start, ram0_len, ram1_start, ram1_len, ram2_start, ram2_len))  # noqa
        f.close()

    def copy_common_linkerscript(self, outpath):
        dirpath = os.path.dirname(os.path.realpath(__file__))

        if self.is_bootloader_fw():
            linker = 'common.ld'
        else:
            linker = self.get_mcu_config('LINKER_CONFIG')
        if linker is None:
            if not self.get_config('EXT_FLASH_SIZE_MB', default=0, type=int):
                linker = 'common.ld'
            elif self.get_config('INT_FLASH_PRIMARY', default=False, type=int):
                linker = 'common_mixf.ld'
            else:
                linker = 'common_extf.ld'
        shutil.copy(os.path.join(dirpath, "../common", linker), outpath)

    def get_USB_IDs(self):
        '''return tuple of USB VID/PID'''

        if self.dual_USB_enabled:
            # use pidcodes allocated ID
            default_vid = 0x1209
            default_pid = 0x5740
        else:
            default_vid = 0x1209
            default_pid = 0x5741
        return (self.get_config('USB_VENDOR', type=int, default=default_vid),
                self.get_config('USB_PRODUCT', type=int, default=default_pid))

    def write_USB_config(self, f):
        '''write USB config defines'''
        if not self.have_type_prefix('OTG'):
            return
        f.write('// USB configuration\n')
        (USB_VID, USB_PID) = self.get_USB_IDs()
        f.write('#define HAL_USB_VENDOR_ID 0x%04x\n' % int(USB_VID))
        f.write('#define HAL_USB_PRODUCT_ID 0x%04x\n' % int(USB_PID))
        f.write('#define HAL_USB_STRING_MANUFACTURER %s\n' %
                self.get_config("USB_STRING_MANUFACTURER", default="\"ArduPilot\""))
        default_product = "%BOARD%"
        if self.is_bootloader_fw():
            default_product += "-BL"
        product_string = self.get_config("USB_STRING_PRODUCT", default="\"%s\"" % default_product)
        if self.is_bootloader_fw() and self.signed_fw:
            product_string = product_string.replace("-BL", "-Secure-BL-v10")
        f.write('#define HAL_USB_STRING_PRODUCT %s\n' % product_string)

        f.write('#define HAL_USB_STRING_SERIAL %s\n' % self.get_config("USB_STRING_SERIAL", default="\"%SERIAL%\""))

        f.write('\n\n')

    def write_SPI_table(self, f):
        '''write SPI device table'''
        f.write('\n// SPI device table\n')
        devlist = []
        for dev in self.spidev:
            if len(dev) != 7:
                self.error("Badly formed SPIDEV line %s" % dev)
            name = '"' + dev[0] + '"'
            bus = dev[1]
            devid = dev[2]
            cs = dev[3]
            mode = dev[4]
            lowspeed = dev[5]
            highspeed = dev[6]
            if not bus.startswith('SPI') or bus not in self.spi_list:
                self.error("Bad SPI bus in SPIDEV line %s" % dev)
            if not devid.startswith('DEVID') or not self.is_int(devid[5:]):
                self.error("Bad DEVID in SPIDEV line %s" % dev)
            if cs not in self.bylabel or not self.bylabel[cs].is_CS():
                self.error("Bad CS pin in SPIDEV line %s" % dev)
            if mode not in ['MODE0', 'MODE1', 'MODE2', 'MODE3']:
                self.error("Bad MODE in SPIDEV line %s" % dev)
            if not lowspeed.endswith('*MHZ') and not lowspeed.endswith('*KHZ'):
                self.error("Bad lowspeed value %s in SPIDEV line %s" % (lowspeed, dev))
            if not highspeed.endswith('*MHZ') and not highspeed.endswith('*KHZ'):
                self.error("Bad highspeed value %s in SPIDEV line %s" %
                           (highspeed, dev))
            cs_pin = self.bylabel[cs]
            pal_line = 'PAL_LINE(GPIO%s,%uU)' % (cs_pin.port, cs_pin.pin)
            devidx = len(devlist)
            f.write(
                '#define HAL_SPI_DEVICE%-2u SPIDesc(%-17s, %2u, %2u, %-19s, SPIDEV_%s, %7s, %7s)\n'
                % (devidx, name, self.spi_list.index(bus), int(devid[5:]), pal_line,
                   mode, lowspeed, highspeed))
            devlist.append('HAL_SPI_DEVICE%u' % devidx)
        self.write_device_table(f, 'spi devices', 'HAL_SPI_DEVICE_LIST', devlist)
        for dev in self.spidev:
            f.write("#define HAL_WITH_SPI_%s 1\n" % dev[0].upper().replace("-", "_"))
        f.write("\n")

    def write_SPI_config(self, f):
        '''write SPI config defines'''
        for t in list(self.bytype.keys()) + list(self.alttype.keys()):
            if t.startswith('SPI'):
                self.spi_list.append(t)
        self.spi_list = sorted(self.spi_list)
        if len(self.spidev) != 0 and len(self.spi_list) == 0:
            self.error("Have SPI devices but no SPI bus?!")
        if len(self.spidev) == 0:
            f.write('#define HAL_USE_SPI FALSE\n')
            return
        devlist = []
        for dev in self.spi_list:
            n = int(dev[3:])
            devlist.append('HAL_SPI%u_CONFIG' % n)
            sck_pin = self.bylabel['SPI%s_SCK' % n]
            sck_line = 'PAL_LINE(GPIO%s,%uU)' % (sck_pin.port, sck_pin.pin)
            f.write(
                '#define HAL_SPI%u_CONFIG { &SPID%u, %u, STM32_SPI_SPI%u_DMA_STREAMS, %s }\n'
                % (n, n, n, n, sck_line))
        f.write('#define HAL_SPI_BUS_LIST %s\n\n' % ','.join(devlist))
        self.write_SPI_table(f)

    def write_WSPI_table(self, f):
        '''write SPI device table'''
        f.write('\n// WSPI device table\n')
        devlist = []
        for dev in self.wspidev:
            if len(dev) != 6:
                self.progress("Badly formed WSPIDEV line %s" % dev)
            name = '"' + dev[0] + '"'
            bus = dev[1]
            mode = dev[2]
            speed = dev[3]
            size_pow2 = dev[4]
            ncs_clk_delay = dev[5]
            if not bus.startswith('QUADSPI') and not bus.startswith('OCTOSPI') or bus not in self.wspi_list:
                self.error("Bad QUADSPI/OCTOSPI bus in QSPIDEV line %s" % dev)
            if mode not in ['MODE1', 'MODE3']:
                self.error("Bad MODE in WSPIDEV line %s" % dev)
            if not speed.endswith('*MHZ') and not speed.endswith('*KHZ'):
                self.error("Bad speed value %s in WSPIDEV line %s" % (speed, dev))

            devidx = len(devlist)
            f.write(
                '#define HAL_WSPI_DEVICE%-2u WSPIDesc(%-17s, %2u, WSPIDEV_%s, %7s, %2u, %2u)\n'
                % (devidx, name, self.wspi_list.index(bus), mode, speed, int(size_pow2), int(ncs_clk_delay)))
            devlist.append('HAL_WSPI_DEVICE%u' % devidx)
        self.write_device_table(f, "wspi devices", "HAL_WSPI_DEVICE_LIST", devlist)
        for dev in self.wspidev:
            f.write("#define HAL_HAS_WSPI_%s 1\n" % dev[0].upper().replace("-", "_"))
            if dev[1].startswith('QUADSPI'):
                f.write("#define HAL_QSPI%d_CLK (%s)" % (int(bus[7:]), speed))
            else:
                f.write("#define HAL_OSPI%d_CLK (%s)" % (int(bus[7:]), speed))
        f.write("\n")

    def write_WSPI_config(self, f):
        '''write SPI config defines'''
        # only the bootloader must run the hal lld (and QSPI clock) otherwise it is not possible to
        # bootstrap into external flash
        for t in list(self.bytype.keys()) + list(self.alttype.keys()):
            if (t.startswith('QUADSPI') or t.startswith('OCTOSPI')) and not self.is_bootloader_fw():
                f.write('#define HAL_XIP_ENABLED TRUE\n')

        if len(self.wspidev) == 0:
            # nothing else to do
            return

        for t in list(self.bytype.keys()) + list(self.alttype.keys()):
            self.progress(t)
            if t.startswith('QUADSPI') or t.startswith('OCTOSPI'):
                self.wspi_list.append(t)

        wspi_list = sorted(self.wspi_list)
        if len(wspi_list) == 0:
            return
        f.write('#define HAL_USE_WSPI TRUE\n')
        devlist = []
        for dev in wspi_list:
            n = int(dev[7:])
            devlist.append('HAL_WSPI%u_CONFIG' % n)
            f.write(
                '#define HAL_WSPI%u_CONFIG { &WSPID%u, %u}\n'
                % (n, n, n))
        f.write('#define HAL_WSPI_BUS_LIST %s\n\n' % ','.join(devlist))
        self.write_WSPI_table(f)

    def write_check_firmware(self, f):
        '''add AP_CHECK_FIRMWARE_ENABLED if needed'''
        if self.is_periph_fw() or self.intdefines.get('AP_OPENDRONEID_ENABLED', 0) == 1:
            f.write('''
#ifndef AP_CHECK_FIRMWARE_ENABLED
#define AP_CHECK_FIRMWARE_ENABLED 1
#endif
''')

    def write_AIRSPEED_config(self, f):
        '''write airspeed config defines'''
        devlist = []
        seen = set()
        idx = 0
        for dev in self.airspeed_list:
            if self.seen_str(dev) in seen:
                self.error("Duplicate AIRSPEED: %s" % self.seen_str(dev))
            seen.add(self.seen_str(dev))
            driver = dev[0]
            wrapper = ''
            a = driver.split(':')
            driver = a[0]
            for i in range(1, len(dev)):
                if dev[i].startswith("SPI:"):
                    dev[i] = self.parse_spi_device(dev[i])
                elif dev[i].startswith("I2C:"):
                    (wrapper, dev[i]) = self.parse_i2c_device(dev[i])
            n = len(devlist)+1
            devlist.append('HAL_AIRSPEED_PROBE%u' % n)
            args = ['*this', str(idx)] + dev[1:]
            f.write(
                '#define HAL_AIRSPEED_PROBE%u %s ADD_BACKEND(AP_Airspeed_%s::probe(%s))\n'
                % (n, wrapper, driver, ','.join(args)))
            idx += 1
        if len(devlist) > 0:
            f.write('#define HAL_AIRSPEED_PROBE_LIST %s\n\n' % ';'.join(devlist))

    def write_DATAFLASH_config(self, f):
        '''write dataflash config defines'''
        # DATAFLASH block|littlefs:<w25nxx|jedec_nor>
        seen = set()
        for dev in self.dataflash_list:
            if not self.has_dataflash_spi():
                self.error("Missing DATAFLASH device: %s" % self.seen_str(dev))
            if self.seen_str(dev) in seen:
                self.error("Duplicate DATAFLASH: %s" % self.seen_str(dev))
            seen.add(self.seen_str(dev))
            a = dev[0].split(':')
            if a[0].startswith('block'):
                if len(a) > 1 and a[1].startswith('w25nxx'):
                    f.write('#define HAL_LOGGING_DATAFLASH_DRIVER AP_Logger_W25NXX\n')
                elif len(a) > 1 and a[1].startswith('jedec_nor'):
                    f.write('#define HAL_LOGGING_DATAFLASH_DRIVER AP_Logger_Flash_JEDEC\n')
                f.write('#define HAL_LOGGING_DATAFLASH_ENABLED TRUE\n')
            elif a[0].startswith('littlefs'):
                f.write('#define USE_POSIX\n')
                f.write('#define HAL_OS_LITTLEFS_IO TRUE\n')
                f.write('#define HAL_OS_POSIX_IO TRUE\n')
                if len(a) > 1 and a[1].startswith('w25nxx'):
                    f.write('#define AP_FILESYSTEM_LITTLEFS_FLASH_TYPE AP_FILESYSTEM_FLASH_W25NXX\n')
                elif len(a) > 1 and a[1].startswith('jedec_nor'):
                    f.write('#define AP_FILESYSTEM_LITTLEFS_FLASH_TYPE AP_FILESYSTEM_FLASH_JEDEC_NOR\n')
                self.build_flags.append('USE_FATFS=no')
                self.env_vars['WITH_LITTLEFS'] = "1"

    def write_board_validate_macro(self, f):
        '''write board validation macro'''
        validate_string = ''
        validate_dict = {}
        if 'BOARD_VALIDATE' in self.config:
            for check in self.config['BOARD_VALIDATE']:
                check_name = check
                check_string = check
                while True:
                    def substitute_alias(m):
                        return '(' + self.get_config(m.group(1), spaces=True) + ')'
                    output = re.sub(r'\$(\w+|\{([^}]*)\})', substitute_alias, check_string)
                    if (output == check_string):
                        break
                    check_string = output
                validate_dict[check_name] = check_string
            # Finally create check conditional
            for check_name in sorted(validate_dict.keys()):
                validate_string += "!" + validate_dict[check_name] + "?" + "\"" + check_name + "\"" + ":"
            validate_string += "nullptr"
            f.write('#define HAL_VALIDATE_BOARD (%s)\n\n' % validate_string)

    def get_gpio_bylabel(self, label):
        '''get GPIO(n) setting on a pin label, or -1'''
        p = self.bylabel.get(label)
        if p is None:
            return -1
        return p.extra_value('GPIO', type=int, default=-1)

    def get_extra_bylabel(self, label, name, default=None):
        '''get extra setting for a label by name'''
        p = self.bylabel.get(label)
        if p is None:
            return default
        return p.extra_value(name, type=str, default=default)

    def write_UART_config(self, f):
        '''write UART config defines'''
        serial_list = self.get_config('SERIAL_ORDER', required=False, aslist=True)
        hide_iomcu_uart = False
        if 'IOMCU_UART' in self.config:
            hide_iomcu_uart = self.config['IOMCU_UART'][0] not in serial_list

        if 'IOMCU_UART' in self.config and self.config['IOMCU_UART'][0] not in serial_list:
            serial_list.append(self.config['IOMCU_UART'][0])
        if serial_list is None:
            return
        while len(serial_list) < 3: # enough ports for CrashCatcher UART discovery
            serial_list += ['EMPTY']
        f.write('\n// UART configuration\n')

        # write out which serial ports we actually have
        nports = 0
        for idx, serial in enumerate(serial_list):
            if hide_iomcu_uart and self.config['IOMCU_UART'][0] == serial:
                # IOMCU UART is not to be displayed in the serial parameters
                f.write('#define HAL_HAVE_SERIAL%u 1\n' % idx)
                f.write('#define HAL_HAVE_SERIAL%u_PARAMS 0\n' % idx)
                continue
            if serial == 'EMPTY':
                f.write('#define HAL_HAVE_SERIAL%u 0\n' % idx)
            else:
                f.write('#define HAL_HAVE_SERIAL%u 1\n' % idx)
                f.write('#define HAL_HAVE_SERIAL%u_PARAMS 1\n' % idx)
                nports = nports + 1
        f.write('#define HAL_NUM_SERIAL_PORTS %u\n' % nports)

        # write out driver declarations for HAL_ChibOS_Class.cpp
        sdev = 0
        for idx, dev in enumerate(serial_list):
            if dev == 'EMPTY':
                f.write('#define HAL_SERIAL%s_DRIVER Empty::UARTDriver serial%sDriver\n' %
                        (idx, idx))
                sdev += 1
            else:
                f.write(
                    '#define HAL_SERIAL%s_DRIVER ChibiOS::UARTDriver serial%sDriver(%u)\n'
                    % (idx, idx, sdev))
                sdev += 1
        for idx in range(len(serial_list), 10):
            f.write('#define HAL_SERIAL%s_DRIVER Empty::UARTDriver serial%sDriver\n' %
                    (idx, idx))

        if 'IOMCU_UART' in self.config:
            if 'io_firmware.bin' not in self.romfs:
                self.error("Need io_firmware.bin in ROMFS for IOMCU")

            self.write_defaulting_define(f, 'HAL_WITH_IO_MCU', 1)

            if self.config['IOMCU_UART'][0]:
                # get index of serial port in serial_list
                index = serial_list.index(self.config['IOMCU_UART'][0])
                f.write('#define HAL_UART_IOMCU_IDX %u\n' % int(index))
                f.write(
                    '#define HAL_UART_IO_DRIVER constexpr ChibiOS::UARTDriver &uart_io = serial%sDriver;\n' % (index)
                )

            f.write('#define HAL_HAVE_SERVO_VOLTAGE 1\n') # make the assumption that IO guarantees servo monitoring
            # all IOMCU capable boards have SBUS out
            f.write('#define AP_FEATURE_SBUS_OUT 1\n')
        else:
            f.write('#define HAL_WITH_IO_MCU 0\n')
        f.write('\n')

        need_uart_driver = False
        OTG2_index = None
        devlist = []
        have_rts_cts = False
        have_low_noise = False
        crash_uart = None

        # write config for CrashCatcher UART
        if not serial_list[0].startswith('OTG') and not serial_list[0].startswith('EMPTY'):
            crash_uart = serial_list[0]
        elif not serial_list[2].startswith('OTG') and not serial_list[2].startswith('EMPTY'):
            crash_uart = serial_list[2]

        if crash_uart is not None and self.get_config('FLASH_SIZE_KB', type=int) >= 2048:
            f.write('#define HAL_CRASH_SERIAL_PORT %s\n' % crash_uart)
            f.write('#define IRQ_DISABLE_HAL_CRASH_SERIAL_PORT() nvicDisableVector(STM32_%s_NUMBER)\n' % crash_uart)
            f.write('#define RCC_RESET_HAL_CRASH_SERIAL_PORT() rccReset%s(); rccEnable%s(true)\n' % (crash_uart, crash_uart))
            f.write('#define HAL_CRASH_SERIAL_PORT_CLOCK STM32_%sCLK\n' % crash_uart)
        # check if we have a UART with a low noise RX pin
        for num, dev in enumerate(serial_list):
            if not dev.startswith('UART') and not dev.startswith('USART'):
                continue
            rx_port = dev + '_RX'
            if rx_port in self.bylabel and self.bylabel[rx_port].has_extra('LOW_NOISE'):
                have_low_noise = True
                break
        for num, dev in enumerate(serial_list):
            if dev.startswith('UART'):
                n = int(dev[4:])
            elif dev.startswith('USART'):
                n = int(dev[5:])
            elif dev.startswith('OTG'):
                n = int(dev[3:])
            elif dev.startswith('EMPTY'):
                devlist.append('{}')
                continue
            else:
                self.error("Invalid element %s in SERIAL_ORDER" % dev)
            devlist.append('HAL_%s_CONFIG' % dev)
            tx_line = self.make_line(dev + '_TX')
            rx_line = self.make_line(dev + '_RX')
            rts_line_name = dev + '_RTS'
            rts_line = self.make_line(rts_line_name)
            cts_line = self.make_line(dev + '_CTS')
            if cts_line == "0":
                cts_line = self.make_line(dev + '_CTS_GPIO')
            if rts_line != "0":
                have_rts_cts = True
                f.write('#define HAL_HAVE_RTSCTS_SERIAL%u\n' % num)

            if dev.startswith('OTG2'):
                f.write(
                    '#define HAL_%s_CONFIG {(BaseSequentialStream*) &SDU2, 2, true, false, 0, 0, false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, UINT8_MAX,' % dev)  # noqa
                if have_low_noise:
                    f.write('false}\n')
                else:
                    f.write('}\n')
                OTG2_index = serial_list.index(dev)
                self.dual_USB_enabled = True
            elif dev.startswith('OTG'):
                f.write(
                    '#define HAL_%s_CONFIG {(BaseSequentialStream*) &SDU1, 1, true, false, 0, 0, false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, UINT8_MAX,' % dev)  # noqa
                if have_low_noise:
                    f.write('false}\n')
                else:
                    f.write('}\n')
            else:
                need_uart_driver = True
                f.write(
                    "#define HAL_%s_CONFIG { (BaseSequentialStream*) &SD%u, %u, false, "
                    % (dev, n, n))
                if self.mcu_series.startswith("STM32F1"):
                    f.write("%s, %s, %s, %s, " % (tx_line, rx_line, rts_line, cts_line))
                else:
                    f.write("STM32_%s_RX_DMA_CONFIG, STM32_%s_TX_DMA_CONFIG, %s, %s, %s, %s, " %
                            (dev, dev, tx_line, rx_line, rts_line, cts_line))

                # add inversion pins, if any
                f.write("%d, " % self.get_gpio_bylabel(dev + "_RXINV"))
                f.write("%s, " % self.get_extra_bylabel(dev + "_RXINV", "POL", "0"))
                f.write("%d, " % self.get_gpio_bylabel(dev + "_TXINV"))
                f.write("%s, " % self.get_extra_bylabel(dev + "_TXINV", "POL", "0"))

                # USB endpoint ID, not used
                f.write("0, ")

                # Find and add RTS alt function number if available
                def get_RTS_alt_function():
                    # Typically we do software RTS control, so there is
                    # no requirement for the pin to have valid UART
                    # RTS alternative function
                    # If it does this enables hardware flow control for RS-485
                    lib = self.get_mcu_lib(self.mcu_type)
                    if (rts_line == "0") or (rts_line_name not in self.bylabel) or not hasattr(lib, "AltFunction_map"):
                        # No pin, 0 is a valid alt function, use UINT8_MAX for invalid
                        return "UINT8_MAX"

                    pin = self.bylabel[rts_line_name]
                    for label in self.af_labels:
                        if rts_line_name.startswith(label):
                            s = pin.portpin + ":" + rts_line_name
                            if s not in lib.AltFunction_map:
                                return "UINT8_MAX"
                            return lib.AltFunction_map[s]
                if have_low_noise:
                    low_noise = 'false'
                    rx_port = dev + '_RX'
                    if rx_port in self.bylabel and self.bylabel[rx_port].has_extra('LOW_NOISE'):
                        low_noise = 'true'
                    f.write("%s, %s}\n" % (get_RTS_alt_function(), low_noise))
                else:
                    f.write("%s}\n" % get_RTS_alt_function())

        if have_low_noise:
            f.write('#define HAL_HAVE_LOW_NOISE_UART 1\n')
        if have_rts_cts:
            f.write('#define AP_FEATURE_RTSCTS 1\n')
        if OTG2_index is not None:
            f.write('#define HAL_OTG2_UART_INDEX %d\n' % OTG2_index)
            f.write('#define HAL_HAVE_DUAL_USB_CDC 1\n')
            if not self.is_periph_fw():
                f.write('''
#if defined(HAL_NUM_CAN_IFACES) && HAL_NUM_CAN_IFACES
#ifndef HAL_OTG2_PROTOCOL
#define HAL_OTG2_PROTOCOL SerialProtocol_MAVLink2
#endif
#define DEFAULT_SERIAL%d_PROTOCOL HAL_OTG2_PROTOCOL
#define DEFAULT_SERIAL%d_BAUD 115200
#endif
''' % (OTG2_index, OTG2_index))

        self.write_device_table(f, "serial devices", "HAL_SERIAL_DEVICE_LIST", devlist)
        if not need_uart_driver and not self.is_bootloader_fw():
            f.write('''
#ifndef HAL_USE_SERIAL
#define HAL_USE_SERIAL HAL_USE_SERIAL_USB
#endif
''')
        num_ports = len(devlist)
        if num_ports > 10:
            self.error("Exceeded max num SERIALs of 10 (%u)" % num_ports)
        f.write('#define HAL_UART_NUM_SERIAL_PORTS %u\n' % num_ports)

    def write_UART_config_bootloader(self, f):
        '''write UART config defines'''
        serial_list = self.get_config('SERIAL_ORDER', required=False, aslist=True)
        if serial_list is None:
            return
        f.write('\n// UART configuration\n')
        devlist = []
        have_serial = False
        OTG2_index = None
        for s in serial_list:
            if s.startswith('OTG2'):
                devlist.append('(BaseChannel *)&SDU2')
                OTG2_index = serial_list.index(s)
            elif s.startswith('OTG'):
                devlist.append('(BaseChannel *)&SDU1')
            else:
                snum = int(s[-1])
                devlist.append('(BaseChannel *)&SD%u' % snum)
                have_serial = True
        if len(devlist) > 0:
            f.write('#define BOOTLOADER_DEV_LIST %s\n' % ','.join(devlist))
        if OTG2_index is not None:
            f.write('#define HAL_OTG2_UART_INDEX %d\n' % OTG2_index)
        if not have_serial:
            f.write('''
#ifndef HAL_USE_SERIAL
#define HAL_USE_SERIAL FALSE
#endif
''')

    def write_I2C_config(self, f):
        '''write I2C config defines'''
        if not self.have_type_prefix('I2C'):
            self.progress("No I2C peripherals")
            f.write('''
#ifndef HAL_USE_I2C
#define HAL_USE_I2C FALSE
#endif
''')
            return
        if 'I2C_ORDER' not in self.config:
            self.error("Missing I2C_ORDER config")
        i2c_list = self.config['I2C_ORDER']
        f.write('// I2C configuration\n')
        if len(i2c_list) == 0:
            self.error("I2C_ORDER invalid")
        devlist = []

        # write out config structures
        for dev in i2c_list:
            if not dev.startswith('I2C') or dev[3] not in "1234":
                self.error("Bad I2C_ORDER element %s" % dev)
            n = int(dev[3:])
            devlist.append('HAL_I2C%u_CONFIG' % n)
            sda_line = self.make_line('I2C%u_SDA' % n)
            scl_line = self.make_line('I2C%u_SCL' % n)
            f.write('''
#if defined(STM32_I2C_I2C%u_RX_DMA_STREAM) && defined(STM32_I2C_I2C%u_TX_DMA_STREAM)
#define HAL_I2C%u_CONFIG { &I2CD%u, %u, STM32_I2C_I2C%u_RX_DMA_STREAM, STM32_I2C_I2C%u_TX_DMA_STREAM, %s, %s }
#else
#define HAL_I2C%u_CONFIG { &I2CD%u, %u, SHARED_DMA_NONE, SHARED_DMA_NONE, %s, %s }
#endif
'''
                    % (n, n, n, n, n, n, n, scl_line, sda_line, n, n, n, scl_line, sda_line))
        f.write('\n')
        self.write_device_table(f, "i2c devices", "HAL_I2C_DEVICE_LIST", devlist)

    def parse_timer(self, str):
        '''parse timer channel string, i.e TIM8_CH2N'''
        result = re.match(r'TIM([0-9]*)_CH([1234])(N?)', str)
        if result:
            tim = int(result.group(1))
            chan = int(result.group(2))
            compl = result.group(3) == 'N'
            if tim < 1 or tim > 17:
                self.error("Bad timer number %s in %s" % (tim, str))
            return (tim, chan, compl)
        else:
            self.error("Bad timer definition %s" % str)

    def write_PWM_config(self, f, ordered_timers):
        '''write PWM config defines'''
        rc_in = None
        rc_in_int = None
        alarm = None
        bidir = None
        up_shared = None
        pwm_out = []
        # start with the ordered list from the dma resolver
        pwm_timers = ordered_timers
        for label in self.bylabel.keys():
            p = self.bylabel[label]
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
                    if p.has_extra('BIDIR'):
                        bidir = p
                    if p.has_extra('UP_SHARED'):
                        up_shared = p
                    if p.type not in pwm_timers:
                        pwm_timers.append(p.type)

        f.write('#define HAL_PWM_COUNT %u\n' % len(pwm_out))
        if not pwm_out and not alarm:
            self.progress("No PWM output defined")
            f.write('''
#ifndef HAL_USE_PWM
#define HAL_USE_PWM FALSE
#endif
''')

        if rc_in is not None:
            (n, chan, compl) = self.parse_timer(rc_in.label)
            if compl:
                # it is an inverted channel
                f.write('#define HAL_RCIN_IS_INVERTED\n')
            if chan not in [1, 2]:
                self.error(
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
            (n, chan, compl) = self.parse_timer(rc_in_int.label)
            if compl:
                self.error('Complementary channel is not supported for RCININT %s' % rc_in_int.label)
            f.write('// RC input config\n')
            f.write('#define HAL_USE_EICU TRUE\n')
            f.write('#define STM32_EICU_USE_TIM%u TRUE\n' % n)
            f.write('#define RCININT_EICU_TIMER EICUD%u\n' % n)
            f.write('#define RCININT_EICU_CHANNEL EICU_CHANNEL_%u\n' % chan)
            f.write('\n')

        if alarm is not None:
            (n, chan, compl) = self.parse_timer(alarm.label)
            if compl:
                self.error("Complementary channel is not supported for ALARM %s" % alarm.label)
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
        if bidir is not None:
            f.write('#define HAL_WITH_BIDIR_DSHOT\n')
        if up_shared is not None:
            f.write('#define HAL_TIM_UP_SHARED\n')
            for t in self.shared_up:
                f.write('#define HAL_%s_SHARED true\n' % t)
        for t in pwm_timers:
            n = int(t[3:])
            f.write('#define STM32_PWM_USE_TIM%u TRUE\n' % n)
            f.write('#define STM32_TIM%u_SUPPRESS_ISR\n' % n)
        f.write('\n')
        f.write('// PWM output config\n')
        groups = []
        # complementary channels require advanced features
        # which are only available on timers 1 and 8
        need_advanced = False

        for t in pwm_timers:
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
                (n, chan, compl) = self.parse_timer(p.label)
                pwm = p.extra_value('PWM', type=int)
                chan_list[chan - 1] = pwm - 1
                if compl:
                    chan_mode[chan - 1] = 'PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH'
                else:
                    chan_mode[chan - 1] = 'PWM_OUTPUT_ACTIVE_HIGH'
                alt_functions[chan - 1] = p.af
                pal_lines[chan - 1] = 'PAL_LINE(GPIO%s,%uU)' % (p.port, p.pin)
            groups.append('HAL_PWM_GROUP%u' % group)
            if n in [1, 8]:
                # only the advanced timers do 8MHz clocks
                need_advanced = True
                advanced_timer = 'true'
            else:
                advanced_timer = 'false'
            pwm_clock = 1000000
            period = 20000 * pwm_clock / 1000000
            hal_icu_def = ''
            hal_icu_cfg = ''
            if bidir is not None:
                hal_icu_cfg = '\n          {'
                hal_icu_def = '\n'
                for i in range(1, 5):
                    hal_icu_cfg += '{HAL_IC%u_CH%u_DMA_CONFIG},' % (n, i)
                    hal_icu_def += '''#if defined(STM32_TIM_TIM%u_CH%u_DMA_STREAM) && defined(STM32_TIM_TIM%u_CH%u_DMA_CHAN)
# define HAL_IC%u_CH%u_DMA_CONFIG true, STM32_TIM_TIM%u_CH%u_DMA_STREAM, STM32_TIM_TIM%u_CH%u_DMA_CHAN
#else
# define HAL_IC%u_CH%u_DMA_CONFIG false, 0, 0
#endif
''' % (n, i, n, i, n, i, n, i, n, i, n, i)
                if up_shared is not None:
                    hal_icu_cfg += '}, HAL_TIM%u_UP_SHARED, \\' % n
                else:
                    hal_icu_cfg += '}, \\'

            f.write('''#if defined(STM32_TIM_TIM%u_UP_DMA_STREAM) && defined(STM32_TIM_TIM%u_UP_DMA_CHAN)
# define HAL_PWM%u_DMA_CONFIG true, STM32_TIM_TIM%u_UP_DMA_STREAM, STM32_TIM_TIM%u_UP_DMA_CHAN
#else
# define HAL_PWM%u_DMA_CONFIG false, 0, 0
#endif\n%s''' % (n, n, n, n, n, n, hal_icu_def))
            f.write('''#if !defined(HAL_TIM%u_UP_SHARED)
#define HAL_TIM%u_UP_SHARED false
#endif\n''' % (n, n))
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
          }, 0, 0}, &PWMD%u, %u, \\
          HAL_PWM%u_DMA_CONFIG, \\%s
          { %u, %u, %u, %u }, \\
          { %s, %s, %s, %s }}\n''' %
                    (group, advanced_timer,
                     chan_list[0], chan_list[1], chan_list[2], chan_list[3],
                     pwm_clock, period,
                     chan_mode[0], chan_mode[1], chan_mode[2], chan_mode[3],
                     n, n, n, hal_icu_cfg,
                     alt_functions[0], alt_functions[1], alt_functions[2], alt_functions[3],
                     pal_lines[0], pal_lines[1], pal_lines[2], pal_lines[3]))
        f.write('#define HAL_PWM_GROUPS %s\n\n' % ','.join(groups))
        if need_advanced:
            f.write('#define STM32_PWM_USE_ADVANCED TRUE\n')

    def write_ADC_config(self, f):
        '''write ADC config defines'''
        f.write('// ADC config\n')
        adc_chans = [[], [], []]
        analogset = {252, 253, 254} # reserved values for VSENSE, VREF and VBAT in H7
        for label in self.bylabel:
            p = self.bylabel[label]
            if not p.type.startswith('ADC'):
                continue
            if p.type.startswith('ADC1'):
                index = 0
                chan = self.get_ADC1_chan(self.mcu_type, p.portpin)
            elif p.type.startswith('ADC2'):
                index = 1
                chan = self.get_ADC2_chan(self.mcu_type, p.portpin)
            elif p.type.startswith('ADC3'):
                index = 2
                chan = self.get_ADC3_chan(self.mcu_type, p.portpin)
            else:
                self.error("Unknown ADC type %s" % p.type)
            scale = p.extra_value('SCALE', default=None)
            analog = p.extra_value('ANALOG', type=int, default=chan) # default to ADC channel if not specified
            if analog in analogset:
                self.error("Duplicate analog pin %u" % analog)
            analogset.add(analog)
            if p.label == 'VDD_5V_SENS':
                f.write('#define ANALOG_VCC_5V_PIN %u\n' % (analog))
                f.write('#define HAL_HAVE_BOARD_VOLTAGE 1\n')
            if p.label == 'FMU_SERVORAIL_VCC_SENS':
                f.write('#define FMU_SERVORAIL_ADC_PIN %u\n' % (analog))
                f.write('#define HAL_HAVE_SERVO_VOLTAGE 1\n')
            adc_chans[index].append((chan, analog, scale, p.label, p.portpin))

        # sort by ADC channel
        for index in range(3):
            adc_chans[index] = sorted(adc_chans[index])

        if len(adc_chans[1]) > 0:
            # ensure ADC1 and ADC2 are of same size
            # add dummy channels that are not already in adc_chans[1]
            for chan in [c[0] for c in adc_chans[0]]:
                if chan not in [c[0] for c in adc_chans[1]]:
                    adc_chans[1].append((chan, 255, None, 'dummy', 'dummy'))
            # add dummy channels that are not already in adc_chans[0]
            for chan in [c[0] for c in adc_chans[1]]:
                if chan not in [c[0] for c in adc_chans[0]]:
                    adc_chans[0].append((chan, 255, None, 'dummy', 'dummy'))
            # check if ADC1 and ADC2 list if they have the same channel for same index
            # if not then jumble the channels around to have no matching channels
            for i in range(len(adc_chans[0])):
                if adc_chans[0][i][0] == adc_chans[1][i][0]:
                    # found a match, jumble the channels around
                    for j in range(len(adc_chans[0])):
                        if adc_chans[0][j][0] != adc_chans[1][j][0]:
                            # found a non-match, swap the channels
                            adc_chans[0][i], adc_chans[0][j] = adc_chans[0][j], adc_chans[0][i]
                            break

        vdd = self.get_config('STM32_VDD', default='330U')
        if vdd[-1] == 'U':
            vdd = vdd[:-1]
        vdd = float(vdd) * 0.01
        f.write('#define HAL_ANALOG_PINS \\\n')
        for (chan, analog, scale, label, portpin) in adc_chans[0]:
            scale_str = '%.2f/4096' % vdd
            if scale is not None and scale != '1':
                scale_str = scale + '*' + scale_str
            f.write('{ %2u, %2u, %12s }, /* %s %s */ \\\n' %
                    (chan, analog, scale_str,  portpin, label))
        f.write('\n\n')
        if len(adc_chans[1]) > 0:
            f.write('#define STM32_ADC_SAMPLES_SIZE 32\n')
            f.write('#define ADC12_CCR_DUAL ADC_CCR_DUAL_REG_INTERL\n')
            f.write('#define STM32_ADC_DUAL_MODE TRUE\n')
            f.write('#define HAL_ANALOG2_PINS \\\n')
            for (chan, analog, scale, label, portpin) in adc_chans[1]:
                scale_str = '%.2f/4096' % vdd
                if scale is not None and scale != '1':
                    scale_str = scale + '*' + scale_str
                f.write('{ %2u, %2u, %12s }, /* %s %s */ \\\n' %
                        (chan, analog, scale_str, portpin, label))
            f.write('\n\n')
        if len(adc_chans[2]) > 0:
            f.write('#define STM32_ADC_USE_ADC3 TRUE\n')
            f.write('#define HAL_ANALOG3_PINS \\\n')
            for (chan, analog, scale, label, portpin) in adc_chans[2]:
                scale_str = '%.2f/4096' % vdd
                if scale is not None and scale != '1':
                    scale_str = scale + '*' + scale_str
                f.write('{ %2u, %2u, %12s }, /* %s %s */ \\\n' %
                        (chan, analog, scale_str, portpin, label))
            f.write('\n\n')

    def write_GPIO_config(self, f):
        '''write GPIO config defines'''
        f.write('// GPIO config\n')
        gpios = []
        gpioset = set()
        for label in self.bylabel:
            p = self.bylabel[label]
            if 'SPI' in label and ('RX' in label or 'TX' in label):
                continue
            gpio = p.extra_value('GPIO', type=int)
            if gpio is None:
                continue
            if gpio in gpioset:
                self.error("Duplicate GPIO value %u" % gpio)
            gpioset.add(gpio)
            # see if it is also a PWM pin
            pwm = p.extra_value('PWM', type=int, default=0)
            port = p.port
            pin = p.pin
            # default config always enabled
            gpios.append((gpio, pwm, port, pin, p, 'true'))
        for alt in self.altmap.keys():
            for pp in self.altmap[alt].keys():
                p = self.altmap[alt][pp]
                gpio = p.extra_value('GPIO', type=int)
                if gpio is None:
                    continue
                if gpio in gpioset:
                    # check existing entry
                    existing_gpio = [item for item in gpios if item[0] == gpio]
                    if (existing_gpio[0][4].label == p.label) and (existing_gpio[0][3] == p.pin) and (existing_gpio[0][2] == p.port):  # noqa
                        # alt item is identical to exiting, do not add again
                        continue
                    self.error("Duplicate GPIO value %u, %s != %s" % (gpio, p, existing_gpio[0][4]))
                pwm = p.extra_value('PWM', type=int, default=0)
                if pwm != 0:
                    self.error("PWM not supported for alt config: %s" % p)
                gpioset.add(gpio)
                port = p.port
                pin = p.pin
                # aux config disabled by default
                gpios.append((gpio, pwm, port, pin, p, 'false'))
        gpios = sorted(gpios)
        for (gpio, pwm, port, pin, p, enabled) in gpios:
            f.write('#define HAL_GPIO_LINE_GPIO%u PAL_LINE(GPIO%s,%uU)\n' % (gpio, port, pin))
        if len(gpios) > 0:
            f.write('#define HAL_GPIO_PINS { \\\n')
            for (gpio, pwm, port, pin, p, enabled) in gpios:
                f.write('{ %3u, %s, %2u, PAL_LINE(GPIO%s,%uU)}, /* %s */ \\\n' %
                        (gpio, enabled, pwm, port, pin, p))
            # and write #defines for use by config code
            f.write('}\n\n')
        f.write('// full pin define list\n')
        last_label = None
        for label in sorted(list(set(self.bylabel.keys()))):
            p = self.bylabel[label]
            label = p.label
            label = label.replace('-', '_')
            if label == last_label:
                continue
            last_label = label
            f.write('#define HAL_GPIO_PIN_%-20s PAL_LINE(GPIO%s,%uU)\n' %
                    (label, p.port, p.pin))
        f.write('\n')

    def bootloader_path(self):
        # always embed a bootloader if it is available
        this_dir = os.path.realpath(__file__)
        rootdir = os.path.relpath(os.path.join(this_dir, "../../../../.."))
        hwdef_dirname = os.path.basename(os.path.dirname(self.hwdef[0]))
        # allow reusing of bootloader from different build:
        use_bootloader_from_board = self.get_config('USE_BOOTLOADER_FROM_BOARD', default=None, required=False)
        if use_bootloader_from_board is not None:
            hwdef_dirname = use_bootloader_from_board
        bootloader_filename = "%s_bl.bin" % (hwdef_dirname,)
        bootloader_path = os.path.join(rootdir,
                                       "Tools",
                                       "bootloaders",
                                       bootloader_filename)
        return bootloader_path

    def embed_bootloader(self, f):
        '''added bootloader to ROMFS'''
        if not self.intdefines.get('AP_BOOTLOADER_FLASHING_ENABLED', 1):
            # or, you know, not...
            return

        if self.is_bootloader_fw():
            return

        if self.is_io_fw():
            return

        bp = self.bootloader_path()
        if not os.path.exists(bp):
            self.error('''Bootloader (%s) does not exist and AP_BOOTLOADER_FLASHING_ENABLED
Please run: Tools/scripts/build_bootloaders.py %s
''' %
                       (bp, os.path.basename(os.path.dirname(self.hwdef[0]))))

        bp = os.path.realpath(bp)

        self.romfs["bootloader.bin"] = bp
        f.write("#define AP_BOOTLOADER_FLASHING_ENABLED 1\n")

    def write_ROMFS(self):
        '''create ROMFS embedded header'''
        romfs_list = []
        for k in self.romfs.keys():
            romfs_list.append((k, self.romfs[k]))
        self.env_vars['ROMFS_FILES'] = romfs_list

    def setup_apj_IDs(self):
        '''setup the APJ board IDs'''
        self.env_vars['APJ_BOARD_ID'] = self.get_numeric_board_id()
        self.env_vars['APJ_BOARD_TYPE'] = self.get_config('APJ_BOARD_TYPE', default=self.mcu_type)
        (USB_VID, USB_PID) = self.get_USB_IDs()
        self.env_vars['USBID'] = '0x%04x/0x%04x' % (USB_VID, USB_PID)

    def write_peripheral_enable(self, f):
        '''write peripheral enable lines'''
        f.write('// peripherals enabled\n')
        for type in sorted(list(self.bytype.keys()) + list(self.alttype.keys())):
            if type.startswith('USART') or type.startswith('UART'):
                dstr = 'STM32_SERIAL_USE_%-6s' % type
                f.write('#ifndef %s\n' % dstr)
                f.write('#define %s TRUE\n' % dstr)
                f.write('#endif\n')
            if type.startswith('SPI'):
                f.write('#define STM32_SPI_USE_%s                  TRUE\n' % type)
            if type.startswith('I2C'):
                f.write('#define STM32_I2C_USE_%s                  TRUE\n' % type)
            if type.startswith('QUADSPI'):
                f.write('#define STM32_WSPI_USE_%s                 TRUE\n' % type)
            if type.startswith('OCTOSPI'):
                f.write('#define STM32_WSPI_USE_%s                 TRUE\n' % type)

    def get_dma_exclude(self, periph_list):
        '''return list of DMA devices to exclude from DMA'''
        dma_exclude = set()
        for p in self.dma_exclude_pattern:
            for periph in periph_list:
                if fnmatch.fnmatch(periph, p):
                    dma_exclude.add(periph)

        for periph in periph_list:
            if periph in self.bylabel:
                p = self.bylabel[periph]
                if p.has_extra('NODMA'):
                    dma_exclude.add(periph)
            if periph in self.altlabel:
                p = self.altlabel[periph]
                if p.has_extra('NODMA'):
                    dma_exclude.add(periph)
        return list(dma_exclude)

    def write_alt_config(self, f):
        '''write out alternate config settings'''
        if len(self.altmap.keys()) == 0:
            # no alt configs
            return
        f.write('''
/* alternative configurations */
#define PAL_STM32_SPEED(n) ((n&3U)<<3U)
#define PAL_STM32_HIGH     0x8000U

#define HAL_PIN_ALT_CONFIG { \\
''')
        for alt in sorted(self.altmap.keys()):
            for pp in sorted(self.altmap[alt].keys()):
                p = self.altmap[alt][pp]
                f.write("    { %u, %s, PAL_LINE(GPIO%s,%uU), %s, %u}, /* %s */ \\\n" %
                        (alt, p.pal_modeline(), p.port, p.pin, p.periph_type(), p.periph_instance(), str(p)))
        f.write('}\n\n')

    def write_all_lines(self, hwdat):
        super(ChibiOSHWDef, self).write_all_lines(hwdat)

        if not self.is_periph_fw() and not os.getenv("NO_ROMFS_HWDEF", False):
            self.romfs["hwdef.dat"] = hwdat

    def write_hwdef_header_content(self, f):
        '''write hwdef header file'''
        f.write('''#define MHZ (1000U*1000U)
#define KHZ (1000U)

''')

        if self.signed_fw:
            f.write('''
#define AP_SIGNED_FIRMWARE 1
''')
        else:
            f.write('''
#define AP_SIGNED_FIRMWARE 0
''')

        enable_dfu_boot = self.get_config('ENABLE_DFU_BOOT', default=0)
        if enable_dfu_boot:
            self.env_vars['ENABLE_DFU_BOOT'] = 1
            f.write('''
#define HAL_ENABLE_DFU_BOOT TRUE
''')
        else:
            self.env_vars['ENABLE_DFU_BOOT'] = 0
            f.write('''
#define HAL_ENABLE_DFU_BOOT FALSE
''')

        self.dma_noshare.extend(self.get_config('DMA_NOSHARE', default='', aslist=True))

        self.write_mcu_config(f)
        self.write_SPI_config(f)
        self.write_WSPI_config(f)
        self.write_ADC_config(f)
        self.write_GPIO_config(f)
        self.write_IMU_config(f)
        self.write_MAG_config(f)
        self.write_BARO_config(f)
        self.write_AIRSPEED_config(f)
        self.write_DATAFLASH_config(f)
        self.write_board_validate_macro(f)
        self.write_check_firmware(f)

        if self.have_defaults_file:
            f.write('''
#ifndef AP_FILESYSTEM_ROMFS_ENABLED
#define AP_FILESYSTEM_ROMFS_ENABLED 1
#endif
''')

        self.write_peripheral_enable(f)

        if self.processed_defaults_filepath:
            self.write_define(f, 'AP_PARAM_DEFAULTS_FILE_PARSING_ENABLED', 1)
        else:
            self.write_define(f, 'AP_PARAM_DEFAULTS_FILE_PARSING_ENABLED', 0)

        if self.mcu_series.startswith("STM32H7"):
            # add in ADC3 on H7 to get MCU temperature and reference voltage
            self.periph_list.append('ADC3')

        if self.get_config('DMA_NOMAP', required=False) is not None:
            dma_unassigned, ordered_timers = [], []
        else:
            dma_unassigned, ordered_timers = dma_resolver.write_dma_header(
                f,
                self.periph_list,
                self.mcu_type,
                dma_exclude=self.get_dma_exclude(self.periph_list),
                dma_priority=self.get_config('DMA_PRIORITY', default='TIM* SPI*', spaces=True),
                dma_noshare=self.dma_noshare,
                quiet=self.quiet,
            )

        if not self.is_bootloader_fw():
            self.write_PWM_config(f, ordered_timers)
            self.write_I2C_config(f)
            self.write_UART_config(f)
        else:
            self.write_UART_config_bootloader(f)

        self.setup_apj_IDs()
        self.write_USB_config(f)

        self.embed_bootloader(f)

        if self.mcu_series.startswith('STM32F1'):
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

        for port in sorted(self.ports):
            f.write("/* PORT%s:\n" % port)
            for pin in range(self.pincount[port]):
                p = self.portmap[port][pin]
                if p.label is not None:
                    f.write(" %s\n" % p)
            f.write("*/\n\n")

            if self.pincount[port] == 0:
                # handle blank ports
                for vtype in self.vtypes:
                    f.write("#define VAL_GPIO%s_%-7s             0x0\n" % (port,
                                                                           vtype))
                f.write("\n\n\n")
                continue

            for vtype in self.vtypes:
                f.write("#define VAL_GPIO%s_%-7s (" % (p.port, vtype))
                first = True
                for pin in range(self.pincount[port]):
                    p = self.portmap[port][pin]
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
        self.write_alt_config(f)

        if not self.mcu_series.startswith("STM32F1"):
            dma_required = ['SPI*', 'ADC*']
            if 'IOMCU_UART' in self.config:
                dma_required.append(self.config['IOMCU_UART'][0] + '*')
            for d in dma_unassigned:
                for r in dma_required:
                    if fnmatch.fnmatch(d, r):
                        self.error("Missing required DMA for %s" % d)

        self.add_apperiph_defaults(f)
        self.add_bootloader_defaults(f)
        self.add_iomcu_firmware_defaults(f)
        self.add_normal_firmware_defaults(f)

    def build_peripheral_list(self):
        '''build a list of peripherals for DMA resolver to work on'''
        peripherals = []
        self.shared_up = []
        done = set()
        prefixes = ['SPI', 'USART', 'UART', 'I2C']
        periph_pins = self.allpins[:]
        for alt in self.altmap.keys():
            for p in self.altmap[alt].keys():
                periph_pins.append(self.altmap[alt][p])
        for p in periph_pins:
            type = p.type
            if type.startswith('TIM'):
                # we need to independently demand DMA for each channel
                type = p.label
            if type in done:
                continue
            for prefix in prefixes:
                if type.startswith(prefix):
                    ptx = type + "_TX"
                    prx = type + "_RX"
                    if prefix in ['SPI', 'I2C']:
                        # in DMA map I2C and SPI has RX and TX suffix
                        if ptx not in self.bylabel:
                            self.bylabel[ptx] = p
                        if prx not in self.bylabel:
                            self.bylabel[prx] = p
                    if prx in self.bylabel or prx in self.altlabel:
                        peripherals.append(prx)
                    if ptx in self.bylabel or ptx in self.altlabel:
                        peripherals.append(ptx)

            if type.startswith('ADC'):
                peripherals.append(type)
            if type.startswith('SDIO') or type.startswith('SDMMC'):
                if not self.mcu_series.startswith("STM32H7"):
                    peripherals.append(type)
            if type.startswith('TIM'):
                if p.has_extra('RCIN'):
                    label = p.label
                    if label[-1] == 'N':
                        label = label[:-1]
                    peripherals.append(label)
                    # RCIN DMA channel cannot be shared as it is running all the time
                    self.dma_noshare.append(label)
                elif not p.has_extra('ALARM') and not p.has_extra('RCININT'):
                    # get the TIMn_UP DMA channels for DShot
                    label = p.type + '_UP'
                    if label not in peripherals and not p.has_extra('NODMA'):
                        peripherals.append(label)
                    ch_label = type
                    (_, _, compl) = self.parse_timer(ch_label)
                    if ch_label not in peripherals and p.has_extra('BIDIR') and not compl:
                        peripherals.append(ch_label)
                    if label not in self.shared_up and p.has_extra('UP_SHARED') and not compl:
                        self.shared_up.append(label)
            done.add(type)
        return peripherals

    def get_processed_defaults_file(self, defaults_filepath, depth=0):
        '''reads defaults_filepath, expanding any @include lines to include
        the contents of the so-references file - recursively.'''
        if depth > 10:
            raise Exception("include loop")
        ret = ""
        with open(defaults_filepath, 'r') as defaults_fh:
            while True:
                line = defaults_fh.readline()
                if line == "":
                    break
                m = re.match(r"^@include\s*([^\s]+)", line)
                if m is None:
                    ret += line
                    continue
                # we've found an include; do that...
                include_filepath = os.path.join(os.path.dirname(defaults_filepath), m.group(1))
                try:
                    # ret += "# Begin included file (%s)" % include_filepath
                    ret += self.get_processed_defaults_file(include_filepath, depth=depth+1)
#                    ret += "# End included file (%s)" % include_filepath
                except FileNotFoundError:
                    raise Exception("%s includes %s but that filepath was not found" %
                                    (defaults_filepath, include_filepath))
        return ret

    def write_processed_defaults_file(self):
        # see if board has a defaults.parm file or a --default-parameters file was specified
        defaults_filename = os.path.join(os.path.dirname(self.hwdef[0]), 'defaults.parm')
        defaults_path = os.path.join(os.path.dirname(self.hwdef[0]), self.default_params_filepath)

        defaults_abspath = None
        if os.path.exists(defaults_path):
            defaults_abspath = os.path.abspath(self.default_params_filepath)
            self.progress("Default parameters path from command line: %s" % self.default_params_filepath)
        elif os.path.exists(defaults_filename):
            defaults_abspath = os.path.abspath(defaults_filename)
            self.progress("Default parameters path from hwdef: %s" % defaults_filename)

        if defaults_abspath is None:
            self.progress("No default parameter file found")
            return None

        content = self.get_processed_defaults_file(defaults_abspath)

        filepath = self.get_output_path("processed_defaults.parm")
        with open(filepath, "w") as processed_defaults_fh:
            processed_defaults_fh.write(content)

        return filepath

    def romfs_add(self, romfs_filename, filename):
        '''add a file to ROMFS'''
        self.romfs[romfs_filename] = filename

    def romfs_wildcard(self, pattern):
        '''add a set of files to ROMFS by wildcard'''
        base_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..')
        (pattern_dir, pattern) = os.path.split(pattern)
        for f in os.listdir(os.path.join(base_path, pattern_dir)):
            if fnmatch.fnmatch(f, pattern):
                self.romfs[f] = os.path.join(pattern_dir, f)

    def romfs_add_dir(self, subdirs, relative_to_base=False):
        '''add a filesystem directory to ROMFS'''
        if self.is_bootloader_fw():
            # FIXME: why were we called?!
            return
        for dirname in subdirs:
            if relative_to_base:
                romfs_dir = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', dirname)
            else:
                romfs_dir = os.path.join(os.path.dirname(self.hwdef[0]), dirname)
            if not os.path.exists(romfs_dir):
                continue

            if True:
                for root, dirs, files in os.walk(romfs_dir):
                    for f in files:
                        if fnmatch.fnmatch(f, '*~'):
                            # skip editor backup files
                            continue
                        fullpath = os.path.join(root, f)
                        relpath = os.path.normpath(os.path.join(dirname, os.path.relpath(root, romfs_dir), f))
                        if relative_to_base:
                            relpath = relpath[len(dirname)+1:]
                        self.romfs[relpath] = fullpath

    def valid_type(self, ptype, label):
        '''check type of a pin line is valid'''
        patterns = [
            r'INPUT', r'OUTPUT', r'TIM\d+', r'USART\d+', r'UART\d+', r'ADC\d+',
            r'SPI\d+', r'OTG\d+', r'SWD', r'CAN\d?', r'I2C\d+', r'CS',
            r'SDMMC\d+', r'SDIO', r'QUADSPI\d', r'OCTOSPI\d', r'ETH\d', r'RCC',
        ]
        matches = False
        for p in patterns:
            if re.match(p, ptype):
                matches = True
                break
        if not matches:
            return False
        # special checks for common errors
        m1 = re.match(r'TIM(\d+)', ptype)
        m2 = re.match(r'TIM(\d+)_CH\d+', label)
        if (m1 and not m2) or (m2 and not m1) or (m1 and m1.group(1) != m2.group(1)):
            '''timer numbers need to match'''
            return False
        m1 = re.match(r'CAN(\d+)', ptype)
        m2 = re.match(r'CAN(\d+)_(RX|TX)', label)
        if (m1 and not m2) or (m2 and not m1) or (m1 and m1.group(1) != m2.group(1)):
            '''CAN numbers need to match'''
            return False
        if ptype == 'OUTPUT' and re.match(r'US?ART\d+_(TXINV|RXINV)', label):
            return True
        m1 = re.match(r'USART(\d+)', ptype)
        m2 = re.match(r'USART(\d+)_(RX|TX|CTS|RTS|CTS_GPIO)', label)
        if (m1 and not m2) or (m2 and not m1) or (m1 and m1.group(1) != m2.group(1)):
            '''usart numbers need to match'''
            return False
        m1 = re.match(r'UART(\d+)', ptype)
        m2 = re.match(r'UART(\d+)_(RX|TX|CTS|RTS|CTS_GPIO)', label)
        if (m1 and not m2) or (m2 and not m1) or (m1 and m1.group(1) != m2.group(1)):
            '''uart numbers need to match'''
            return False
        return True

    def process_line(self, line, depth):
        '''process one line of pin definition file'''
        self.all_lines.append(line)
        a = shlex.split(line, posix=False)
        # keep all config lines for later use
        self.alllines.append(line)

        p = None
        if a[0].startswith('P') and a[0][1] in self.ports:
            # it is a port/pin definition
            try:
                port = a[0][1]
                pin = int(a[0][2:])
                label = a[1]
                type = a[2]
                extra = a[3:]
            except Exception:
                self.error("Bad pin line: %s" % a)
                return

            if not self.valid_type(type, label):
                self.error("bad type on line: %s" % a)

            p = self.generic_pin(port, pin, label, type, extra, self.mcu_type, self.mcu_series, self.get_ADC1_chan, self.get_ADC2_chan, self.get_ADC3_chan, self.af_labels)  # noqa
            af = self.get_alt_function(self.mcu_type, a[0], label)
            if af is not None:
                p.af = af

            alt = p.extra_value("ALT", type=int, default=0)
            if alt != 0:
                if self.mcu_series.startswith("STM32F1"):
                    self.error("Alt config not allowed for F1 MCU")
                if alt not in self.altmap:
                    self.altmap[alt] = {}
                if p.portpin in self.altmap[alt]:
                    self.error("Pin %s ALT(%u) redefined" % (p.portpin, alt))
                self.altmap[alt][p.portpin] = p
                # we need to add alt pins into self.bytype[] so they are enabled in chibios config
                if type not in self.alttype:
                    self.alttype[type] = []
                self.alttype[type].append(p)
                self.altlabel[label] = p
                return

            if a[0] in self.config:
                self.error("Pin %s redefined" % a[0])

        if p is None and line.find('ALT(') != -1:
            self.error("ALT() invalid for %s" % a[0])

        if a[0] == 'DEFAULTGPIO':
            self.default_gpio = a[1:]
            return

        if a[0] == 'NODMA':
            self.dma_exclude_pattern.extend(a[1:])
            return

        self.config[a[0]] = a[1:]
        if p is not None:
            # add to set of pins for primary config
            self.portmap[port][pin] = p
            self.allpins.append(p)
            if type not in self.bytype:
                self.bytype[type] = []
            self.bytype[type].append(p)
            self.bylabel[label] = p
        elif a[0] == 'MCU':
            self.mcu_type = a[2]
            self.mcu_series = a[1]
            self.setup_mcu_type_defaults()
        elif a[0] == 'SPIDEV':
            self.spidev.append(a[1:])
        elif a[0] == 'QSPIDEV':
            self.wspidev.append(a[1:])
        elif a[0] == 'OSPIDEV':
            self.wspidev.append(a[1:])
        elif a[0] == 'DATAFLASH':
            self.dataflash_list.append(a[1:])
        elif a[0] == 'AIRSPEED':
            self.airspeed_list.append(a[1:])
        elif a[0] == 'ROMFS':
            self.romfs_add(a[1], a[2])
        elif a[0] == 'ROMFS_WILDCARD':
            self.romfs_wildcard(a[1])
        elif a[0] == 'ROMFS_DIRECTORY':
            self.romfs_add_dir([a[1]], relative_to_base=True)
        else:
            super(ChibiOSHWDef, self).process_line(line, depth)

    def process_line_undef(self, line, depth, a):
        for u in a[1:]:
            self.progress("Removing %s" % u)
            self.bytype.pop(u, '')
            self.bylabel.pop(u, '')
            # remove alt config definitions
            for alt in sorted(self.altmap.keys()):
                for pp in sorted(self.altmap[alt].keys()):
                    p = self.altmap[alt][pp]
                    if p.portpin == u:
                        del self.altmap[alt][pp]
                        if p.label in self.altlabel.keys():
                            del self.altlabel[p.label]
            self.alttype.pop(u, '')
            for dev in self.spidev:
                if u == dev[0]:
                    self.spidev.remove(dev)
            # also remove all occurrences of defines in previous lines if any
            for line in self.alllines[:]:
                if line.startswith('STM32_') and u == line.split()[0]:
                    self.alllines.remove(line)
            newpins = []
            for pin in self.allpins:
                if pin.type == u or pin.label == u or pin.portpin == u:
                    if pin.label is not None:
                        self.bylabel.pop(pin.label, '')
                    self.portmap[pin.port][pin.pin] = self.generic_pin(pin.port, pin.pin, None, 'INPUT', [], self.mcu_type, self.mcu_series, self.get_ADC1_chan, self.get_ADC2_chan, self.get_ADC3_chan, self.af_labels)  # noqa
                    continue
                newpins.append(pin)
            self.allpins = newpins
            if u == 'DATAFLASH':
                self.dataflash_list = []
            if u == 'AIRSPEED':
                self.airspeed_list = []
            if u == 'ROMFS':
                self.romfs = {}

        super(ChibiOSHWDef, self).process_line_undef(line, depth, a)

    def process_line_env(self, line, depth, a):
        name = a[1]
        value = ' '.join(a[2:])
        if name == 'AP_PERIPH' and value != "1":
            raise ValueError("AP_PERIPH may only have value 1")

        super(ChibiOSHWDef, self).process_line_env(line, depth, a)

    def add_apperiph_defaults(self, f):
        '''add default defines for peripherals'''
        if not self.is_periph_fw():
            # not AP_Periph
            return

        self.add_firmware_defaults_from_file(f, "defaults_periph.h", "AP_Periph")

    def is_bootloader_fw(self):
        return self.bootloader

    def add_bootloader_defaults(self, f):
        '''add default defines for peripherals'''
        if not self.is_bootloader_fw():
            return

        self.add_firmware_defaults_from_file(f, "defaults_bootloader.h", "bootloader")

    def add_firmware_defaults_from_file(self, f, filename, description):
        self.progress("Setting up as %s" % description)

        dirpath = os.path.dirname(os.path.realpath(__file__))
        filepath = os.path.join(dirpath, filename)

        content = open(filepath, 'r').read()
        f.write('''
// %s defaults

%s

// end %s defaults
''' % (description, content, description))

    def is_io_fw(self):
        return int(self.env_vars.get('IOMCU_FW', 0)) != 0

    def add_iomcu_firmware_defaults(self, f):
        '''add default defines IO firmwares'''
        if not self.is_io_fw():
            # not IOMCU firmware
            return

        self.add_firmware_defaults_from_file(f, "defaults_iofirmware.h", "IOMCU Firmware")

    def is_periph_fw_unprocessed_file(self, hwdef, includer=None):
        '''helper/recursion function for is_periph_fw_unprocessed'''
        if not os.path.exists(hwdef):
            raise ChibiOSHWDefIncludeNotFoundException(
                os.path.normpath(hwdef),
                os.path.normpath(includer)
            )
        with open(hwdef, "r") as f:
            content = f.read()
            if 'AP_PERIPH' in content:
                return True
            # process any include lines:
            for m in re.finditer(r"^include\s+([^\s]*)", content, re.MULTILINE):
                include_path = os.path.join(os.path.dirname(hwdef), m.group(1))
                if self.is_periph_fw_unprocessed_file(include_path, includer=hwdef):
                    return True

    def is_periph_fw_unprocessed(self):
        '''it takes ~2 seconds to process all hwdefs.  This is a shortcut to
        make things much faster in the case we are filtering boards to
        just peripherals.  Note that this parsing is very coarse -
        AP_PERIPH could be in a comment or part of a define
        (e.g. AP_PERIPH_GPS_SUPPORT), for example, and this method
        will still return True.  Also can't "undef" AP_PERIPH - if we
        ever see the string we return true.
        '''
        for xhwdef in self.hwdef:
            if self.is_periph_fw_unprocessed_file(xhwdef):
                return True
        return False

    def is_periph_fw(self):
        if not self.processed_hwdefs:
            raise ValueError("Need to process_hwdefs() first")
        return int(self.env_vars.get('AP_PERIPH', 0)) != 0

    def is_normal_fw(self):
        if self.is_io_fw():
            # IOMCU firmware
            return False
        if self.is_periph_fw():
            # Periph firmware
            return False
        if self.is_bootloader_fw():
            # guess
            return False
        return True

    def add_normal_firmware_defaults(self, f):
        '''add default defines to builds with are not bootloader, periph or IOMCU'''
        if not self.is_normal_fw():
            return

        self.add_firmware_defaults_from_file(f, "defaults_normal.h", "normal")

    def write_default_parameters(self):
        '''handle default parameters'''

        if self.is_bootloader_fw():
            return

        if self.is_io_fw():
            return

        self.processed_defaults_filepath = self.write_processed_defaults_file()
        if not self.processed_defaults_filepath:
            return

        if self.get_config('FORCE_APJ_DEFAULT_PARAMETERS', default=False):
            # set env variable so that post-processing in waf uses
            # apj-tool to append parameters to image:
            if os.path.exists(self.processed_defaults_filepath):
                self.env_vars['DEFAULT_PARAMETERS'] = self.processed_defaults_filepath
            return

        self.romfs_add('defaults.parm', self.processed_defaults_filepath)
        self.have_defaults_file = True

    def get_stale_defines(self):
        '''returns a map with a stale define and a comment as to what to do about it'''
        ret = super().get_stale_defines()
        ret.update({
            'HAL_NO_RCIN_THREAD': 'HAL_NO_RCIN_THREAD is no longer used; try "define HAL_RCIN_THREAD_ENABLED 0"',
            'HAL_NO_MONITOR_THREAD': 'HAL_NO_MONITOR_THREAD is no longer used; try "define HAL_MONITOR_THREAD_ENABLED 0"',
            'HAL_NO_GPIO_IRQ': 'HAL_NO_GPIO_IRQ is no longer used; remove it from your hwdef',
            'DISABLE_SERIAL_ESC_COMM': 'DISABLE_SERIAL_ESC_COMM is no longer used; try "define HAL_SERIAL_ESC_COMM_ENABLED 1"',
        })
        return ret

    def run(self):
        # process input file
        self.process_hwdefs()

        if "MCU" not in self.config:
            self.error("Missing MCU type in config")

        self.mcu_type = self.get_config('MCU', 1)
        self.progress("Setup for MCU %s" % self.mcu_type)

        # put USE_BOOTLOADER_FROM_BOARD into the environment so the
        # build process can use it when generating hex files:
        use_bootloader_from_board = self.get_config('USE_BOOTLOADER_FROM_BOARD', default=None, required=False)
        if use_bootloader_from_board is not None:
            self.env_vars['USE_BOOTLOADER_FROM_BOARD'] = use_bootloader_from_board

        # build a list for peripherals for DMA resolver
        self.periph_list = self.build_peripheral_list()

        # write out a default parameters file, decide how to use it:
        self.write_default_parameters()

        # write out hw.dat for ROMFS
        self.write_all_lines(self.get_output_path("hw.dat"))

        # Add ROMFS directories
        self.romfs_add_dir(['scripts'])
        self.romfs_add_dir(['param'])

        # write out hwdef.h
        self.write_hwdef_header(self.get_output_path("hwdef.h"))

        # write out ldscript.ld
        self.write_ldscript(self.get_output_path("ldscript.ld"))

        self.write_ROMFS()

        # copy the shared linker script into the build directory; it must
        # exist in the same directory as the ldscript.ld file we generate.
        self.copy_common_linkerscript(self.get_output_path("common.ld"))

        # CHIBIOS_BUILD_FLAGS is passed to the ChibiOS makefile
        self.env_vars['CHIBIOS_BUILD_FLAGS'] = ' '.join(self.build_flags)


if __name__ == '__main__':

    parser = argparse.ArgumentParser("chibios_pins.py")
    parser.add_argument(
        '-D', '--outdir', type=str, default="/tmp", help='Output directory')
    parser.add_argument(
        '--bootloader', action='store_true', default=False, help='configure for bootloader')
    parser.add_argument(
        '--signed-fw', action='store_true', default=False, help='configure for signed FW')
    parser.add_argument(
        'hwdef', type=str, nargs='+', default=None, help='hardware definition file')
    parser.add_argument(
        '--params', type=str, default=None, help='user default params path')
    parser.add_argument(
        '--quiet', action='store_true', default=False, help='quiet running')

    args = parser.parse_args()

    c = ChibiOSHWDef(
        outdir=args.outdir,
        bootloader=args.bootloader,
        signed_fw=args.signed_fw,
        hwdef=args.hwdef,
        default_params_filepath=args.params,
        quiet=args.quiet,
    )
    c.run()
