#!/usr/bin/env python
'''
setup board.h for chibios
'''

import argparse, sys, fnmatch, os, dma_resolver, shlex

parser = argparse.ArgumentParser("chibios_pins.py")
parser.add_argument('-D', '--outdir', type=str, default=None, help='Output directory')
parser.add_argument('hwdef', type=str, default=None, help='hardware definition file')

args = parser.parse_args()

# output variables for each pin
vtypes = ['MODER', 'OTYPER', 'OSPEEDR', 'PUPDR', 'ODR', 'AFRL', 'AFRH']
        
# number of pins in each port
pincount = { 'A': 16, 'B' : 16, 'C' : 16, 'D' : 16, 'E' : 16, 'F' : 16, 'G' : 16, 'H' : 2, 'I' : 0, 'J' : 0, 'K' : 0 }

ports = pincount.keys()

portmap = {}

# dictionary of all config lines, indexed by first word
config = {}

# list of all pins in config file order
allpins = []

# list of configs by type
bytype = {}

mcu_type = None

def get_alt_function(mcu, pin, function):
    '''return alternative function number for a pin'''
    import importlib
    try:
            lib = importlib.import_module(mcu)
            alt_map = lib.AltFunction_map
    except ImportError:
            print("Unable to find module for MCU %s" % mcu)
            sys.exit(1)
    
    af_labels = ['USART', 'UART', 'SPI', 'I2C', 'SDIO', 'OTG', 'JT', 'TIM']
    for l in af_labels:
        if function.startswith(l):
            s = pin+":"+function
            if not s in alt_map:
                print("Unknown pin function %s for MCU %s" % (s, mcu))
                sys.exit(1)
            return alt_map[s]
    return None

def get_ADC1_chan(mcu, pin):
    '''return ADC1 channel for an analog pin'''
    import importlib
    try:
            lib = importlib.import_module(mcu)
            ADC1_map = lib.ADC1_map
    except ImportError:
            print("Unable to find ADC1_Map for MCU %s" % mcu)
            sys.exit(1)
    
    if not pin in ADC1_map:
        print("Unable to find ADC1 channel for pin %s" % pin)
        sys.exit(1)
    return ADC1_map[pin]
        
class generic_pin(object):
        '''class to hold pin definition'''
        def __init__(self, port, pin, label, type, extra):
                self.portpin = "P%s%u" % (port, pin)
                self.port = port
                self.pin = pin
                self.label = label
                self.type = type
                self.extra = extra
                self.af = None

        def has_extra(self, v):
                return v in self.extra

        def get_MODER(self):
                '''return one of ALTERNATE, OUTPUT, ANALOG, INPUT'''
                if self.af is not None:
                        v = "ALTERNATE"
                elif self.type == 'OUTPUT':
                        v = "OUTPUT"
                elif self.type.startswith('ADC'):
                        v = "ANALOG"
                elif self.has_extra("CS"):
                        v = "OUTPUT"
                else:
                        v = "INPUT"
                return "PIN_MODE_%s(%uU)" % (v, self.pin)

        def get_OTYPER(self):
                '''return one of PUSHPULL, OPENDRAIN'''
                v = 'PUSHPULL'
                if self.type.startswith('I2C'):
                        # default I2C to OPENDRAIN
                        v = 'OPENDRAIN'
                values = ['PUSHPULL', 'OPENDRAIN']
                for e in self.extra:
                        if e in values:
                                v = e
                return "PIN_OTYPE_%s(%uU)" % (v, self.pin)

        def get_OSPEEDR(self):
                '''return one of SPEED_VERYLOW, SPEED_LOW, SPEED_MEDIUM, SPEED_HIGH'''
                values = ['SPEED_VERYLOW', 'SPEED_LOW', 'SPEED_MEDIUM', 'SPEED_HIGH']
                v = 'SPEED_HIGH'
                if self.has_extra("CS"):
                        v = "SPEED_MEDIUM"
                if self.type.startswith("I2C"):
                        v = "SPEED_MEDIUM"
                for e in self.extra:
                        if e in values:
                                v = e
                return "PIN_O%s(%uU)" % (v, self.pin)

        def get_PUPDR(self):
                '''return one of FLOATING, PULLUP, PULLDOWN'''
                values = ['FLOATING', 'PULLUP', 'PULLDOWN']
                v = 'FLOATING'
                if self.has_extra("CS"):
                        v = "PULLUP"
                for e in self.extra:
                        if e in values:
                                v = e
                return "PIN_PUPDR_%s(%uU)" % (v, self.pin)

        def get_ODR(self):
                '''return one of LOW, HIGH'''
                values = ['LOW', 'HIGH']
                v = 'HIGH'
                for e in self.extra:
                        if e in values:
                                v = e
                return "PIN_ODR_%s(%uU)" % (v, self.pin)

        def get_AFIO(self):
                '''return AFIO'''
                af = self.af
                if af is None:
                        af = 0
                return "PIN_AFIO_AF(%uU, %uU)" % (self.pin, af)

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

        def __str__(self):
                afstr = ''
                adcstr = ''
                if self.af is not None:
                    afstr = " AF%u" % self.af
                if self.type.startswith('ADC1'):
                    adcstr = " ADC1_IN%u" % get_ADC1_chan(mcu_type, self.portpin)
                return "P%s%u %s %s%s%s" % (self.port, self.pin, self.label, self.type, afstr, adcstr)

# setup default as input pins
for port in ports:
        portmap[port] = []
        for pin in range(pincount[port]):
                portmap[port].append(generic_pin(port, pin, None, 'INPUT', []))

def get_config(name, column=0, required=True, default=None):
    '''get a value from config dictionary'''
    if not name in config:
        if required and default is None:
            print("Error: missing required value %s in hwdef.dat" % name)
            sys.exit(1)
        return default
    if len(config[name]) < column+1:
        print("Error: missing required value %s in hwdef.dat (column %u)" % (name, column))
        sys.exit(1)
    return config[name][column]

def process_line(line):
        '''process one line of pin definition file'''
        a = shlex.split(line)
        # keep all config lines for later use
        config[a[0]] = a[1:]
        if a[0] == 'MCU':
                global mcu_type
                mcu_type = a[2]
        if a[0].startswith('P') and a[0][1] in ports:
                # it is a port/pin definition
                try:
                        port = a[0][1]
                        pin = int(a[0][2:])
                        label = a[1]
                        type = a[2]
                        extra = a[3:]
                except Exception:
                        print("Bad pin line: %s" % a)
                        return

                p = generic_pin(port, pin, label, type, extra)
                portmap[port][pin] = p
                allpins.append(p)
                if not type in bytype:
                        bytype[type] = []
                bytype[type].append(p)
                af = get_alt_function(mcu_type, a[0], label)
                if af is not None:
                        p.af = af
                

def write_mcu_config(f):
        '''write MCU config defines'''
        f.write('// MCU type (ChibiOS define)\n')
        f.write('#define %s_MCUCONF\n' % get_config('MCU'))
        f.write('#define %s\n\n' % get_config('MCU', 1))
        f.write('// Board voltage. Required for performance limits calculation\n')
        f.write('#define STM32_VDD %s\n\n' % get_config('STM32_VDD'))
        f.write('// crystal frequency\n')
        f.write('#define STM32_HSECLK %sU\n\n' % get_config('OSCILLATOR_HZ'))
        f.write('// UART used for stdout (printf)\n')
        f.write('#define HAL_STDOUT_SERIAL %s\n\n' % get_config('STDOUT_SERIAL'))
        f.write('// baudrate used for stdout (printf)\n')
        f.write('#define HAL_STDOUT_BAUDRATE %s\n\n' % get_config('STDOUT_BAUDRATE'))
        if 'SDIO' in bytype:
                f.write('// SDIO available, enable POSIX filesystem support\n')
                f.write('#define USE_POSIX\n\n')


def write_USB_config(f):
        '''write USB config defines'''
        if not 'USB_VENDOR' in config:
            return
        f.write('// USB configuration\n')
        f.write('#define HAL_USB_VENDOR_ID %s\n' % get_config('USB_VENDOR'))
        f.write('#define HAL_USB_PRODUCT_ID %s\n' % get_config('USB_PRODUCT'))
        for s in ['USB_STRING_MANUFACTURER', 'USB_STRING_PRODUCT', 'USB_STRING_SERIAL']:
            f.write('#define HAL_%s "%s"\n' % (s, get_config(s)))

        f.write('\n\n')

def write_I2C_config(f):
        '''write I2C config defines'''
        i2c_list = get_config('I2C_ORDER').split()
        devlist = []
        for dev in i2c_list:
            if not dev.startswith('I2C') or dev[3] not in "1234":
                print("Bad I2C_ORDER element %s" % dev)
                sys.exit(1)
            n = int(dev[3:])
            devlist.append('&I2CD%u' % n)
        f.write('\n// List of I2C devices\n')
        f.write('#define HAL_I2C_DEVICE_LIST %s\n\n' % ','.join(devlist))

def write_prototype_file():
    '''write the prototype file for apj generation'''
    pf = open(os.path.join(outdir, "apj.prototype"), "w")
    pf.write('''{
    "board_id": %s, 
    "magic": "PX4FWv1", 
    "description": "Firmware for the %s board", 
    "image": "", 
    "build_time": 0, 
    "summary": "PX4FMUv3",
    "version": "0.1",
    "image_size": 0,
    "git_identity": "",
    "board_revision": 0
}
''' % (get_config('APJ_BOARD_ID'),
       get_config('APJ_BOARD_TYPE', default=mcu_type)))

def write_peripheral_enable(f):
        '''write peripheral enable lines'''
        f.write('// peripherals enabled\n')
        for type in sorted(bytype.keys()):
                if type.startswith('USART') or type.startswith('UART'):
                        f.write('#define STM32_SERIAL_USE_%-6s             TRUE\n' % type)
                if type.startswith('SPI'):
                        f.write('#define STM32_SPI_USE_%s                  TRUE\n' % type)
                if type.startswith('OTG'):
                        f.write('#define STM32_USB_USE_%s                  TRUE\n' % type)
                if type.startswith('I2C'):
                        f.write('#define STM32_I2C_USE_%s                  TRUE\n' % type)
        
def write_hwdef_header(outfilename):
        '''write hwdef header file'''
        print("Writing hwdef setup in %s" % outfilename)
        f = open(outfilename, 'w')

        f.write('''/*
 generated hardware definitions from hwdef.dat - DO NOT EDIT
*/

#pragma once

''');

        write_mcu_config(f)
        write_USB_config(f)
        write_I2C_config(f)

        write_peripheral_enable(f)
        write_prototype_file()

        dma_resolver.write_dma_header(f, periph_list, mcu_type)

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
                                f.write("#define VAL_GPIO%s_%-7s             0x0\n" % (port, vtype))
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

def build_peripheral_list():
        '''build a list of peripherals for DMA resolver to work on'''
        peripherals = []
        done = set()
        prefixes = ['SPI', 'USART', 'UART', 'I2C']
        for p in allpins:
                type = p.type
                if type in done:
                        continue
                for prefix in prefixes:
                        if type.startswith(prefix):
                                peripherals.append(type + "_TX")
                                peripherals.append(type + "_RX")
                if type.startswith('ADC'):
                        peripherals.append(type)                        
                if type.startswith('SDIO'):
                        peripherals.append(type)                        
                done.add(type)
        return peripherals

# process input file
hwdef_file = args.hwdef

f = open(hwdef_file,"r")
for line in f.readlines():
        line = line.strip()
        if len(line) == 0 or line[0] == '#':
                continue
        process_line(line)

outdir = args.outdir
if outdir is None:
        outdir = os.path.dirname(hwdef_file)

if not "MCU" in config:
        print("Missing MCU type in config")
        sys.exit(1)

mcu_type = get_config('MCU',1)
print("Setup for MCU %s" % mcu_type)

# build a list for peripherals for DMA resolver
periph_list = build_peripheral_list()

# write out hwdef.h
write_hwdef_header(os.path.join(outdir, "hwdef.h"))

