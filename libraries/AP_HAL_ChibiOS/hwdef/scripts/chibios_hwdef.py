#!/usr/bin/env python
'''
setup board.h for chibios
'''

import argparse, sys, fnmatch, os, dma_resolver, shlex

parser = argparse.ArgumentParser("chibios_pins.py")
parser.add_argument(
    '-D', '--outdir', type=str, default=None, help='Output directory')
parser.add_argument(
    'hwdef', type=str, default=None, help='hardware definition file')

args = parser.parse_args()

# output variables for each pin
vtypes = ['MODER', 'OTYPER', 'OSPEEDR', 'PUPDR', 'ODR', 'AFRL', 'AFRH']

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

# list of all pins in config file order
allpins = []

# list of configs by type
bytype = {}

# list of configs by label
bylabel = {}

# list of SPI devices
spidev = []

# SPI bus list
spi_list = []

# all config lines in order
alllines = []

mcu_type = None


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


def get_alt_function(mcu, pin, function):
    '''return alternative function number for a pin'''
    import importlib
    try:
        lib = importlib.import_module(mcu)
        alt_map = lib.AltFunction_map
    except ImportError:
        error("Unable to find module for MCU %s" % mcu)

    if function and function.endswith("_RTS") and (
            function.startswith('USART') or function.startswith('UART')):
        # we do software RTS
        return None

    af_labels = ['USART', 'UART', 'SPI', 'I2C', 'SDIO', 'OTG', 'JT', 'TIM', 'CAN']
    for l in af_labels:
        if function.startswith(l):
            s = pin + ":" + function
            if not s in alt_map:
                error("Unknown pin function %s for MCU %s" % (s, mcu))
            return alt_map[s]
    return None


def get_ADC1_chan(mcu, pin):
    '''return ADC1 channel for an analog pin'''
    import importlib
    try:
        lib = importlib.import_module(mcu)
        ADC1_map = lib.ADC1_map
    except ImportError:
        error("Unable to find ADC1_Map for MCU %s" % mcu)

    if not pin in ADC1_map:
        error("Unable to find ADC1 channel for pin %s" % pin)
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

    def get_MODER(self):
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
        # on STM32F4 these speeds correspond to 2MHz, 25MHz, 50MHz and 100MHz
        values = ['SPEED_VERYLOW', 'SPEED_LOW', 'SPEED_MEDIUM', 'SPEED_HIGH']
        v = 'SPEED_MEDIUM'
        for e in self.extra:
            if e in values:
                v = e
        return "PIN_O%s(%uU)" % (v, self.pin)

    def get_PUPDR(self):
        '''return one of FLOATING, PULLUP, PULLDOWN'''
        values = ['FLOATING', 'PULLUP', 'PULLDOWN']
        v = 'FLOATING'
        if self.is_CS():
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
        str = ''
        if self.af is not None:
            str += " AF%u" % self.af
        if self.type.startswith('ADC1'):
            str += " ADC1_IN%u" % get_ADC1_chan(mcu_type, self.portpin)
        if self.extra_value('PWM', type=int):
            str += " PWM%u" % self.extra_value('PWM', type=int)
        return "P%s%u %s %s%s" % (self.port, self.pin, self.label, self.type,
                                  str)


# setup default as input pins
for port in ports:
    portmap[port] = []
    for pin in range(pincount[port]):
        portmap[port].append(generic_pin(port, pin, None, 'INPUT', []))


def get_config(name, column=0, required=True, default=None, type=None):
    '''get a value from config dictionary'''
    if not name in config:
        if required and default is None:
            error("missing required value %s in hwdef.dat" % name)
        return default
    if len(config[name]) < column + 1:
        error("missing required value %s in hwdef.dat (column %u)" % (name,
                                                                      column))
    ret = config[name][column]
    if type is not None:
        try:
            ret = type(ret)
        except Exception:
            error("Badly formed config value %s (got %s)" % (name, ret))
    return ret


def write_mcu_config(f):
    '''write MCU config defines'''
    f.write('// MCU type (ChibiOS define)\n')
    f.write('#define %s_MCUCONF\n' % get_config('MCU'))
    f.write('#define %s\n\n' % get_config('MCU', 1))
    f.write('// crystal frequency\n')
    f.write('#define STM32_HSECLK %sU\n\n' % get_config('OSCILLATOR_HZ'))
    f.write('// UART used for stdout (printf)\n')
    f.write('#define HAL_STDOUT_SERIAL %s\n\n' % get_config('STDOUT_SERIAL'))
    f.write('// baudrate used for stdout (printf)\n')
    f.write('#define HAL_STDOUT_BAUDRATE %u\n\n' % get_config(
        'STDOUT_BAUDRATE', type=int))
    if 'SDIO' in bytype:
        f.write('// SDIO available, enable POSIX filesystem support\n')
        f.write('#define USE_POSIX\n\n')
        f.write('#define HAL_USE_SDC TRUE\n')
    else:
        f.write('#define HAL_USE_SDC FALSE\n')
    if 'OTG1' in bytype:
        f.write('#define STM32_USB_USE_OTG1                  TRUE\n')
        f.write('#define HAL_USE_USB TRUE\n')
        f.write('#define HAL_USE_SERIAL_USB TRUE\n')
    if 'OTG2' in bytype:
        f.write('#define STM32_USB_USE_OTG2                  TRUE\n')
    # write any custom STM32 defines
    for d in alllines:
        if d.startswith('STM32_'):
            f.write('#define %s\n' % d)
        if d.startswith('define '):
            f.write('#define %s\n' % d[7:])
    flash_size = get_config('FLASH_SIZE_KB', type=int)
    f.write('#define BOARD_FLASH_SIZE %u\n' % flash_size)
    f.write('#define CRT1_AREAS_NUMBER 1\n')
    if mcu_type in ['STM32F427xx', 'STM32F405xx']:
        def_ccm_size = 64
    else:
        def_ccm_size = None
    ccm_size = get_config(
        'CCM_RAM_SIZE_KB', default=def_ccm_size, required=False, type=int)
    if ccm_size is not None:
        f.write('#define CCM_RAM_SIZE %u\n' % ccm_size)
    f.write('\n')


def write_ldscript(fname):
    '''write ldscript.ld for this board'''
    flash_size = get_config('FLASH_SIZE_KB', type=int)

    # space to reserve for bootloader and storage at start of flash
    flash_reserve_start = get_config(
        'FLASH_RESERVE_START_KB', default=16, type=int)

    # space to reserve for storage at end of flash
    flash_reserve_end = get_config('FLASH_RESERVE_END_KB', default=0, type=int)

    # ram size
    ram_size = get_config('RAM_SIZE_KB', default=192, type=int)

    flash_base = 0x08000000 + flash_reserve_start * 1024
    flash_length = flash_size - (flash_reserve_start + flash_reserve_end)

    print("Generating ldscript.ld")
    f = open(fname, 'w')
    f.write('''/* generated ldscript.ld */
MEMORY
{
    flash : org = 0x%08x, len = %uK
    ram0  : org = 0x20000000, len = %uk
}

INCLUDE common.ld
''' % (flash_base, flash_length, ram_size))


def write_USB_config(f):
    '''write USB config defines'''
    if not 'OTG1' in bytype:
        return;
    f.write('// USB configuration\n')
    f.write('#define HAL_USB_VENDOR_ID %s\n' % get_config('USB_VENDOR', default=0x0483)) # default to ST
    f.write('#define HAL_USB_PRODUCT_ID %s\n' % get_config('USB_PRODUCT', default=0x5740))
    f.write('#define HAL_USB_STRING_MANUFACTURER "%s"\n' % get_config("USB_STRING_MANUFACTURER", default="ArduPilot"))
    f.write('#define HAL_USB_STRING_PRODUCT "%s"\n' % get_config("USB_STRING_PRODUCT", default="%BOARD%"))
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
        if not bus.startswith('SPI') or not bus in spi_list:
            error("Bad SPI bus in SPIDEV line %s" % dev)
        if not devid.startswith('DEVID') or not is_int(devid[5:]):
            error("Bad DEVID in SPIDEV line %s" % dev)
        if not cs in bylabel or not bylabel[cs].is_CS():
            error("Bad CS pin in SPIDEV line %s" % dev)
        if not mode in ['MODE0', 'MODE1', 'MODE2', 'MODE3']:
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
    for t in bytype.keys():
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
            '#define HAL_SPI%u_CONFIG { &SPID%u, %u, STM32_SPI_SPI%u_TX_DMA_STREAM, STM32_SPI_SPI%u_RX_DMA_STREAM }\n'
            % (n, n, n, n, n))
    f.write('#define HAL_SPI_BUS_LIST %s\n\n' % ','.join(devlist))
    write_SPI_table(f)


def write_UART_config(f):
    '''write UART config defines'''
    get_config('UART_ORDER')
    uart_list = config['UART_ORDER']
    f.write('\n// UART configuration\n')

    # write out driver declarations for HAL_ChibOS_Class.cpp
    devnames = "ABCDEFGH"
    sdev = 0
    for dev in uart_list:
        idx = uart_list.index(dev)
        if dev == 'EMPTY':
            f.write('#define HAL_UART%s_DRIVER Empty::UARTDriver uart%sDriver\n' %
                (devnames[idx], devnames[idx]))
        else:
            f.write(
                '#define HAL_UART%s_DRIVER ChibiOS::UARTDriver uart%sDriver(%u)\n'
                % (devnames[idx], devnames[idx], sdev))
            sdev += 1
    for idx in range(len(uart_list), 6):
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
    else:
        f.write('#define HAL_WITH_IO_MCU 0\n')
    f.write('\n')

    need_uart_driver = False
    devlist = []
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
        if dev + "_RTS" in bylabel:
            p = bylabel[dev + '_RTS']
            rts_line = 'PAL_LINE(GPIO%s,%uU)' % (p.port, p.pin)
        else:
            rts_line = "0"
        if dev.startswith('OTG'):
            f.write(
                '#define HAL_%s_CONFIG {(BaseSequentialStream*) &SDU1, true, false, 0, 0, false, 0, 0}\n'
                % dev)
        else:
            need_uart_driver = True
            f.write(
                "#define HAL_%s_CONFIG { (BaseSequentialStream*) &SD%u, false, "
                % (dev, n))
            f.write("STM32_%s_RX_DMA_CONFIG, STM32_%s_TX_DMA_CONFIG, %s}\n" %
                    (dev, dev, rts_line))
    f.write('#define HAL_UART_DEVICE_LIST %s\n\n' % ','.join(devlist))
    if not need_uart_driver:
        f.write('#define HAL_USE_SERIAL FALSE\n')


def write_I2C_config(f):
    '''write I2C config defines'''
    if not 'I2C_ORDER' in config:
        error("Missing I2C_ORDER config")
    i2c_list = config['I2C_ORDER']
    f.write('// I2C configuration\n')
    if len(i2c_list) == 0:
        f.write('#define HAL_USE_I2C FALSE\n')
        return
    devlist = []
    for dev in i2c_list:
        if not dev.startswith('I2C') or dev[3] not in "1234":
            error("Bad I2C_ORDER element %s" % dev)
        if dev + "_SCL" in bylabel:
            p = bylabel[dev + "_SCL"]
            f.write(
                '#define HAL_%s_SCL_AF %d\n' % (dev, p.af)
            )
        n = int(dev[3:])
        devlist.append('HAL_I2C%u_CONFIG' % n)
        f.write(
            '#define HAL_I2C%u_CONFIG { &I2CD%u, STM32_I2C_I2C%u_RX_DMA_STREAM, STM32_I2C_I2C%u_TX_DMA_STREAM }\n'
            % (n, n, n, n))
    f.write('#define HAL_I2C_DEVICE_LIST %s\n\n' % ','.join(devlist))


def write_PWM_config(f):
    '''write PWM config defines'''
    rc_in = None
    alarm = None
    pwm_out = []
    pwm_timers = []
    for l in bylabel.keys():
        p = bylabel[l]
        if p.type.startswith('TIM'):
            if p.has_extra('RCIN'):
                rc_in = p
            elif p.has_extra('ALARM'):
                alarm = p
            else:
                if p.extra_value('PWM', type=int) is not None:
                    pwm_out.append(p)
                if p.type not in pwm_timers:
                    pwm_timers.append(p.type)
    if rc_in is not None:
        a = rc_in.label.split('_')
        chan_str = a[1][2:]
        timer_str = a[0][3:]
        if chan_str[-1] == 'N':
            # it is an inverted channel
            f.write('#define HAL_RCIN_IS_INVERTED\n')
            chan_str = chan_str[:-1]
        if not is_int(chan_str) or not is_int(timer_str):
            error("Bad timer channel %s" % rc_in.label)
        if int(chan_str) not in [1, 2]:
            error(
                "Bad channel number, only channel 1 and 2 supported for RCIN")
        n = int(a[0][3:])
        dma_chan_str = rc_in.extra_prefix('DMA_CH')[6:]
        dma_chan = int(dma_chan_str)
        f.write('// RC input config\n')
        f.write('#define HAL_USE_ICU TRUE\n')
        f.write('#define STM32_ICU_USE_TIM%u TRUE\n' % n)
        f.write('#define RCIN_ICU_TIMER ICUD%u\n' % n)
        f.write(
            '#define RCIN_ICU_CHANNEL ICU_CHANNEL_%u\n' % int(chan_str))
        f.write('#define STM32_RCIN_DMA_CHANNEL %u' % dma_chan)
        f.write('\n')
    if alarm is not None:

        a = alarm.label.split('_')
        chan_str = a[1][2:]
        timer_str = a[0][3:]
        if not is_int(chan_str) or not is_int(timer_str):
            error("Bad timer channel %s" % alarm.label)
        n = int(timer_str)
        f.write('\n')
        f.write('// Alarm PWM output config\n')
        f.write('#define STM32_PWM_USE_TIM%u TRUE\n' % n)
        f.write('#define STM32_TIM%u_SUPPRESS_ISR\n' % n)

        chan_mode = [
            'PWM_OUTPUT_DISABLED', 'PWM_OUTPUT_DISABLED',
            'PWM_OUTPUT_DISABLED', 'PWM_OUTPUT_DISABLED'
        ]
        chan = int(chan_str)
        if chan not in [1, 2, 3, 4]:
            error("Bad channel number %u for ALARM PWM %s" % (chan, p))
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
        n = int(t[3])
        f.write('#define STM32_PWM_USE_TIM%u TRUE\n' % n)
        f.write('#define STM32_TIM%u_SUPPRESS_ISR\n' % n)
    f.write('\n')
    f.write('// PWM output config\n')
    groups = []
    for t in sorted(pwm_timers):
        group = len(groups) + 1
        n = int(t[3])
        chan_list = [255, 255, 255, 255]
        chan_mode = [
            'PWM_OUTPUT_DISABLED', 'PWM_OUTPUT_DISABLED',
            'PWM_OUTPUT_DISABLED', 'PWM_OUTPUT_DISABLED'
        ]
        for p in pwm_out:
            if p.type != t:
                continue
            chan_str = p.label[7]
            if not is_int(chan_str):
                error("Bad channel for PWM %s" % p)
            chan = int(chan_str)
            if chan not in [1, 2, 3, 4]:
                error("Bad channel number %u for PWM %s" % (chan, p))
            pwm = p.extra_value('PWM', type=int)
            chan_list[chan - 1] = pwm - 1
            chan_mode[chan - 1] = 'PWM_OUTPUT_ACTIVE_HIGH'
        groups.append('HAL_PWM_GROUP%u' % group)
        if n in [1, 8]:
            # only the advanced timers do 8MHz clocks
            advanced_timer = 'true'
        else:
            advanced_timer = 'false'
        pwm_clock = 1000000
        period = 20000 * pwm_clock / 1000000
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
          }, 0, 0}, &PWMD%u}\n''' %
                (group, advanced_timer, chan_list[0], chan_list[1],
                 chan_list[2], chan_list[3], pwm_clock, period, chan_mode[0],
                 chan_mode[1], chan_mode[2], chan_mode[3], n))
    f.write('#define HAL_PWM_GROUPS %s\n\n' % ','.join(groups))


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
    for l in bylabel:
        p = bylabel[l]
        gpio = p.extra_value('GPIO', type=int)
        if gpio is None:
            continue
        # see if it is also a PWM pin
        pwm = p.extra_value('PWM', type=int, default=0)
        port = p.port
        pin = p.pin
        gpios.append((gpio, pwm, port, pin, p))
    gpios = sorted(gpios)
    f.write('#define HAL_GPIO_PINS { \\\n')
    for (gpio, pwm, port, pin, p) in gpios:
        f.write('{ %3u, true, %2u, PAL_LINE(GPIO%s, %2uU) }, /* %s */ \\\n' %
                (gpio, pwm, port, pin, p))
    # and write #defines for use by config code
    f.write('}\n\n')
    f.write('// full pin define list\n')
    for l in sorted(bylabel.keys()):
        p = bylabel[l]
        label = p.label
        label = label.replace('-', '_')
        f.write('#define HAL_GPIO_PIN_%-20s PAL_LINE(GPIO%s,%uU)\n' %
                (label, p.port, p.pin))
    f.write('\n')


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

def get_dma_exclude(periph_list):
    '''return list of DMA devices to exclude from DMA'''
    dma_exclude = []
    for periph in periph_list:
        if periph not in bylabel:
            continue
        p = bylabel[periph]
        if p.has_extra('NODMA'):
            dma_exclude.append(periph)
    return dma_exclude

def write_hwdef_header(outfilename):
    '''write hwdef header file'''
    print("Writing hwdef setup in %s" % outfilename)
    f = open(outfilename, 'w')

    f.write('''/*
 generated hardware definitions from hwdef.dat - DO NOT EDIT
*/

#pragma once

''')

    write_mcu_config(f)
    write_USB_config(f)
    write_I2C_config(f)
    write_SPI_config(f)
    write_PWM_config(f)
    write_ADC_config(f)
    write_GPIO_config(f)

    write_peripheral_enable(f)
    write_prototype_file()

    dma_resolver.write_dma_header(f, periph_list, mcu_type,
                                  dma_exclude=get_dma_exclude(periph_list),
                                  dma_priority=get_config('DMA_PRIORITY',default=''),
                                  dma_noshare=get_config('DMA_NOSHARE',default=''))

    write_UART_config(f)

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
        if type.startswith('TIM') and p.has_extra('RCIN'):
            label = p.label
            if label[-1] == 'N':
                label = label[:-1]
            peripherals.append(label)
        done.add(type)
    return peripherals


def process_line(line):
    '''process one line of pin definition file'''
    global allpins
    a = shlex.split(line)
    # keep all config lines for later use
    alllines.append(line)

    if a[0].startswith('P') and a[0][1] in ports and a[0] in config:
        print("WARNING: Pin %s redefined" % a[0])
    
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
            error("Bad pin line: %s" % a)
            return

        p = generic_pin(port, pin, label, type, extra)
        portmap[port][pin] = p
        allpins.append(p)
        if not type in bytype:
            bytype[type] = []
        bytype[type].append(p)
        bylabel[label] = p
        af = get_alt_function(mcu_type, a[0], label)
        if af is not None:
            p.af = af
    if a[0] == 'SPIDEV':
        spidev.append(a[1:])
    if a[0] == 'undef':
        print("Removing %s" % a[1])
        config.pop(a[1], '')
        bytype.pop(a[1],'')
        bylabel.pop(a[1],'')
        #also remove all occurences of defines in previous lines if any
        for line in alllines[:]:
            if line.startswith('define') and a[1] in line:
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

if not "MCU" in config:
    error("Missing MCU type in config")

mcu_type = get_config('MCU', 1)
print("Setup for MCU %s" % mcu_type)

# build a list for peripherals for DMA resolver
periph_list = build_peripheral_list()

# write out hwdef.h
write_hwdef_header(os.path.join(outdir, "hwdef.h"))

# write out ldscript.ld
write_ldscript(os.path.join(outdir, "ldscript.ld"))

