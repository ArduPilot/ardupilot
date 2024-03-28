#!/usr/bin/env python
'''
convert a betaflight unified configuration file into a hwdef.dat
currently very approximate, file requires cleanup afterwards
must be run from within a source tree as it relies on other other python files

code by Andy Piper <github@andypiper.com>
'''

import sys, re
import argparse

timers = {}
timers_by_name = {}
resources = {}
features = []
chip_select = {}
chip_select_by_type = {}
defines = {}

functions = {
    "SPI" : {},
    "MOTOR" : {},
    "BEEPER" : {},
    "SERVO" : {},
    "LED" : {},
    "SERIAL" : {},
    "SPI" : {},
    "I2C" : {},
    "ADC" : {},
    "CAMERA" : {},
}
settings = {}

dma_noshare = {}

# betaflight sensor alignment is a mysterious art
# some of these might be right but you should always check
alignment = {
    "CW0" : "ROTATION_YAW_270",
    "CW90" : "ROTATION_NONE",
    "CW180" : "ROTATION_YAW_90",
    "CW270" : "ROTATION_YAW_180",
    "CW0FLIP" : "ROTATION_ROLL_180_YAW_270",
    "CW90FLIP" : "ROTATION_ROLL_180",
    "CW180FLIP" : "ROTATION_ROLL_180_YAW_90",
    "CW270FLIP" : "ROTATION_PITCH_180",
    "DEFAULT" : "ROTATION_NONE",
}

parser = argparse.ArgumentParser("convert_betaflight_unified.py")
parser.add_argument('-I', '--id', type=str, default=None, help='Board id')
parser.add_argument('fname', help='Betaflight unified config file')

args = parser.parse_args()

def convert_pin(pin):
    pinnum = str(int(pin[1:]))
    return 'P' + pin[0] + pinnum

def error(str):
    '''show an error and exit'''
    print("Error: " + str)
    sys.exit(1)

def get_ADC1_chan(mcu, pin):
    '''return ADC1 channel for an analog pin'''
    import importlib
    try:
        lib = importlib.import_module("STM32" + mcu + "xx")
        ADC1_map = lib.ADC1_map
    except ImportError:
        error("Unable to find ADC1_Map for MCU %s" % mcu)

    if pin not in ADC1_map:
        error("Unable to find ADC1 channel for pin %s" % pin)
    return ADC1_map[pin]


def write_osd_config(f, bus):
    f.write('''
# OSD setup
SPIDEV osd SPI%s DEVID1 OSD1_CS   MODE0  10*MHZ  10*MHZ

define OSD_ENABLED 1
define HAL_OSD_TYPE_DEFAULT 1
ROMFS_WILDCARD libraries/AP_OSD/fonts/font*.bin
''' % bus)

def write_flash_config(f, bus):
    f.write('''
# Dataflash setup
SPIDEV dataflash SPI%s DEVID1 FLASH1_CS     MODE3 104*MHZ 104*MHZ

define HAL_LOGGING_DATAFLASH_ENABLED 1
''' % bus)

def write_imu_config(f, n):
    global dma_noshare
    global dma_pri
    bus = settings['gyro_' + n + '_spibus']
    sensor_align = 'gyro_' + n + '_sensor_align'
    if sensor_align in settings:
        align = settings[sensor_align]
    else:
        align = 'CW0'
    f.write('''
# IMU setup
SPIDEV imu%s   SPI%s DEVID1 GYRO%s_CS   MODE3   1*MHZ   8*MHZ
''' % (n, bus, n))

    c = 0
    for define in defines:
        for imudefine in ['USE_GYRO_SPI_', 'USE_ACCGYRO_']:
            if define.startswith(imudefine):
                imu = define[len(imudefine):]
                c = c + 1
                if c == int(n):
                    if imu == 'ICM42688P':
                        imudriver = 'Invensensev3'
                    elif imu == 'BMI270':
                        imudriver = 'BMI270'
                    else:
                        imudriver = 'Invensense'
                    f.write('''
IMU %s SPI:imu%s %s
''' % (imudriver, n, alignment[align]))

    dma = "SPI" + bus + "*"
    dma_noshare[dma] = dma

def convert_file(fname, board_id):

    f = open("hwdef.dat", 'w')

    lines = open(fname, 'r').readlines()
    mcu = ""
    mcuclass = ""
    board_name = ""
    flash_size = 2048
    for i in range(len(lines)):
        line = lines[i]
        if line.__contains__('STM32'):
            result = re.search("STM32(..)(..) ", line)
            mcuclass = result.group(1)
            mcu = result.group(1) + result.group(2)
        if line.startswith('board_name'):
            board_name = line.split()[1]

    if mcuclass == "F4":
        if mcu == "F405":
            flash_size = 1024
        reserve_start = 48
    elif mcuclass == "F7":
        if mcu == "F745":
            flash_size = 1024
        reserve_start = 96
    elif mcuclass == "H7":
        reserve_start = 384
    else:
        mcuclass = "F4"
        mcu = "F405"
        flash_size = 1024
        reserve_start = 48

    # preamble

    f.write('''
# hw definition file for processing by chibios_hwdef.py
# for %s hardware.
# thanks to betaflight for pin information

# MCU class and specific type
MCU STM32%sxx STM32%sxx

# board ID for firmware load
APJ_BOARD_ID %s

# crystal frequency, setup to use external oscillator
OSCILLATOR_HZ 8000000

FLASH_SIZE_KB %u

# bootloader takes first sector
FLASH_RESERVE_START_KB %u

define HAL_STORAGE_SIZE 16384
define STORAGE_FLASH_PAGE 1
''' % (board_name, mcuclass, mcu, board_id, flash_size, reserve_start))

    for i in range(len(lines)):
        line = lines[i]
        if line.startswith('resource'):
            a = line.split()

            if a[3] == 'NONE':
                continue
            # function, number, pin
            pin = convert_pin(a[3])
            resource = [ a[2] , pin, a[1].split('_')[0], a[1] ]
            resources[a[1]] = resource
            if resource[2] in functions.keys():
                functions[resource[2]][resource[1]] = resource

            if (resource[3].endswith("_CS")):
                chip_select[resource[1]] = resource
                chip_select_by_type[resource[3] + resource[0]] = resource[0]

            print("resource: %s %s %s %s" % (resource[0], resource[1], resource[2], resource[3]))

        elif line.startswith('timer'):
            # parse the comment, bit hacky but seems to work
            # # pin A08: TIM1 CH1 (AF1)
            pindef = lines[i+1].split()
            # timer A08 AF1
            a = line.split()
            # function, number, pin
            pin = convert_pin(a[1])
            timer = [ pin, pindef[3] , pindef[4] ]
            timers[pin] = timer
            timers_by_name[pindef[3]] = pin

            print("timer: %s %s %s" % (timer[0], timer[1], timer[2]))

        elif line.startswith('feature'):
            a = line.split()
            features.append(a[1])

            print("feature: %s" % (a[1]))

        elif line.startswith('set'):
            a = line.split()
            settings[a[1]] = a[3]

            print("settings: %s %s" % (a[1], a[3]))

        elif line.startswith('#define'):
            a = line.split()
            defines[a[1]] = a[1]

            print("define: %s" % (a[1]))

    #open(fname, 'w').write(''.join(lines))

    # system timer
    if 'TIM2' in timers_by_name and mcuclass != 'H7':
        f.write("\nSTM32_ST_USE_TIMER 5\n")
    elif 'TIM5' in timers_by_name:
        f.write("\nSTM32_ST_USE_TIMER 2\n")

    f.write("\n# SPI devices\n")
    # PIN FN SPI
    spin = -1
    for spi in sorted(functions["SPI"].values()):
        if (spin != int(spi[0])):
            spin = int(spi[0])
            f.write("\n# SPI%s\n" % spin)
        f.write("%s SPI%s_%s SPI%s\n" % (spi[1], spin, spi[3].split('_')[1], spin))

    f.write("\n# Chip select pins\n")
    for cs in chip_select.values():
        f.write("%s %s%s_CS CS\n" % (cs[1], cs[2], int(cs[0])))

    beeper = list(functions["BEEPER"].values())[0]
    f.write('''\n# Beeper
%s BUZZER OUTPUT GPIO(80) LOW
define HAL_BUZZER_PIN 80
''' % beeper[1])

    f.write("\n# SERIAL ports\n")
    usarts = "SERIAL_ORDER OTG1"
    uartn = 1
    for uart in sorted(list(set([uart[0] for uart in functions["SERIAL"].values()]))):
        while (uartn < int(uart)):
            uartn = uartn + 1
            usarts += " EMPTY"
        name = ("USART" if int(uartn) < 4 or int(uartn) == 6 else "UART") + uart
        usarts += (" " + name)
        uartn = uartn + 1
    
    f.write(usarts)
    f.write('''
# PA10 IO-debug-console
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1
''')

    # PIN FN UART
    serialn = ""
    for uart in sorted(functions["SERIAL"].values()):
        if (serialn != uart[0]):
            serialn = uart[0]
            name = "USART" if int(serialn) < 4 or int(serialn) == 6 else "UART"
            name += serialn
            f.write("\n# %s\n" % name)
        # no need for all UARTs to have DMA. Picking the first four is better than nothing
        dma = ""
        if int(serialn) > 4:
            dma = " NODMA"
        f.write("%s %s_%s %s%s\n" % (uart[1], name, uart[3].split('_')[1], name, dma))

    f.write("\n# I2C ports\n")
    i2cs = "I2C_ORDER"
    i2cn = 1
    for i2c in sorted(list(set([i2c[0] for i2c in functions["I2C"].values()]))):
        i2cs += (" " + "I2C" + i2c)
        i2cn = i2cn + 1
    
    f.write(i2cs)

    # PIN FN I2C
    i2cn = ""
    for i2c in sorted(functions["I2C"].values()):
        if (i2cn != i2c[0]):
            i2cn = i2c[0]
            name = "I2C" + i2cn
            f.write("\n# %s\n" % name)
        f.write("%s %s_%s %s\n" % (i2c[1], name, i2c[3].split('_')[1], name))

    # PIN FN I2C
    f.write("\n# Servos\n")
    servon = 0
    for servo in sorted(functions["SERVO"].values()) + sorted(functions["CAMERA"].values()):
        f.write("%s %s%u OUTPUT GPIO(%u) LOW\n" % (servo[1], servo[2], int(servo[0]), servon+70))
        f.write("define RELAY%u_PIN_DEFAULT %u\n" % (servon + 2, servon+70))
        servon = servon+1

    f.write("\n# ADC ports\n")
    # PIN FN ADC
    adcn = ""
    for adc in sorted(functions["ADC"].values()):
        if (adcn != adc[0]):
            adcn = adc[0]
            name = "ADC" + adcn
            f.write("\n# %s\n" % name)
        if (adc[3] == "ADC_BATT"):
            f.write("%s BATT_VOLTAGE_SENS %s SCALE(1)\n" % (adc[1], name))
            f.write('''define HAL_BATT_VOLT_PIN %s
define HAL_BATT_VOLT_SCALE 11.0
''' % (get_ADC1_chan(mcu, adc[1])))
        elif (adc[3] == "ADC_CURR"):
            f.write("%s BATT_CURRENT_SENS %s SCALE(1)\n" % (adc[1], name))
            f.write('''define HAL_BATT_CURR_PIN %s
define HAL_BATT_CURR_SCALE %.1f
''' % (get_ADC1_chan(mcu, adc[1]), 10000 / int(settings['ibata_scale'])))
        elif (adc[3] == "ADC_RSSI"):
            f.write("%s RSSI_ADC %s\n" % (adc[1], name))
            f.write("define BOARD_RSSI_ANA_PIN %s\n" % (get_ADC1_chan(mcu, adc[1])))

    # define default battery setup
    f.write("define HAL_BATT_MONITOR_DEFAULT 4\n")

    f.write("\n# MOTORS\n")
    nmotors = 0
    # PIN  TIMx_CHy TIMx PWM(p) GPIO(g)
    for pin, motor in functions["MOTOR"].items():
        timer = timers[pin]
        nmotors = max(nmotors, int(motor[0]))
        # for safety don't share the _UP channel
        dma_noshare[timer[1] + '_UP'] =  timer[1] + '_UP'
        f.write("%s %s_%s %s PWM(%s) GPIO(%s)" % (motor[1], timer[1], timer[2], timer[1], motor[0], 49+int(motor[0])))
        # on H7 we can reasonably safely assign bi-dir channel
        if mcuclass == "H7" and (int(timer[2][2:]) == 1 or int(timer[2][2:]) == 3):
            f.write(" BIDIR # M%s\n" % (motor[0]))
        else:
            f.write("       # M%s\n" % (motor[0]))

    # LEDs
    f.write("\n# LEDs\n")
    for led in sorted(functions["LED"].values()):
        if (led[3].endswith('_STRIP')):
            pin = led[1]
            if not pin in timers.keys():
                continue
            timer = timers[pin]
            nmotors = nmotors+1
            f.write("%s %s_%s %s PWM(%s) GPIO(%s) # M%s\n" % (led[1], timer[1], timer[2], timer[1], nmotors, 49+nmotors, nmotors))
            continue
        ledn = int(led[0])
        f.write('''
%s LED%u OUTPUT LOW GPIO(%u)
define HAL_GPIO_%s_LED_PIN %u
''' % (led[1], ledn-1, 89+ledn, chr(ledn+64), 89+ledn))
    f.write("define HAL_GPIO_LED_OFF 1\n")

    # write out devices
    if 'blackbox_device' in settings and settings['blackbox_device'] == 'SPIFLASH' or 'USE_FLASH' in defines:
        write_flash_config(f, settings['flash_spi_bus'])

    if 'max7456_spi_bus' in settings:
        write_osd_config(f, settings['max7456_spi_bus'])

    if 'baro_i2c_device' in settings:
        for define in defines:
            if define.startswith('USE_BARO_'):
                baro = define[len('USE_BARO_'):]
                f.write('''
# Barometer setup
BARO %s I2C:%s:0x76
        ''' % (baro, int(settings['baro_i2c_device']) - 1))
    else:
        f.write("define HAL_BARO_ALLOW_INIT_NO_BARO 1\n")

    f.write('''
# IMU setup
''')

    if 'gyro_1_spibus' in settings and 'GYRO_CS1' in chip_select_by_type:
        write_imu_config(f, "1")

    if 'gyro_2_spibus' in settings and 'GYRO_CS2' in chip_select_by_type:
        write_imu_config(f, "2")

    if len(dma_noshare) > 0:
        f.write("DMA_NOSHARE")
        for dma in dma_noshare.keys():
            f.write(" " + dma)
        f.write("\n")
        f.write("DMA_PRIORITY")
        for dma in dma_noshare.keys():
            f.write(" " + dma)
        f.write("\n")

    # postamble

    f.write('''
# no built-in compass, but probe the i2c bus for all possible
# external compass types
define ALLOW_ARM_NO_COMPASS
define HAL_PROBE_EXTERNAL_I2C_COMPASSES
define HAL_I2C_INTERNAL_MASK 0
define HAL_COMPASS_AUTO_ROT_DEFAULT 2
define HAL_DEFAULT_INS_FAST_SAMPLE 3
# Motor order implies Betaflight/X for standard ESCs
define HAL_FRAME_TYPE_DEFAULT 12
''')

    f.close()

def convert_bootloader(fname, board_id):

    f = open("hwdef-bl.dat", 'w')

    lines = open(fname, 'r').readlines()
    mcu = ""
    mcuclass = ""
    board_name = ""
    flash_size = 2048
    for i in range(len(lines)):
        line = lines[i]
        if line.__contains__('STM32'):
            result = re.search("STM32(..)(..) ", line)
            mcuclass = result.group(1)
            mcu = result.group(1) + result.group(2)
        if line.startswith('board_name'):
            board_name = line.split()[1]

    if mcuclass == "F4":
        if mcu == "F405":
            flash_size = 1024
        reserve_start = 48
    elif mcuclass == "F7":
        if mcu == "F745":
            flash_size = 1024
        reserve_start = 96
    elif mcuclass == "H7":
        reserve_start = 384
    else:
        mcuclass = "F4"
        mcu = "F405"
        flash_size = 1024
        reserve_start = 48

    # preamble

    f.write('''
# hw definition file for processing by chibios_hwdef.py
# for %s hardware.
# thanks to betaflight for pin information

# MCU class and specific type
MCU STM32%sxx STM32%sxx

# board ID for firmware load
APJ_BOARD_ID %s

# crystal frequency, setup to use external oscillator
OSCILLATOR_HZ 8000000

FLASH_SIZE_KB %u

# bootloader starts at zero offset
FLASH_RESERVE_START_KB 0

# the location where the bootloader will put the firmware
FLASH_BOOTLOADER_LOAD_KB %u

# order of UARTs (and USB)
SERIAL_ORDER OTG1

# PA10 IO-debug-console
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1

PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# default to all pins low to avoid ESD issues
DEFAULTGPIO OUTPUT LOW PULLDOWN

''' % (board_name, mcuclass, mcu, board_id, flash_size, reserve_start))

    f.write("\n# Chip select pins\n")
    for cs in chip_select.values():
        f.write("%s %s%s_CS CS\n" % (cs[1], cs[2], int(cs[0])))

    for led in sorted(functions["LED"].values()):
        f.write('''
%s LED_BOOTLOADER OUTPUT LOW
define HAL_LED_ON 0
''' % led[1])
        break

    f.close()

fname=args.fname

if args.id is None:
    board_id = input('Board id? ')
else:
    board_id = args.id

convert_file(fname, board_id)
convert_bootloader(fname, board_id)
