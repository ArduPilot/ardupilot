experimental ap-periph support for ESP32-S3

how?
setup idf/submodule, run it, etc then..
...
source ~/ardupilot/modules/esp_idf/export.sh 
rm -rf build
./waf configure --board=esp32s3buzz_periph
./waf AP_Periph -v --upload


-what works?   it boots, mounts sd-card, starts wifitasks, gets scheduler up. etc

-what doesn't work?  CAN, yet.

boot log and compile log/s below...

S3 pin notes: 

// on esp32s3, some pins aren't defined, such as 'GPIO_NUM_23', just pick a different pin. 
// disallowed:0,3, 19,20,  22,23,24,25, 26,27,28,29,30,31,32, 39,40,41,42 , 43,44, 45,46, 48  esp32s3 - range 0-48. 
// 22,23,24,25 - these are totally absent.
// 19,20  - these are D+/D- for the integrated USB port/peripheral that does CDC or OTG etc.   the 'USB' labled one near the reset button.
// 43(u0tx),44(u0rx) , also has 15(u0rts),16(u0cts) - these are D+/D- for the 'uart' port, that is not smart, and the flow control lines, these are also known as U0TXD,U0RXD the 'USB' labled one near the reset button.
//  26,27,28,29,30,31,32 - GPIO26-32 are usually used for SPI flash and PSRAM and not recommended for other uses.
//  try to avoid 0,3,45,46 as GPIO0, GPIO3, GPIO45 and GPIO46 are strapping pins, and they might, in some cases be used, only if u are careful.
//'JTAG communication will likely fail, if configuration of JTAG pins is changed by user application.' / GPIO39 GPIO40 GPIO41 GPIO42
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/gpio.html
// on the Devkit-M board GPIO48 is connected to the RGB LED, so its usable on oter hardware without this LED, but worth noting here.

// some devices like SPI, can get speed boost by 'bypassing GPIO matrix' for INPUT to these devices.
//
// ... Two registers must be configured in order to bypass: 
//1. IO_MUX_MCU_SEL for the GPIO must be set to reqd pin functin from Section 5.12
//2. Clear GPIO_SIGX_IN_SEL to route the input directly to the peripheral.
// for OUTPUT from these devices, only the first one must be done...
//1. Two registers must be configured in order to bypass: IO_MUX_MCU_SEL for the GPIO must be set to reqd pin functin from Section 5.12
// so... essentially 5.12 is a table of 'alt functions' like stm has... but we can fallback to using the GPIO matrix, a bit slower, if that doesn't work for us.

// on S3 the IMU will to go on  'SPI3' for the sensors such as bmp280 and mpu9250, as 'SPI0 and SPI1 are used internally to access the ESP32-S3’s attached flash memory', and some pply might want to use a SPI-based SD driver on SPI2 if not using MMC.
// we use 1-wire MMC mode as its faster.

// spi.. Normally, the driver will use the GPIO matrix to route the signals. An exception is made when all signals either can be routed through the IO_MUX or are -1. In that case, the IO_MUX is used, allowing for >40MHz speeds. speeds up to 80MHz on the dedicated SPI pins, without it, using 'matrix', typically full-duplex 26Mhz. The IOMUX pins are indicated in the datasheet by the pins demarked as FSPI*.
from ref manual:
eg output signal 'FSPID_out'       = mosi = 'd' = signal 103, needs enable signal 'FSPID_oe' when GPIO_FUNCX_OEN_SEL=0
eg input signal 'FSPIQ_in'         = miso = 'q' = signal 102
eg output signal 'FSPICLK_out_mux' = clk        = signal 101, needs enable signal 'FSPICLK_oe' when GPIO_FUNCX_OEN_SEL=0
So.. the only way to get super-fast SPI3 is to use:
 gpio pins 11(d),12(clk),13(q) as 'function4', or 
 gpio pins 35(d),36(clk),37(q) as 'function2'.
so, we might as well, use one of those sets nad worry about the software later.. lets use 11,12,13

----------------------------------------------------------------------------------------------
buzzs S3 has 45 GPIO PIN assignments  0-21 and 26-48:
//0.   0,3,45,46 as GPIO0, GPIO3, GPIO45 and GPIO46 are strapping pins
1 //i2c-SCL  - see pin5 for i2c pair.
2 //SDCARD-MMC-D0
//3.   0,3,45,46 as GPIO0, GPIO3, GPIO45 and GPIO46 are strapping pins//
4 RC-INPUT, PPM ?
5 -//i2c-SDA - see pin1 for i2c pair.
6 //uart1-rx serial1/gps?
7 //uart1-tx serial1/gps?
8  //servo-out-1
9  //servo-out-2
10 //servo-out-3
11 // SPI3 'D' / mosi  / SDO - IMU sensor/s,  other SPI devs etc. see 11,12,13,21,33.   GY-91 SDO/SAO
12 // SPI3 'clk' / clk / SCL - IMU sensor/s,  other SPI devs etc. see 11,12,13,21,33    GY-91 SCL
13 // SPI3 'Q' / miso  / SDA - IMU sensor/s,  other SPI devs etc. see 11,12,13,21,33    GY-91 SDA
14 //SDCARD-MMC-CLK
15 //SDCARD-MMC-CMD - caution, flow control line below tied to uart on 43/44
//15.   43(u0tx),44(u0rx) , also has 15(u0rts),16(u0cts) - these are D+/D- for the 'uart' port, that is not smart, and the flow control lines
//16.   43(u0tx),44(u0rx) , also has 15(u0rts),16(u0cts) - these are D+/D- for the 'uart' port, that is not smart, and the flow control lines
16 //servo-out-4 - conflicts with other flow control line, prob ok.
17 //servo-out-5
18 //servo-out-6
//19,20.  - these are D+/D- for the integrated USB port/peripheral that does CDC or OTG
21 // SPI3 'chipselect1' - IMU sensor 1 see 11,12,13,21,33 as well.   GY-91 'NCS' label on conector is wired to the MPU9250's nCS
//22-25. - do not exist
//26,27,28,29,30,31,32. - GPIO26-32 are usually used for SPI flash and PSRAM and not recommended for other uses.
33 // SPI3 'chipselect2' - IMU sensor 2 see 11,12,13,21,33 as well.   GY-91 'CSB' label on conector is wired to the BMP280's CSB
34  //AnalogIn1  (--input only on classic, bi-dir on -S3 )
35  //AnalogIn2  (--input only on classic, bi-dir on -S3 )
36  //AnalogIn3  (--input only on classic, bi-dir on -S3 )
37  //AnalogIn4  (--input only on classic, bi-dir on -S3 )
38  CAN-R  , pin 4 on SN65HVD231/ VP231 (--input only on classic, bi-dir on -S3 )
//39-42. JTAG GPIO39 GPIO40 GPIO41 GPIO42 (--39 is input only on classic , bi-dir on -S3)
//43.   43(u0tx),44(u0rx) , also has 15(u0rts),16(u0cts) - these are D+/D- for the 'uart' port, that is not smart, and the flow control lines
//44.   43(u0tx),44(u0rx) , also has 15(u0rts),16(u0cts) - these are D+/D- for the 'uart' port, that is not smart, and the flow control lines
//45.   0,3,45,46 as GPIO0, GPIO3, GPIO45 and GPIO46 are strapping pins
//46.   0,3,45,46 as GPIO0, GPIO3, GPIO45 and GPIO46 are strapping pins
47   CAN-D , pin 1 on SN65HVD231/ VP231
48. RGB led SK68XXMINI-HS control line.  SK68xx leds are basically WS28xx leds by another manufacturer. ak etca 'neopixel' aka ws2812
//https://learn.watterott.com/kb/ws281x/
----------------------------------------------------------------------------------------------


-------------------------------------------------------------
boot log, to prove it happened:
-------------------------------------------------------------
I (512) system_api: read default base MAC address from EFUSE
OK created task _timer_thread
I (522) wifi:wifi firmware version: 195d59c
I (523) wifi:wifi certification version: v7.0
I (526) wifi:config NVS flash: enabled
I (529) wifi:config nano formating: disabled
OK created task _rcout_thread
I (532) wifi:Init data frame dynamic rx buffer num: 32
I (538) wifi:Init management frame dynamic rx buffer num: 32
I (542) wifi:Init management short buffer num: 32
OK created task _rcin_thread
I (546) wifi:Init dynamic tx buffer num: 32
I (551) wifi:Init static tx FG buffer num: 2
I (554) wifi:Init static rx buffer size: 1600
I (557) wifi:Init static rx buffer num: 10
OK created task _uart_thread
I (560) wifi:Init dynamic rx buffer num: 32
I (566) wifi_init: rx ba win: 6
I (569) wifi_init: tcpip mbox: 32
I (572) wifi_init: udp mbox: 6
I (575) wifi_init: tcp mbox: 6
I (578) wifi_init: tcp tx win: 5744
I (581) wifi_init: tcp rx win: 5744
I (584) wifi_init: tcp mss: 1440
I (588) wifi_init: WiFi IRAM OP enabled
I (591) wifi_init: WiFi RX IRAM OP enabled
Mounting sd 
I (596) SD...: Initializing SD card as SDMMC
I (599) phy_init: phy_version 501,79e7e9b,Apr 19 2022,11:10:08
I (601) gpio: GPIO[14]| InputEn: 0| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:0 
OK created task _io_thread
I (613) gpio: GPIO[15]| InputEn: 0| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:0 
I (622) gpio: GPIO[2]| InputEn: 0| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:0 
OK created task _storage_thread

virtual void HAL_ESP32::run(int, char* const*, AP_HAL::HAL::Callbacks*) const:103 scheduler init() done
sdcard is mounted
I (705) wifi:mode : softAP (84:f7:03:c0:88:05)
I (710) wifi:Total power save buffer number: 16
I (710) wifi:Init max length of beacon: 752/752
I (711) wifi:Init max length of beacon: 752/752
Set up softAP with IP: 192.168.4.1
wifi_init_softap finished. SSID:'ardupilot123' password:'ardupilot123'

static void ESP32::Scheduler::_main_thread(void*):571 end of uarts
virtual void ESP32::UARTDriver::begin(uint32_t, uint16_t, uint16_t):85 
virtual void ESP32::UARTDriver::begin(uint32_t, uint16_t, uint16_t):85 
virtual void ESP32::WiFiDriver::begin(uint32_t, uint16_t, uint16_t):72 TCP state:1
Booting 00000000:00000000 0/0 len=0 0x00000000
MAIN b4 set_system_initialized

2.MAIN thread has ID 45 and 1240 bytes free stack
void ESP32::Scheduler::_run_io():404 initialised _num_io_procs:1
Waiting for dynamic node ID allocation on IF0... (pool 0)
E (869) system_api: eFuse MAC_CUSTOM is empty
static void ESP32::Scheduler::_main_thread(void*):593 _run_timers  started
Waiting for dynamic node ID allocation on IF0... (pool 0)
E (1518) system_api: eFuse MAC_CUSTOM is empty
Waiting for dynamic node ID allocation on IF0... (pool 1)
E (2151) system_api: eFuse MAC_CUSTOM is empty
Waiting for dynamic node ID allocation on IF0... (pool 2)
E (2712) system_api: eFuse MAC_CUSTOM is empty
Waiting for dynamic node ID allocation on IF0... (pool 3)
E (3228) system_api: eFuse MAC_CUSTOM is empty
Waiting for dynamic node ID allocation on IF0... (pool 4)
E (3967) system_api: eFuse MAC_CUSTOM is empty
Waiting for dynamic node ID allocation on IF0... (pool 4)
E (4594) system_api: eFuse MAC_CUSTOM is empty
I (4856) wifi:new:<1,0>, old:<1,1>, ap:<1,1>, sta:<255,255>, prof:1
I (4858) wifi:station: f8:34:41:da:b8:a3 join, AID=1, bg, 20
Waiting for dynamic node ID allocation on IF0... (pool 5)
E (5161) system_api: eFuse MAC_CUSTOM is empty
Waiting for dynamic node ID allocation on IF0... (pool 6)
E (5826) system_api: eFuse MAC_CUSTOM is empty
Waiting for dynamic node ID allocation on IF0... (pool 7)
E (6388) system_api: eFuse MAC_CUSTOM is empty
Waiting for dynamic node ID allocation on IF0... (pool 8)



-------------------------------------------------------------
full compile/flash  log, to prove it happened ( without -v):
-------------------------------------------------------------

buzz@buzz-metabox:~/ardupilot$ rm -rf build/
buzz@buzz-metabox:~/ardupilot$ ./waf configure --board=esp32s3buzz_periph
get_ap_periph_boards? ['CarbonixL496', 'Sierra-F412', 'f103-Airspeed', 'HolybroGPS', 'MatekH743-periph', 'f303-Universal', 'CubeOrange-periph', 'CUAV_GPS', 'Pixracer-periph', 'f103-HWESC', 'HereID', 'f303-M10025', 'ZubaxGNSS', 'Nucleo-L496', 'Nucleo-L476', 'H757I_EVAL', 'Sierra-L431', 'f405-MatekGPS', 'f103-RangeFinder', 'mRo-M10095', 'CubeOrange-periph-heavy', 'f303-PWM', 'FreeflyRTK', 'f103-GPS', 'f303-MatekGPS', 'f103-QiotekPeriph', 'HitecMosaic', 'H757I_EVAL_intf', 'esp32s3buzz_periph', 'f303-M10070', 'f103-Trigger', 'sitl_periph_gps', 'BirdCANdy', 'ARK_GPS', 'f303-GPS', 'HerePro', 'G4-ESC', 'Hitec-Airspeed', 'f103-ADSB', 'f303-HWESC', 'Sierra-F405', 'CubeBlack-periph']
Setting top to                           : /home/buzz/ardupilot 
Setting out to                           : /home/buzz/ardupilot/build 
Autoconfiguration                        : enabled 
Setting board to                         : esp32s3buzz_periph 
Using toolchain                          : xtensa-esp32s3-elf 
Checking for 'g++' (C++ compiler)        : /home/buzz/.espressif/tools/xtensa-esp32s3-elf/esp-2021r2-patch3-8.4.0/xtensa-esp32s3-elf/bin/xtensa-esp32s3-elf-g++ 
Checking for 'gcc' (C compiler)          : /home/buzz/.espressif/tools/xtensa-esp32s3-elf/esp-2021r2-patch3-8.4.0/xtensa-esp32s3-elf/bin/xtensa-esp32s3-elf-gcc 
Checking for c flags '-MMD'              : yes 
Checking for cxx flags '-MMD'            : yes 
CXX Compiler                             : g++ 8.4.0 
BUZZ CMAKE CONFIGURE: ['CMAKE_BLD_DIR'] ${CMAKE} ${CMAKE_FLAGS} ${CMAKE_SRC_DIR} ${CMAKE_VARS} ${CMAKE_GENERATOR_OPTION}
BUZZ CMAKE BUILD: ${CMAKE} --build ${CMAKE_BLD_DIR} --target ${CMAKE_TARGET}
Checking for program 'cmake'             : /usr/bin/cmake 
Checking for program 'ninja, ninja-build' : /usr/bin/ninja 
USING EXPRESSIF IDF:/home/buzz/ardupilot/modules/esp_idf
USING DEFAULT_PARAMETERS:/home/buzz/ardupilot/libraries/AP_HAL_ESP32/boards/defaults.parm
BUZZ generate hwdef.h from .dat /home/buzz/.espressif/python_env/idf4.4_py3.8_env/bin/python '/home/buzz/ardupilot/libraries/AP_HAL_ESP32/hwdef/scripts/esp32_hwdef.py' -D '/home/buzz/ardupilot/build/esp32s3buzz_periph' --params '/home/buzz/ardupilot/libraries/AP_HAL_ESP32/boards/defaults.parm' '/home/buzz/ardupilot/libraries/AP_HAL_ESP32/hwdef/esp32s3buzz_periph/hwdef.dat'
MCU series: esp32  type: classic
Adding environment AP_PERIPH 1
port,pin,p A -- 0 -- PA0 UART4_TX UART4 AF8
port,pin,p A -- 1 -- PA1 UART4_RX UART4 AF8
port,pin,p A -- 2 -- PA2 BATT_VOLTAGE_SENS ADC1 ADC1_IN2
port,pin,p A -- 3 -- PA3 BATT_CURRENT_SENS ADC1 ADC1_IN3
port,pin,p A -- 4 -- PA4 VDD_5V_SENS ADC1 ADC1_IN4
port,pin,p A -- 18 -- PA18 SPI1_SCK SPI1 AF5
port,pin,p A -- 19 -- PA19 SPI1_MISO SPI1 AF10
port,pin,p A -- 23 -- PA23 SPI1_MOSI SPI1 AF3
port,pin,p A -- 26 -- PA26 MAG_CS CS
port,pin,p A -- 5 -- PA5 MPU_CS CS
port,pin,p A -- 8 -- PA8 VDD_5V_PERIPH_EN OUTPUT
port,pin,p A -- 9 -- PA9 VBUS INPUT
port,pin,p A -- 11 -- PA11 OTG_FS_DM OTG1 AF10
port,pin,p A -- 12 -- PA12 OTG_FS_DP OTG1 AF10
port,pin,p A -- 13 -- PA13 JTMS-SWDIO SWD AF0
port,pin,p A -- 14 -- PA14 JTCK-SWCLK SWD AF0
port,pin,p A -- 15 -- PA15 TIM2_CH1 TIM2 AF1
port,pin,p B -- 0 -- PB0 EXTERN_GPIO1 OUTPUT
port,pin,p B -- 1 -- PB1 EXTERN_GPIO2 OUTPUT
port,pin,p B -- 2 -- PB2 BOOT1 INPUT
port,pin,p B -- 3 -- PB3 FMU_SW0 INPUT
port,pin,p B -- 5 -- PB5 VDD_BRICK_nVALID INPUT
port,pin,p B -- 7 -- PB7 VDD_BRICK2_nVALID INPUT
Setup for MCU classic
Writing hwdef setup in /home/buzz/ardupilot/build/esp32s3buzz_periph/hwdef.h
BUZZ EEP2-------------------------------------------------xx
Setting up as AP_Periph
Writing DMA map
No I2C peripherals
Generating ldscript.ld
Checking for env.py
esp32 env set AP_PERIPH=1
esp32 env set DISABLE_SCRIPTING=True
esp32 env set PROCESS_STACK=0x1C00
esp32 env set MAIN_STACK=0x600
esp32 env set IOMCU_FW=0
esp32 env set PERIPH_FW=0
esp32 env set BOARD_FLASH_SIZE=2048
esp32 env set EXTERNAL_PROG_FLASH_MB=0
esp32 env set ENABLE_CRASHDUMP=1
esp32 env set CPU_FLAGS=['-mcpu=cortex-m4', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard', '-DARM_MATH_CM4']
esp32 env set CORTEX=cortex-m4
esp32 env set APP_DESCRIPTOR=MissionPlanner
esp32 env set HAL_NUM_CAN_IFACES=1
esp32 env set APJ_BOARD_ID=9
esp32 env set APJ_BOARD_TYPE=classic
esp32 env set USBID=0x1209/0x5741
esp32 env set BOOTLOADER_EMBED=0
esp32 env set FLASH_RESERVE_START_KB=16
esp32 env set FLASH_TOTAL=2080768
esp32 env set HAS_EXTERNAL_FLASH_SECTIONS=0
esp32 env set CHIBIOS_BUILD_FLAGS=USE_FATFS=no CHIBIOS_STARTUP_MK=os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32f4xx.mk CHIBIOS_PLATFORM_MK=os/hal/ports/STM32/STM32F4xx/platform.mk MCU=cortex-m4
Checking for HAVE_CMATH_ISFINITE          : yes 
Checking for HAVE_CMATH_ISINF             : yes 
Checking for HAVE_CMATH_ISNAN             : yes 
Checking for NEED_CMATH_ISFINITE_STD_NAMESPACE : yes 
Checking for NEED_CMATH_ISINF_STD_NAMESPACE    : yes 
Checking for NEED_CMATH_ISNAN_STD_NAMESPACE    : yes 
Checking for header endian.h                   : not found 
Checking for header byteswap.h                 : not found 
Checking for HAVE_MEMRCHR                      : no 
Configured VSCode Intellisense:                : no 
Checking for program 'python'                  : /home/buzz/.espressif/python_env/idf4.4_py3.8_env/bin/python 
Checking for python version >= 2.7.0           : 3.8.10 
Checking for program 'python'                  : /home/buzz/.espressif/python_env/idf4.4_py3.8_env/bin/python 
Checking for python version >= 2.7.0           : 3.8.10 
Source is git repository                       : yes 
Update submodules                              : yes 
Checking for program 'git'                     : /usr/bin/git 
Checking for program 'xtensa-esp32s3-elf-size' : /home/buzz/.espressif/tools/xtensa-esp32s3-elf/esp-2021r2-patch3-8.4.0/xtensa-esp32s3-elf/bin/xtensa-esp32s3-elf-size 
Benchmarks                                     : disabled 
Unit tests                                     : enabled 
Scripting                                      : disabled 
Scripting runtime checks                       : enabled 
Debug build                                    : disabled 
Coverage build                                 : disabled 
SITL 32-bit build                              : disabled 
Checking for program 'rsync'                   : /usr/bin/rsync 
'configure' finished successfully (0.646s)
buzz@buzz-metabox:~/ardupilot$ ./waf AP_Periph --upload
get_ap_periph_boards? ['HolybroGPS', 'f303-PWM', 'f103-Airspeed', 'Sierra-L431', 'sitl_periph_gps', 'f303-Universal', 'HitecMosaic', 'ARK_GPS', 'f303-MatekGPS', 'f103-GPS', 'CubeBlack-periph', 'f303-GPS', 'H757I_EVAL_intf', 'FreeflyRTK', 'Sierra-F405', 'f303-M10025', 'f103-QiotekPeriph', 'Nucleo-L476', 'Pixracer-periph', 'CubeOrange-periph-heavy', 'CUAV_GPS', 'BirdCANdy', 'CarbonixL496', 'Sierra-F412', 'f303-M10070', 'f103-HWESC', 'HerePro', 'f103-RangeFinder', 'Hitec-Airspeed', 'CubeOrange-periph', 'MatekH743-periph', 'f103-Trigger', 'G4-ESC', 'f303-HWESC', 'mRo-M10095', 'esp32s3buzz_periph', 'H757I_EVAL', 'Nucleo-L496', 'f405-MatekGPS', 'f103-ADSB', 'ZubaxGNSS', 'HereID']
BUZZ CMAKE CONFIGURE: ['CMAKE_BLD_DIR'] ${CMAKE} ${CMAKE_FLAGS} ${CMAKE_SRC_DIR} ${CMAKE_VARS} ${CMAKE_GENERATOR_OPTION}
BUZZ CMAKE BUILD: ${CMAKE} --build ${CMAKE_BLD_DIR} --target ${CMAKE_TARGET}
Waf: Entering directory `/home/buzz/ardupilot/build/esp32s3buzz_periph'
_load_pre_build 1
BUZZ pre_build periph needs dsdl for libcanard..
Generating DSDLC for CANARD: /home/buzz/.espressif/python_env/idf4.4_py3.8_env/bin/python /home/buzz/ardupilot/modules/DroneCAN/dronecan_dsdlc/dronecan_dsdlc.py -O /home/buzz/ardupilot/build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated /home/buzz/ardupilot/modules/DroneCAN/DSDL/ardupilot /home/buzz/ardupilot/modules/DroneCAN/DSDL/com /home/buzz/ardupilot/modules/DroneCAN/DSDL/cuav /home/buzz/ardupilot/modules/DroneCAN/DSDL/dronecan /home/buzz/ardupilot/modules/DroneCAN/DSDL/mppt /home/buzz/ardupilot/modules/DroneCAN/DSDL/uavcan
Checking for env.py
env added AP_PERIPH=1
env added DISABLE_SCRIPTING=True
env added PROCESS_STACK=0x1C00
env added MAIN_STACK=0x600
env added IOMCU_FW=0
env added PERIPH_FW=0
env added BOARD_FLASH_SIZE=2048
env added EXTERNAL_PROG_FLASH_MB=0
env added ENABLE_CRASHDUMP=1
env appended CPU_FLAGS=['-mcpu=cortex-m4', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard', '-DARM_MATH_CM4']
env added CORTEX=cortex-m4
env added APP_DESCRIPTOR=MissionPlanner
env added HAL_NUM_CAN_IFACES=1
env added APJ_BOARD_ID=9
env added APJ_BOARD_TYPE=classic
env added USBID=0x1209/0x5741
env added BOOTLOADER_EMBED=0
env added FLASH_RESERVE_START_KB=16
env added FLASH_TOTAL=2080768
env added HAS_EXTERNAL_FLASH_SECTIONS=0
env added CHIBIOS_BUILD_FLAGS=USE_FATFS=no CHIBIOS_STARTUP_MK=os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32f4xx.mk CHIBIOS_PLATFORM_MK=os/hal/ports/STM32/STM32F4xx/platform.mk MCU=cortex-m4
BUZZ pre_build ARDUPILOT_CMD= AP_Periph
Embedding file io_firmware.bin:Tools/IO_Firmware/iofirmware_lowpolh.bin
BUZZ _build_dynamic_sources
BUZZ PERIPH_FW is-not-none 0
BUZZ PERIPH_FW ... 1
BUZZ AP_Periph wscript!
get_ap_periph_boards? ['HolybroGPS', 'f303-PWM', 'f103-Airspeed', 'Sierra-L431', 'sitl_periph_gps', 'f303-Universal', 'HitecMosaic', 'ARK_GPS', 'f303-MatekGPS', 'f103-GPS', 'CubeBlack-periph', 'f303-GPS', 'H757I_EVAL_intf', 'FreeflyRTK', 'Sierra-F405', 'f303-M10025', 'f103-QiotekPeriph', 'Nucleo-L476', 'Pixracer-periph', 'CubeOrange-periph-heavy', 'CUAV_GPS', 'BirdCANdy', 'CarbonixL496', 'Sierra-F412', 'f303-M10070', 'f103-HWESC', 'HerePro', 'f103-RangeFinder', 'Hitec-Airspeed', 'CubeOrange-periph', 'MatekH743-periph', 'f103-Trigger', 'G4-ESC', 'f303-HWESC', 'mRo-M10095', 'esp32s3buzz_periph', 'H757I_EVAL', 'Nucleo-L496', 'f405-MatekGPS', 'f103-ADSB', 'ZubaxGNSS', 'HereID']
[1/3] CMake Configure esp-idf
BUZZ CONF CMD: ['/usr/bin/cmake', '/home/buzz/ardupilot/libraries/AP_HAL_ESP32/targets/esp32s3/esp-idf', "-DARDUPILOT_CMD='AP_Periph'", "-DARDUPILOT_LIB='/home/buzz/ardupilot/build/esp32s3buzz_periph/lib'", "-DARDUPILOT_BIN='/home/buzz/ardupilot/build/esp32s3buzz_periph/lib/bin'", '-GNinja']
-- The C compiler identification is GNU 8.4.0
-- The CXX compiler identification is GNU 8.4.0
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Check for working C compiler: /home/buzz/.espressif/tools/xtensa-esp32s3-elf/esp-2021r2-patch3-8.4.0/xtensa-esp32s3-elf/bin/xtensa-esp32s3-elf-gcc - skipped
-- Detecting C compile features
-- Detecting C compile features - done
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Check for working CXX compiler: /home/buzz/.espressif/tools/xtensa-esp32s3-elf/esp-2021r2-patch3-8.4.0/xtensa-esp32s3-elf/bin/xtensa-esp32s3-elf-g++ - skipped
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- Found Git: /usr/bin/git (found version "2.25.1") 
-- Checking Python dependencies...
Python requirements from /home/buzz/ardupilot/modules/esp_idf/requirements.txt are satisfied.
-- Project sdkconfig file /home/buzz/ardupilot/libraries/AP_HAL_ESP32/targets/esp32s3/esp-idf/sdkconfig
-- The ASM compiler identification is GNU
-- Found assembler: /home/buzz/.espressif/tools/xtensa-esp32s3-elf/esp-2021r2-patch3-8.4.0/xtensa-esp32s3-elf/bin/xtensa-esp32s3-elf-gcc
-- Found Python3: /home/buzz/.espressif/python_env/idf4.4_py3.8_env/bin/python3.8 (found version "3.8.10") found components: Interpreter 
-- Performing Test C_COMPILER_SUPPORTS_WFORMAT_SIGNEDNESS
-- Performing Test C_COMPILER_SUPPORTS_WFORMAT_SIGNEDNESS - Success
-- App "ardupilot" version: 1
-- Adding linker script /home/buzz/ardupilot/build/esp32s3buzz_periph/esp-idf_build/esp-idf/esp_system/ld/memory.ld
-- Adding linker script /home/buzz/ardupilot/modules/esp_idf/components/esp_system/ld/esp32s3/sections.ld.in
-- Adding linker script /home/buzz/ardupilot/modules/esp_idf/components/esp_rom/esp32s3/ld/esp32s3.rom.ld
-- Adding linker script /home/buzz/ardupilot/modules/esp_idf/components/esp_rom/esp32s3/ld/esp32s3.rom.api.ld
-- Adding linker script /home/buzz/ardupilot/modules/esp_idf/components/esp_rom/esp32s3/ld/esp32s3.rom.libgcc.ld
-- Adding linker script /home/buzz/ardupilot/modules/esp_idf/components/esp_rom/esp32s3/ld/esp32s3.rom.newlib.ld
-- Adding linker script /home/buzz/ardupilot/modules/esp_idf/components/esp_rom/esp32s3/ld/esp32s3.rom.version.ld
-- Adding linker script /home/buzz/ardupilot/modules/esp_idf/components/esp_rom/esp32s3/ld/esp32s3.rom.newlib-time.ld
-- Adding linker script /home/buzz/ardupilot/modules/esp_idf/components/soc/esp32s3/ld/esp32s3.peripherals.ld
-- Configuring done
-- Generating done
-- Build files have been written to: /home/buzz/ardupilot/build/esp32s3buzz_periph/esp-idf_build
Building for AP_Periph
CMake Warning:
  Manually-specified variables were not used by the project:

    ARDUPILOT_BIN



[2/3] CMake Build esp-idf showinc
BUZZ BLD CMD: ['/usr/bin/cmake', '--build', '/home/buzz/ardupilot/build/esp32s3buzz_periph/esp-idf_build', '--target', 'showinc']
[1/1] cd /home/buzz/ardupilot/build/esp32s3buzz_periph/esp-idf_b...ot/build/esp32s3buzz_periph/esp-idf_build/config > includes.list
[3/3] Processing build/esp32s3buzz_periph/esp-idf_build/includes.list
[6/9] Compiling libraries/AP_Scripting/generator/src/main.c
[7/9] Processing modules/mavlink/message_definitions/v1.0/ardupilotmega.xml
[8/9] Creating build/esp32s3buzz_periph/ap_version.h
Validation skipped for /home/buzz/ardupilot/modules/mavlink/message_definitions/v1.0/ardupilotmega.xml.
Parsing /home/buzz/ardupilot/modules/mavlink/message_definitions/v1.0/ardupilotmega.xml
Validation skipped for /home/buzz/ardupilot/modules/mavlink/message_definitions/v1.0/common.xml.
Parsing /home/buzz/ardupilot/modules/mavlink/message_definitions/v1.0/common.xml
Validation skipped for /home/buzz/ardupilot/modules/mavlink/message_definitions/v1.0/uAvionix.xml.
Parsing /home/buzz/ardupilot/modules/mavlink/message_definitions/v1.0/uAvionix.xml
Validation skipped for /home/buzz/ardupilot/modules/mavlink/message_definitions/v1.0/icarous.xml.
Parsing /home/buzz/ardupilot/modules/mavlink/message_definitions/v1.0/icarous.xml
Validation skipped for /home/buzz/ardupilot/modules/mavlink/message_definitions/v1.0/minimal.xml.
Parsing /home/buzz/ardupilot/modules/mavlink/message_definitions/v1.0/minimal.xml
Merged enum MAV_CMD
Found 254 MAVLink message types in 5 XML files
Generating C implementation in directory /home/buzz/ardupilot/build/esp32s3buzz_periph/libraries/GCS_MAVLink/include/mavlink/v2.0/ardupilotmega
[9/9] Processing /home/buzz/ardupilot/build/esp32s3buzz_periph/libraries/AP_Scripting/lua_generated_bindings.cpp,/home/buzz/ardupilot/build/esp32s3buzz_periph/libraries/AP_Scripting/lua_generated_bindings.h: libraries/AP_Scripting/generator/description/bindings.desc build/esp32s3buzz_periph/gen-bindings -> build/esp32s3buzz_periph/libraries/AP_Scripting/lua_generated_bindings.cpp build/esp32s3buzz_periph/libraries/AP_Scripting/lua_generated_bindings.h
Generating C implementation in directory /home/buzz/ardupilot/build/esp32s3buzz_periph/libraries/GCS_MAVLink/include/mavlink/v2.0/common
Generating C implementation in directory /home/buzz/ardupilot/build/esp32s3buzz_periph/libraries/GCS_MAVLink/include/mavlink/v2.0/uAvionix
Generating C implementation in directory /home/buzz/ardupilot/build/esp32s3buzz_periph/libraries/GCS_MAVLink/include/mavlink/v2.0/icarous
Generating C implementation in directory /home/buzz/ardupilot/build/esp32s3buzz_periph/libraries/GCS_MAVLink/include/mavlink/v2.0/minimal
Copying fixed headers for protocol 2.0 to /home/buzz/ardupilot/build/esp32s3buzz_periph/libraries/GCS_MAVLink/include/mavlink/v2.0
BUZZ periph: /home/buzz/ardupilot/build/esp32s3buzz_periph/lib/bin/libAP_Periph.a
[ 10/477] Compiling libraries/AC_PID/AC_PI_2D.cpp
[ 11/477] Compiling libraries/AC_PID/AC_HELI_PID.cpp
[ 12/477] Compiling libraries/AC_PID/AC_P.cpp
[ 13/477] Compiling libraries/AC_PID/AC_P_2D.cpp
[ 14/477] Compiling libraries/AC_PID/AC_PID_2D.cpp
[ 15/477] Compiling libraries/AC_PID/AC_P_1D.cpp
[ 16/477] Compiling libraries/AC_PID/AC_PID.cpp
[ 17/477] Compiling libraries/AC_PID/AC_PI.cpp
[ 18/477] Compiling libraries/AC_PID/AC_PID_Basic.cpp
[ 19/477] Compiling libraries/AP_AccelCal/AccelCalibrator.cpp
[ 20/477] Compiling libraries/AP_AccelCal/AP_AccelCal.cpp
[ 21/477] Compiling libraries/AP_Airspeed/AP_Airspeed_Backend.cpp
[ 22/477] Compiling libraries/AP_Airspeed/AP_Airspeed_MS4525.cpp
[ 23/477] Compiling libraries/AP_Airspeed/AP_Airspeed_MS5525.cpp
[ 24/477] Compiling libraries/AP_Airspeed/AP_Airspeed_DLVR.cpp
[ 25/477] Compiling libraries/AP_Airspeed/AP_Airspeed_MSP.cpp
[ 26/477] Compiling libraries/AP_Airspeed/AP_Airspeed_SDP3X.cpp
[ 27/477] Compiling libraries/AP_Airspeed/AP_Airspeed_Health.cpp
[ 28/477] Compiling libraries/AP_Airspeed/AP_Airspeed_UAVCAN.cpp
[ 29/477] Compiling libraries/AP_Airspeed/AP_Airspeed_analog.cpp
[ 30/477] Compiling libraries/AP_Airspeed/Airspeed_Calibration.cpp
[ 31/477] Compiling libraries/AP_Airspeed/AP_Airspeed_ASP5033.cpp
[ 32/477] Compiling libraries/AP_Baro/AP_Baro_LPS2XH.cpp
[ 33/477] Compiling libraries/AP_Baro/AP_Baro_ICM20789.cpp
[ 34/477] Compiling libraries/AP_Baro/AP_Baro_Logging.cpp
[ 35/477] Compiling libraries/AP_Baro/AP_Baro_MS5611.cpp
[ 36/477] Compiling libraries/AP_Baro/AP_Baro_MSP.cpp
[ 37/477] Compiling libraries/AP_Baro/AP_Baro_SPL06.cpp
[ 38/477] Compiling libraries/AP_Baro/AP_Baro_UAVCAN.cpp
[ 39/477] Compiling libraries/AP_Baro/AP_Baro_Wind.cpp
[ 40/477] Compiling libraries/AP_Baro/AP_Baro_BMP085.cpp
[ 41/477] Compiling libraries/AP_Baro/AP_Baro_BMP280.cpp
[ 42/477] Compiling libraries/AP_Baro/AP_Baro_BMP388.cpp
[ 43/477] Compiling libraries/AP_Baro/AP_Baro_Backend.cpp
[ 44/477] Compiling libraries/AP_Baro/AP_Baro_DPS280.cpp
[ 45/477] Compiling libraries/AP_Baro/AP_Baro_Dummy.cpp
[ 46/477] Compiling libraries/AP_Baro/AP_Baro_ExternalAHRS.cpp
[ 47/477] Compiling libraries/AP_Baro/AP_Baro_FBM320.cpp
[ 48/477] Compiling libraries/AP_Baro/AP_Baro_HIL.cpp
[ 49/477] Compiling libraries/AP_Baro/AP_Baro_KellerLD.cpp
[ 50/477] Compiling libraries/AP_BattMonitor/AP_BattMonitor_SMBus_Solo.cpp
[ 51/477] Compiling libraries/AP_BattMonitor/AP_BattMonitor_FuelLevel_PWM.cpp
[ 52/477] Compiling libraries/AP_BattMonitor/AP_BattMonitor_Bebop.cpp
[ 53/477] Compiling libraries/AP_BattMonitor/AP_BattMonitor_SMBus_Generic.cpp
[ 54/477] Compiling libraries/AP_BattMonitor/AP_BattMonitor_Sum.cpp
[ 55/477] Compiling libraries/AP_BattMonitor/AP_BattMonitor_ESC.cpp
[ 56/477] Compiling libraries/AP_BattMonitor/AP_BattMonitor_FuelFlow.cpp
[ 57/477] Compiling libraries/AP_BattMonitor/AP_BattMonitor_SMBus_SUI.cpp
[ 58/477] Compiling libraries/AP_BattMonitor/AP_BattMonitor_Backend.cpp
[ 59/477] Compiling libraries/AP_BattMonitor/AP_BattMonitor_SMBus_NeoDesign.cpp
[ 60/477] Compiling libraries/AP_BattMonitor/AP_BattMonitor_SMBus_Rotoye.cpp
[ 61/477] Compiling libraries/AP_BattMonitor/AP_BattMonitor_Analog.cpp
[ 62/477] Compiling libraries/AP_BattMonitor/AP_BattMonitor_Torqeedo.cpp
[ 63/477] Compiling libraries/AP_BattMonitor/AP_BattMonitor_INA2xx.cpp
[ 64/477] Compiling libraries/AP_BattMonitor/AP_BattMonitor_Logging.cpp
[ 65/477] Compiling libraries/AP_BattMonitor/AP_BattMonitor_SMBus.cpp
[ 66/477] Compiling libraries/AP_BattMonitor/AP_BattMonitor_UAVCAN.cpp
[ 67/477] Compiling libraries/AP_BattMonitor/AP_BattMonitor_Generator.cpp
[ 68/477] Compiling libraries/AP_BattMonitor/AP_BattMonitor_LTC2946.cpp
[ 69/477] Compiling libraries/AP_BoardConfig/IMU_heater.cpp
[ 70/477] Compiling libraries/AP_BoardConfig/board_drivers.cpp
[ 71/477] Compiling libraries/AP_CANManager/AP_CANSensor.cpp
[ 72/477] Compiling libraries/AP_CANManager/AP_CANTester_KDECAN.cpp
[ 73/477] Compiling libraries/AP_CANManager/AP_SLCANIface.cpp
[ 74/477] Compiling libraries/AP_CANManager/AP_CANIfaceParams.cpp
[ 75/477] Compiling libraries/AP_CANManager/AP_CANTester.cpp
[ 76/477] Compiling libraries/AP_Common/AP_Common.cpp
[ 77/477] Compiling libraries/AP_Common/AP_ExpandingArray.cpp
[ 78/477] Compiling libraries/AP_Common/AP_FWVersion.cpp
[ 79/477] Compiling libraries/AP_Common/ExpandingString.cpp
[ 80/477] Compiling libraries/AP_Common/Location.cpp
[ 81/477] Compiling libraries/AP_Common/c++.cpp
[ 82/477] Compiling libraries/AP_Common/NMEA.cpp
[ 83/477] Compiling libraries/AP_Compass/AP_Compass_UAVCAN.cpp
[ 84/477] Compiling libraries/AP_Compass/AP_Compass_AK8963.cpp
[ 85/477] Compiling libraries/AP_Compass/AP_Compass_ExternalAHRS.cpp
[ 86/477] Compiling libraries/AP_Compass/Compass_learn.cpp
[ 87/477] Compiling libraries/AP_Compass/CompassCalibrator.cpp
[ 88/477] Compiling libraries/AP_Compass/Compass_PerMotor.cpp
[ 89/477] Compiling libraries/AP_Compass/AP_Compass_RM3100.cpp
[ 90/477] Compiling libraries/AP_Compass/AP_Compass_MMC5xx3.cpp
[ 91/477] Compiling libraries/AP_Compass/AP_Compass_Calibration.cpp
[ 92/477] Compiling libraries/AP_Compass/AP_Compass_SITL.cpp
[ 93/477] Compiling libraries/AP_Compass/AP_Compass_MSP.cpp
[ 94/477] Compiling libraries/AP_Compass/AP_Compass_MMC3416.cpp
[ 95/477] Compiling libraries/AP_Compass/AP_Compass_QMC5883L.cpp
[ 96/477] Compiling libraries/AP_Compass/AP_Compass_MAG3110.cpp
[ 97/477] Compiling libraries/AP_Compass/AP_Compass_AK09916.cpp
[ 98/477] Compiling libraries/AP_Compass/AP_Compass_LSM303D.cpp
[ 99/477] Compiling libraries/AP_Compass/AP_Compass_LSM9DS1.cpp
[100/477] Compiling libraries/AP_Compass/AP_Compass_BMM150.cpp
[101/477] Compiling libraries/AP_Compass/AP_Compass_HMC5843.cpp
[102/477] Compiling libraries/AP_Compass/AP_Compass_LIS3MDL.cpp
[103/477] Compiling libraries/AP_Compass/AP_Compass_IST8308.cpp
[104/477] Compiling libraries/AP_Compass/AP_Compass_IST8310.cpp
[105/477] Compiling libraries/AP_ESC_Telem/AP_ESC_Telem.cpp
[106/477] Compiling libraries/AP_ESC_Telem/AP_ESC_Telem_Backend.cpp
[107/477] Compiling libraries/AP_ESC_Telem/AP_ESC_Telem_SITL.cpp
[108/477] Compiling libraries/AP_Filesystem/AP_Filesystem_FATFS.cpp
[109/477] Compiling libraries/AP_Filesystem/posix_compat.cpp
[110/477] Compiling libraries/AP_Filesystem/AP_Filesystem_ESP32.cpp
[111/477] Compiling libraries/AP_Filesystem/AP_Filesystem.cpp
[112/477] Compiling libraries/AP_Filesystem/AP_Filesystem_Mission.cpp
[113/477] Compiling libraries/AP_Filesystem/AP_Filesystem_Param.cpp
[114/477] Compiling libraries/AP_Filesystem/AP_Filesystem_ROMFS.cpp
[115/477] Compiling libraries/AP_Filesystem/AP_Filesystem_backend.cpp
[116/477] Compiling libraries/AP_Filesystem/AP_Filesystem_Sys.cpp
[117/477] Compiling libraries/AP_FlashStorage/AP_FlashStorage.cpp
[118/477] Compiling libraries/AP_GPS/RTCM3_Parser.cpp
[119/477] Compiling libraries/AP_GPS/AP_GPS_SBF.cpp
[120/477] Compiling libraries/AP_GPS/MovingBase.cpp
[121/477] Compiling libraries/AP_GPS/AP_GPS_UBLOX.cpp
[122/477] Compiling libraries/AP_GPS/AP_GPS_SBP.cpp
[123/477] Compiling libraries/AP_GPS/GPS_Backend.cpp
[124/477] Compiling libraries/AP_GPS/AP_GPS_UAVCAN.cpp
[125/477] Compiling libraries/AP_GPS/AP_GPS_SBP2.cpp
[126/477] Compiling libraries/AP_GPS/AP_GPS_SIRF.cpp
[127/477] Compiling libraries/AP_GPS/AP_GPS_NOVA.cpp
[128/477] Compiling libraries/AP_GPS/AP_GPS_GSOF.cpp
[129/477] Compiling libraries/AP_GPS/AP_GPS_NMEA.cpp
[130/477] Compiling libraries/AP_GPS/AP_GPS_MAV.cpp
[131/477] Compiling libraries/AP_GPS/AP_GPS_MSP.cpp
[132/477] Compiling libraries/AP_GPS/AP_GPS_ExternalAHRS.cpp
[133/477] Compiling libraries/AP_GPS/AP_GPS.cpp
[134/477] Compiling libraries/AP_GPS/AP_GPS_ERB.cpp
[135/477] Compiling libraries/AP_InertialSensor/AP_InertialSensor_ADIS1647x.cpp
../../libraries/AP_GPS/AP_GPS_GSOF.cpp:21: warning: "ALLOW_DOUBLE_MATH_FUNCTIONS" redefined
 #define ALLOW_DOUBLE_MATH_FUNCTIONS
 
<command-line>: note: this is the location of the previous definition

[136/477] Compiling libraries/AP_InertialSensor/AP_InertialSensor_Invensensev2.cpp
[137/477] Compiling libraries/AP_InertialSensor/AP_InertialSensor_Invensense.cpp
[138/477] Compiling libraries/AP_InertialSensor/AP_InertialSensor_NONE.cpp
[139/477] Compiling libraries/AP_InertialSensor/AP_InertialSensor_SITL.cpp
[140/477] Compiling libraries/AP_InertialSensor/AP_InertialSensor_RST.cpp
[141/477] Compiling libraries/AP_InertialSensor/AP_InertialSensor_tempcal.cpp
[142/477] Compiling libraries/AP_InertialSensor/AP_InertialSensor_ExternalAHRS.cpp
[143/477] Compiling libraries/AP_InertialSensor/AP_InertialSensor_BMI270.cpp
[144/477] Compiling libraries/AP_InertialSensor/AP_InertialSensor_Backend.cpp
[145/477] Compiling libraries/AP_InertialSensor/AP_InertialSensor_BMI160.cpp
[146/477] Compiling libraries/AP_InertialSensor/AP_InertialSensor_BMI088.cpp
[147/477] Compiling libraries/AP_InertialSensor/AP_InertialSensor_BMI055.cpp
[148/477] Compiling libraries/AP_InertialSensor/AP_InertialSensor_L3G4200D.cpp
[149/477] Compiling libraries/AP_InertialSensor/AuxiliaryBus.cpp
[150/477] Compiling libraries/AP_InertialSensor/BatchSampler.cpp
[151/477] Compiling libraries/AP_InertialSensor/AP_InertialSensor_Invensensev3.cpp
[152/477] Compiling libraries/AP_InertialSensor/AP_InertialSensor_LSM9DS0.cpp
[153/477] Compiling libraries/AP_InertialSensor/AP_InertialSensor_Logging.cpp
[154/477] Compiling libraries/AP_InertialSensor/AP_InertialSensor_LSM9DS1.cpp
[155/477] Compiling libraries/AP_InternalError/AP_InternalError.cpp
[156/477] Compiling libraries/AP_Logger/LogFile.cpp
[157/477] Compiling libraries/AP_Logger/LoggerMessageWriter.cpp
[158/477] Compiling libraries/AP_Logger/AP_Logger_Block.cpp
[159/477] Compiling libraries/AP_Logger/AP_Logger_MAVLink.cpp
[160/477] Compiling libraries/AP_Logger/AP_Logger_DataFlash.cpp
[161/477] Compiling libraries/AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp
[162/477] Compiling libraries/AP_MSP/AP_MSP_Telem_Backend.cpp
[163/477] Compiling libraries/AP_MSP/AP_MSP_Telem_DJI.cpp
[164/477] Compiling libraries/AP_MSP/AP_MSP.cpp
[165/477] Compiling libraries/AP_MSP/AP_MSP_Telem_DisplayPort.cpp
[166/477] Compiling libraries/AP_MSP/msp_sbuf.cpp
[167/477] Compiling libraries/AP_MSP/AP_MSP_Telem_Generic.cpp
[168/477] Compiling libraries/AP_MSP/msp.cpp
[169/477] Compiling libraries/AP_Math/vector2.cpp
[170/477] Compiling libraries/AP_Math/spline5.cpp
[171/477] Compiling libraries/AP_Math/vector3.cpp
[172/477] Compiling libraries/AP_Math/polygon.cpp
[173/477] Compiling libraries/AP_Math/matrix3.cpp
[174/477] Compiling libraries/AP_Math/quaternion.cpp
[175/477] Compiling libraries/AP_Math/matrix_alg.cpp
[176/477] Compiling libraries/AP_Math/matrixN.cpp
[177/477] Compiling libraries/AP_Math/polyfit.cpp
[178/477] Compiling libraries/AP_Math/location_double.cpp
[179/477] Compiling libraries/AP_Math/location.cpp
[180/477] Compiling libraries/AP_Math/AP_GeodesicGrid.cpp
[181/477] Compiling libraries/AP_Math/crc.cpp
[182/477] Compiling libraries/AP_Math/AP_Math.cpp
[183/477] Compiling libraries/AP_Math/SCurve.cpp
../../libraries/AP_Math/location_double.cpp:22: warning: "ALLOW_DOUBLE_MATH_FUNCTIONS" redefined
 #define ALLOW_DOUBLE_MATH_FUNCTIONS
 
<command-line>: note: this is the location of the previous definition

[184/477] Compiling libraries/AP_Math/control.cpp
[185/477] Compiling libraries/AP_Math/SplineCurve.cpp
[186/477] Compiling libraries/AP_Notify/ToshibaLED_I2C.cpp
[187/477] Compiling libraries/AP_Notify/RCOutputRGBLed.cpp
[188/477] Compiling libraries/AP_Notify/NCP5623.cpp
[189/477] Compiling libraries/AP_Notify/Display.cpp
[190/477] Compiling libraries/AP_Notify/VRBoard_LED.cpp
[191/477] Compiling libraries/AP_Notify/RGBLed.cpp
[192/477] Compiling libraries/AP_Notify/NavigatorLED.cpp
[193/477] Compiling libraries/AP_Notify/ProfiLED.cpp
[194/477] Compiling libraries/AP_Notify/UAVCAN_RGB_LED.cpp
[195/477] Compiling libraries/AP_Notify/Led_Sysfs.cpp
[196/477] Compiling libraries/AP_Notify/Display_SH1106_I2C.cpp
[197/477] Compiling libraries/AP_Notify/MMLPlayer.cpp
[198/477] Compiling libraries/AP_Notify/ExternalLED.cpp
[199/477] Compiling libraries/AP_Notify/Display_SITL.cpp
[200/477] Compiling libraries/AP_Notify/Display_SSD1306_I2C.cpp
[201/477] Compiling libraries/AP_Notify/ScriptingLED.cpp
[202/477] Compiling libraries/AP_Notify/OreoLED_I2C.cpp
[203/477] Compiling libraries/AP_Notify/DiscoLED.cpp
[204/477] Compiling libraries/AP_Notify/SITL_SFML_LED.cpp
[205/477] Compiling libraries/AP_Notify/SerialLED.cpp
[206/477] Compiling libraries/AP_Notify/DiscreteRGBLed.cpp
[207/477] Compiling libraries/AP_Notify/ToneAlarm.cpp
[208/477] Compiling libraries/AP_Notify/PixRacerLED.cpp
[209/477] Compiling libraries/AP_Notify/DShotLED.cpp
[210/477] Compiling libraries/AP_Notify/AP_BoardLED.cpp
[211/477] Compiling libraries/AP_Notify/NeoPixel.cpp
[212/477] Compiling libraries/AP_Notify/PCA9685LED_I2C.cpp
[213/477] Compiling libraries/AP_Notify/AP_BoardLED2.cpp
[214/477] Compiling libraries/AP_Notify/Buzzer.cpp
[215/477] Compiling libraries/AP_Notify/AP_Notify.cpp
[216/477] Compiling libraries/AP_Param/AP_Param.cpp
[217/477] Compiling libraries/AP_RAMTRON/AP_RAMTRON.cpp
[218/477] Compiling libraries/AP_ROMFS/tinflate.cpp
[219/477] Compiling libraries/AP_ROMFS/tinfgzip.cpp
[220/477] Compiling libraries/AP_ROMFS/AP_ROMFS.cpp
[221/477] Compiling libraries/AP_RTC/AP_RTC.cpp
[222/477] Compiling libraries/AP_RTC/JitterCorrection.cpp
[223/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_analog.cpp
[224/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_Benewake_CAN.cpp
[225/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_Backend.cpp
[226/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_BLPing.cpp
[227/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_BBB_PRU.cpp
[228/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_TeraRangerI2C.cpp
[229/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_Benewake.cpp
[230/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_Backend_Serial.cpp
[231/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_Bebop.cpp
[232/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_Wasp.cpp
[233/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_VL53L0X.cpp
[234/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_SITL.cpp
[235/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_VL53L1X.cpp
[236/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_USD1_Serial.cpp
[237/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_UAVCAN.cpp
[238/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_USD1_CAN.cpp
[239/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_Params.cpp
[240/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_MaxsonarI2CXL.cpp
[241/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_GYUS42v2.cpp
[242/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_Benewake_TFMiniPlus.cpp
[243/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_PulsedLightLRF.cpp
[244/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_PWM.cpp
[245/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_MaxsonarSerialLV.cpp
[246/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_NMEA.cpp
[247/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_MSP.cpp
[248/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_LightWareSerial.cpp
[249/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_HC_SR04.cpp
[250/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_MAVLink.cpp
[251/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_LightWareI2C.cpp
[252/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_Lanbao.cpp
[253/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_LeddarVu8.cpp
[254/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder_LeddarOne.cpp
[255/477] Compiling libraries/AP_SerialLED/AP_SerialLED.cpp
[256/477] Compiling libraries/AP_SerialManager/AP_SerialManager.cpp
[257/477] Compiling libraries/Filter/HarmonicNotchFilter.cpp
[258/477] Compiling libraries/Filter/SlewLimiter.cpp
[259/477] Compiling libraries/Filter/NotchFilter.cpp
[260/477] Compiling libraries/Filter/ModeFilter.cpp
[261/477] Compiling libraries/Filter/LowPassFilter2p.cpp
[262/477] Compiling libraries/Filter/LowPassFilter.cpp
[263/477] Compiling libraries/Filter/DerivativeFilter.cpp
[264/477] Compiling libraries/SRV_Channel/SRV_Channel.cpp
[265/477] Compiling libraries/SRV_Channel/SRV_Channel_aux.cpp
[266/477] Compiling libraries/AP_HAL/Scheduler.cpp
[267/477] Compiling libraries/AP_HAL/RCOutput.cpp
[268/477] Compiling libraries/AP_HAL/DSP.cpp
[269/477] Compiling libraries/AP_HAL/Semaphores.cpp
[270/477] Compiling libraries/AP_HAL/HAL.cpp
[271/477] Compiling libraries/AP_HAL/Device.cpp
[272/477] Compiling libraries/AP_HAL/GPIO.cpp
[273/477] Compiling libraries/AP_HAL/CANIface.cpp
[274/477] Compiling libraries/AP_HAL/EventHandle.cpp
[275/477] Compiling libraries/AP_HAL/utility/RingBuffer.cpp
[276/477] Compiling libraries/AP_HAL/utility/getopt_cpp.cpp
[277/477] Compiling libraries/AP_HAL/utility/sumd.cpp
[278/477] Compiling libraries/AP_HAL/utility/st24.cpp
[279/477] Compiling libraries/AP_HAL/utility/packetise.cpp
[280/477] Compiling libraries/AP_HAL/Storage.cpp
[281/477] Compiling libraries/AP_HAL/utility/utoa_invert.cpp
[282/477] Compiling libraries/AP_HAL/utility/srxl.cpp
[283/477] Compiling libraries/AP_HAL/utility/print_vprintf.cpp
[284/477] Compiling libraries/AP_HAL/utility/replace.cpp
[285/477] Compiling libraries/AP_HAL/utility/ftoa_engine.cpp
[286/477] Compiling libraries/AP_HAL/utility/Socket.cpp
[287/477] Compiling libraries/AP_HAL/Util.cpp
[288/477] Compiling libraries/AP_HAL/utility/dsm.cpp
[289/477] Compiling libraries/AP_HAL/utility/RCOutput_Tap_Linux.cpp
[290/477] Compiling libraries/AP_HAL/system.cpp
[291/477] Compiling libraries/AP_HAL/utility/RCOutput_Tap.cpp
[292/477] Compiling libraries/AP_HAL/utility/BetterStream.cpp
[293/477] Compiling libraries/AP_HAL_Empty/Semaphores.cpp
[294/477] Compiling libraries/AP_HAL_Empty/RCInput.cpp
[295/477] Compiling libraries/AP_HAL_Empty/RCOutput.cpp
[296/477] Compiling libraries/AP_HAL_Empty/Scheduler.cpp
[297/477] Compiling libraries/AP_HAL_Empty/GPIO.cpp
[298/477] Compiling libraries/AP_HAL_Empty/HAL_Empty_Class.cpp
[299/477] Compiling libraries/AP_HAL_Empty/AnalogIn.cpp
[300/477] Compiling libraries/AP_HAL_Empty/Storage.cpp
[301/477] Compiling libraries/AP_HAL_Empty/UARTDriver.cpp
[302/477] Compiling libraries/AP_HAL_ESP32/Profile.cpp
[303/477] Compiling libraries/AP_HAL_ESP32/DeviceBus.cpp
[304/477] Compiling libraries/AP_HAL_ESP32/Util.cpp
[305/477] Compiling libraries/AP_HAL_ESP32/CanIface.cpp
[306/477] Compiling libraries/AP_HAL_ESP32/AnalogIn.cpp
[307/477] Compiling libraries/AP_HAL_ESP32/SPIDevice.cpp
[308/477] Compiling libraries/AP_HAL_ESP32/i2c_sw.c
[309/477] Compiling libraries/AP_HAL_ESP32/EventSource.cpp
[310/477] Compiling libraries/AP_HAL_ESP32/system.cpp
[311/477] Compiling libraries/AP_HAL_ESP32/SoftSigReaderInt.cpp
[312/477] Compiling libraries/AP_HAL_ESP32/WiFiDriver.cpp
[313/477] Compiling libraries/AP_HAL_ESP32/Semaphores.cpp
[314/477] Compiling libraries/AP_HAL_ESP32/HAL_ESP32_Class.cpp
In file included from ../../libraries/AP_HAL_ESP32/CANIface.h:64,
                 from ../../libraries/AP_HAL_ESP32/CanIface.cpp:12:
../../modules/esp_idf/components/driver/include/driver/twai.h:36:103: warning: invalid conversion from 'int' to 'gpio_num_t' [-fpermissive]
                                                                     .intr_flags = ESP_INTR_FLAG_LEVEL1}
                                                                                                       ^
../../libraries/AP_HAL_ESP32/CanIface.cpp:59:47: note: in expansion of macro 'TWAI_GENERAL_CONFIG_DEFAULT'
 static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);
                                               ^~~~~~~~~~~~~~~~~~~~~~~~~~~
../../modules/esp_idf/components/driver/include/driver/twai.h:36:103: warning: invalid conversion from 'int' to 'gpio_num_t' [-fpermissive]
                                                                     .intr_flags = ESP_INTR_FLAG_LEVEL1}
                                                                                                       ^
../../libraries/AP_HAL_ESP32/CanIface.cpp:59:47: note: in expansion of macro 'TWAI_GENERAL_CONFIG_DEFAULT'
 static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);
                                               ^~~~~~~~~~~~~~~~~~~~~~~~~~~

[315/477] Compiling libraries/AP_HAL_ESP32/WiFiUdpDriver.cpp
[316/477] Compiling libraries/AP_HAL_ESP32/WiFiSetup.cpp
[317/477] Compiling libraries/AP_HAL_ESP32/Storage.cpp
[318/477] Compiling libraries/AP_HAL_ESP32/UARTDriver.cpp
[319/477] Compiling libraries/AP_HAL_ESP32/SoftSigReaderRMT.cpp
[320/477] Compiling libraries/AP_HAL_ESP32/Scheduler.cpp
[321/477] Compiling libraries/AP_HAL_ESP32/I2CDevice.cpp
[322/477] Compiling libraries/AP_HAL_ESP32/SdCard.cpp
[323/477] Compiling libraries/AP_HAL_ESP32/RmtSigReader.cpp
[324/477] Compiling libraries/AP_HAL_ESP32/OSD.cpp
[325/477] Compiling libraries/AP_HAL_ESP32/RCOutput.cpp
[326/477] Compiling libraries/AP_HAL_ESP32/RCInput.cpp
[327/477] Compiling libraries/AP_BoardConfig/AP_BoardConfig.cpp
[328/477] Compiling libraries/AP_BattMonitor/AP_BattMonitor.cpp
[329/477] Compiling libraries/AP_BattMonitor/AP_BattMonitor_Params.cpp
[330/477] Compiling libraries/AP_CANManager/AP_CANDriver.cpp
[331/477] Compiling libraries/AP_CANManager/AP_CANManager.cpp
[332/477] Compiling libraries/StorageManager/StorageManager.cpp
[333/477] Compiling libraries/AP_Compass/AP_Compass.cpp
[334/477] Compiling libraries/AP_Compass/AP_Compass_Backend.cpp
[335/477] Compiling libraries/AP_Baro/AP_Baro_SITL.cpp
[336/477] Compiling libraries/AP_Baro/AP_Baro.cpp
[337/477] Compiling libraries/AP_Airspeed/AP_Airspeed_NMEA.cpp
[338/477] Compiling libraries/AP_Airspeed/AP_Airspeed.cpp
[339/477] Compiling libraries/AP_RangeFinder/AP_RangeFinder.cpp
[340/477] Compiling libraries/SRV_Channel/SRV_Channels.cpp
[341/477] Compiling libraries/AP_Filesystem/AP_Filesystem_posix.cpp
[342/477] Compiling libraries/AP_InertialSensor/AP_InertialSensor.cpp
[343/477] Compiling libraries/AP_Logger/AP_Logger.cpp
[344/477] Compiling libraries/AP_Logger/AP_Logger_File.cpp
[345/477] Compiling libraries/AP_Logger/AP_Logger_Backend.cpp
[346/477] Compiling libraries/AP_BLHeli/AP_BLHeli.cpp
[347/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.tunnel.Protocol.c
[348/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.power.CircuitStatus.c
[349/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.Error.c
[350/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.GetNodeInfo_res.c
[351/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.GetInfo_res.c
[352/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.indication.RGB565.c
[353/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.actuator.ArrayCommand.c
[354/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.sensors.hygrometer.Hygrometer.c
[355/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.GetTransportStats_res.c
[356/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.param.GetSet_req.c
[357/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.AccessCommandShell_res.c
[358/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.gnss.Heading.c
[359/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.indication.SingleLightCommand.c
[360/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.ice.FuelTankStatus.c
[361/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.hardpoint.Command.c
[362/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.indication.NotifyState.c
[363/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.esc.RPMCommand.c
[364/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.esc.RawCommand.c
[365/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.BeginFirmwareUpdate_req.c
[366/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.power.BatteryInfo.c
[367/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.ahrs.MagneticFieldStrength.c
[368/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.indication.SafetyState.c
[369/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/mppt.OutputEnable_req.c
[370/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.tunnel.Call_res.c
[371/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.GetInfo_req.c
[372/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.gnss.RelPosHeading.c
[373/477] Compiling modules/DroneCAN/libcanard/canard.c
[374/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.param.ExecuteOpcode_req.c
[375/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.equipment.trafficmonitor.TrafficReport.c
[376/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.air_data.RawAirData.c
[377/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/cuav.equipment.power.CBAT.c
[378/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.indication.Button.c
[379/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.param.Value.c
[380/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.dynamic_node_id.server.Entry.c
[381/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.debug.LogMessage.c
[382/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.debug.KeyValue.c
[383/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.range_sensor.Measurement.c
[384/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.HardwareVersion.c
[385/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.safety.ArmingStatus.c
[386/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.tunnel.Broadcast.c
[387/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.navigation.GlobalNavigationSolution.c
[388/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.air_data.StaticTemperature.c
[389/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.air_data.IndicatedAirspeed.c
[390/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.ahrs.Solution.c
[391/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.Delete_req.c
[392/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.GlobalTimeSync.c
[393/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.GetDirectoryEntryInfo_res.c
[394/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.GetTransportStats_req.c
[395/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.camera_gimbal.Mode.c
[396/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.Delete_res.c
[397/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.Read_res.c
[398/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.EntryType.c
[399/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.actuator.Command.c
[400/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.Panic.c
[401/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.gnss.Fix.c
[402/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.enumeration.Begin_req.c
[403/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.CoarseOrientation.c
[404/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.gnss.ECEFPositionVelocity.c
[405/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.air_data.StaticPressure.c
[406/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.tunnel.Call_req.c
[407/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.ahrs.RawIMU.c
[408/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.ice.reciprocating.CylinderStatus.c
[409/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.enumeration.Begin_res.c
[410/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.dynamic_node_id.Allocation.c
[411/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.power.PrimaryPowerSupplyStatus.c
[412/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.air_data.TrueAirspeed.c
[413/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.gnss.RTCMStream.c
[414/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.param.ExecuteOpcode_res.c
[415/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/mppt.Stream.c
[416/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.debug.LogLevel.c
[417/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.param.Empty.c
[418/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hex.equipment.flow.Measurement.c
[419/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.BeginFirmwareUpdate_res.c
[420/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.air_data.Sideslip.c
[421/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.Read_req.c
[422/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.gnss.Status.c
[423/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.dynamic_node_id.server.RequestVote_req.c
[424/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.GetNodeInfo_req.c
[425/477] Linking build/esp32s3buzz_periph/lib/libAP_Periph_libs.a
[426/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.GetDirectoryEntryInfo_req.c
[427/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.ahrs.MagneticFieldStrength2.c
[428/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.enumeration.Indication.c
[429/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.camera_gimbal.AngularCommand.c
[430/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.gnss.MovingBaselineData.c
[431/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.param.GetSet_res.c
[432/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.camera_gimbal.Status.c
[433/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.indication.LightsCommand.c
[434/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.gnss.Fix2.c
[435/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.RestartNode_req.c
[436/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.equipment.power.BatteryInfoAux.c
[437/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.Timestamp.c
[438/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.camera_gimbal.GEOPOICommand.c
[439/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.dynamic_node_id.server.RequestVote_res.c
[440/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.ice.reciprocating.Status.c
[441/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/mppt.OutputEnable_res.c
[442/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.CANIfaceStats.c
[443/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.device.Temperature.c
[444/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.esc.Status.c
[445/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.GetDataTypeInfo_req.c
[446/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.hardpoint.Status.c
[447/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.GetDataTypeInfo_res.c
[448/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.dynamic_node_id.server.AppendEntries_req.c
[449/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.DataTypeKind.c
[450/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.Path.c
[451/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.dynamic_node_id.server.AppendEntries_res.c
[452/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.dynamic_node_id.server.Discovery.c
[453/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.indication.BeepCommand.c
[454/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.Write_res.c
[455/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.param.NumericValue.c
[456/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.air_data.AngleOfAttack.c
[457/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.actuator.Status.c
[458/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.AccessCommandShell_req.c
[459/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.RestartNode_res.c
[460/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.Write_req.c
[461/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.gnss.Auxiliary.c
[462/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.SoftwareVersion.c
[463/477] Compiling build/esp32s3buzz_periph/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.NodeStatus.c
[464/477] Compiling Tools/AP_Periph/can.cpp
[465/477] Compiling Tools/AP_Periph/Parameters.cpp
[466/477] Compiling Tools/AP_Periph/GCS_MAVLink.cpp
[467/477] Compiling Tools/AP_Periph/AP_Periph.cpp
[468/477] Compiling Tools/AP_Periph/msp.cpp
[469/477] Compiling Tools/AP_Periph/version.cpp
[470/477] Compiling Tools/AP_Periph/rc_out.cpp
[471/477] Compiling Tools/AP_Periph/adsb.cpp
[472/477] Linking build/esp32s3buzz_periph/Tools/AP_Periph/liblibcanard.a
[473/477] Compiling Tools/AP_Periph/hwing_esc.cpp
[474/477] Linking build/esp32s3buzz_periph/lib/bin/libAP_Periph.a
[475/477] tuning periph triple-libs into a single one with AR and custom target... idf-periph/AP_Periph.a
[476/477] CMake Build esp-idf all
BUZZ BLD CMD: ['/usr/bin/cmake', '--build', '/home/buzz/ardupilot/build/esp32s3buzz_periph/esp-idf_build', '--target', 'all']
[13/706] Generating ../../partition_table/partition-table.bin
Partition table binary generated. Contents:
*******************************************************************************
# ESP-IDF Partition Table
# Name, Type, SubType, Offset, Size, Flags
nvs,data,nvs,0x11000,24K,
phy_init,data,phy,0x17000,4K,
factory,app,factory,0x20000,2M,
storage,69,0,0x220000,128K,
*******************************************************************************
[307/706] Performing configure step for 'bootloader'
-- Found Git: /usr/bin/git (found version "2.25.1") 
-- The C compiler identification is GNU 8.4.0
-- The CXX compiler identification is GNU 8.4.0
-- The ASM compiler identification is GNU
-- Found assembler: /home/buzz/.espressif/tools/xtensa-esp32s3-elf/esp-2021r2-patch3-8.4.0/xtensa-esp32s3-elf/bin/xtensa-esp32s3-elf-gcc
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Check for working C compiler: /home/buzz/.espressif/tools/xtensa-esp32s3-elf/esp-2021r2-patch3-8.4.0/xtensa-esp32s3-elf/bin/xtensa-esp32s3-elf-gcc - skipped
-- Detecting C compile features
-- Detecting C compile features - done
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Check for working CXX compiler: /home/buzz/.espressif/tools/xtensa-esp32s3-elf/esp-2021r2-patch3-8.4.0/xtensa-esp32s3-elf/bin/xtensa-esp32s3-elf-g++ - skipped
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- Building ESP-IDF components for target esp32s3
-- Project sdkconfig file /home/buzz/ardupilot/libraries/AP_HAL_ESP32/targets/esp32s3/esp-idf/sdkconfig
-- Adding linker script /home/buzz/ardupilot/modules/esp_idf/components/soc/esp32s3/ld/esp32s3.peripherals.ld
-- Adding linker script /home/buzz/ardupilot/modules/esp_idf/components/esp_rom/esp32s3/ld/esp32s3.rom.ld
-- Adding linker script /home/buzz/ardupilot/modules/esp_idf/components/esp_rom/esp32s3/ld/esp32s3.rom.api.ld
-- Adding linker script /home/buzz/ardupilot/modules/esp_idf/components/esp_rom/esp32s3/ld/esp32s3.rom.libgcc.ld
-- Adding linker script /home/buzz/ardupilot/modules/esp_idf/components/esp_rom/esp32s3/ld/esp32s3.rom.newlib.ld
-- Adding linker script /home/buzz/ardupilot/modules/esp_idf/components/bootloader/subproject/main/ld/esp32s3/bootloader.ld
-- Adding linker script /home/buzz/ardupilot/modules/esp_idf/components/bootloader/subproject/main/ld/esp32s3/bootloader.rom.ld
-- Components: bootloader bootloader_support efuse esp32s3 esp_common esp_hw_support esp_rom esp_system esptool_py freertos hal log main micro-ecc newlib partition_table soc spi_flash xtensa
-- Component paths: /home/buzz/ardupilot/modules/esp_idf/components/bootloader /home/buzz/ardupilot/modules/esp_idf/components/bootloader_support /home/buzz/ardupilot/modules/esp_idf/components/efuse /home/buzz/ardupilot/modules/esp_idf/components/esp32s3 /home/buzz/ardupilot/modules/esp_idf/components/esp_common /home/buzz/ardupilot/modules/esp_idf/components/esp_hw_support /home/buzz/ardupilot/modules/esp_idf/components/esp_rom /home/buzz/ardupilot/modules/esp_idf/components/esp_system /home/buzz/ardupilot/modules/esp_idf/components/esptool_py /home/buzz/ardupilot/modules/esp_idf/components/freertos /home/buzz/ardupilot/modules/esp_idf/components/hal /home/buzz/ardupilot/modules/esp_idf/components/log /home/buzz/ardupilot/modules/esp_idf/components/bootloader/subproject/main /home/buzz/ardupilot/modules/esp_idf/components/bootloader/subproject/components/micro-ecc /home/buzz/ardupilot/modules/esp_idf/components/newlib /home/buzz/ardupilot/modules/esp_idf/components/partition_table /home/buzz/ardupilot/modules/esp_idf/components/soc /home/buzz/ardupilot/modules/esp_idf/components/spi_flash /home/buzz/ardupilot/modules/esp_idf/components/xtensa
-- Configuring done
-- Generating done
-- Build files have been written to: /home/buzz/ardupilot/build/esp32s3buzz_periph/esp-idf_build/bootloader
[401/706] Performing build step for 'bootloader'
[1/101] Generating project_elf_src_esp32s3.c
[2/101] Building C object esp-idf/soc/CMakeFiles/__idf_soc.dir/soc_include_legacy_warn.c.obj
[3/101] Building C object esp-idf/hal/CMakeFiles/__idf_hal.dir/mpu_hal.c.obj
[4/101] Building C object esp-idf/soc/CMakeFiles/__idf_soc.dir/esp32s3/dedic_gpio_periph.c.obj
[5/101] Building C object esp-idf/soc/CMakeFiles/__idf_soc.dir/lldesc.c.obj
[6/101] Building C object esp-idf/soc/CMakeFiles/__idf_soc.dir/esp32s3/gdma_periph.c.obj
[7/101] Building C object CMakeFiles/bootloader.elf.dir/project_elf_src_esp32s3.c.obj
[8/101] Building C object esp-idf/soc/CMakeFiles/__idf_soc.dir/esp32s3/adc_periph.c.obj
[9/101] Building C object esp-idf/hal/CMakeFiles/__idf_hal.dir/wdt_hal_iram.c.obj
[10/101] Building C object esp-idf/hal/CMakeFiles/__idf_hal.dir/cpu_hal.c.obj
[11/101] Building C object esp-idf/soc/CMakeFiles/__idf_soc.dir/esp32s3/gpio_periph.c.obj
[12/101] Building C object esp-idf/soc/CMakeFiles/__idf_soc.dir/esp32s3/interrupts.c.obj
[13/101] Building C object esp-idf/soc/CMakeFiles/__idf_soc.dir/esp32s3/lcd_periph.c.obj
[14/101] Building C object esp-idf/soc/CMakeFiles/__idf_soc.dir/esp32s3/i2c_periph.c.obj
[15/101] Building C object esp-idf/soc/CMakeFiles/__idf_soc.dir/esp32s3/i2s_periph.c.obj
[16/101] Building C object esp-idf/soc/CMakeFiles/__idf_soc.dir/esp32s3/rmt_periph.c.obj
[17/101] Building C object esp-idf/soc/CMakeFiles/__idf_soc.dir/esp32s3/ledc_periph.c.obj
[18/101] Building C object esp-idf/soc/CMakeFiles/__idf_soc.dir/esp32s3/pcnt_periph.c.obj
[19/101] Building C object esp-idf/soc/CMakeFiles/__idf_soc.dir/esp32s3/mcpwm_periph.c.obj
[20/101] Building C object esp-idf/soc/CMakeFiles/__idf_soc.dir/esp32s3/timer_periph.c.obj
[21/101] Building C object esp-idf/soc/CMakeFiles/__idf_soc.dir/esp32s3/rtc_io_periph.c.obj
[22/101] Building C object esp-idf/soc/CMakeFiles/__idf_soc.dir/esp32s3/sigmadelta_periph.c.obj
[23/101] Building C object esp-idf/soc/CMakeFiles/__idf_soc.dir/esp32s3/sdmmc_periph.c.obj
[24/101] Building C object esp-idf/soc/CMakeFiles/__idf_soc.dir/esp32s3/usb_phy_periph.c.obj
[25/101] Building C object esp-idf/soc/CMakeFiles/__idf_soc.dir/esp32s3/sdio_slave_periph.c.obj
[26/101] Building C object esp-idf/soc/CMakeFiles/__idf_soc.dir/esp32s3/uart_periph.c.obj
[27/101] Building C object esp-idf/soc/CMakeFiles/__idf_soc.dir/esp32s3/spi_periph.c.obj
[28/101] Building C object esp-idf/spi_flash/CMakeFiles/__idf_spi_flash.dir/esp32s3/spi_flash_rom_patch.c.obj
[29/101] Building C object esp-idf/soc/CMakeFiles/__idf_soc.dir/esp32s3/touch_sensor_periph.c.obj
[30/101] Building C object esp-idf/soc/CMakeFiles/__idf_soc.dir/esp32s3/usb_periph.c.obj
[31/101] Building C object esp-idf/bootloader_support/CMakeFiles/__idf_bootloader_support.dir/src/bootloader_clock_init.c.obj
[32/101] Building C object esp-idf/bootloader_support/CMakeFiles/__idf_bootloader_support.dir/src/bootloader_mem.c.obj
[33/101] Building C object esp-idf/bootloader_support/CMakeFiles/__idf_bootloader_support.dir/src/bootloader_random.c.obj
[34/101] Building C object esp-idf/bootloader_support/CMakeFiles/__idf_bootloader_support.dir/src/bootloader_random_esp32s3.c.obj
[35/101] Building C object esp-idf/bootloader_support/CMakeFiles/__idf_bootloader_support.dir/src/bootloader_common_loader.c.obj
[36/101] Building C object esp-idf/bootloader_support/CMakeFiles/__idf_bootloader_support.dir/src/bootloader_common.c.obj
[37/101] Building C object esp-idf/bootloader_support/CMakeFiles/__idf_bootloader_support.dir/src/flash_encrypt.c.obj
[38/101] Building C object esp-idf/bootloader_support/CMakeFiles/__idf_bootloader_support.dir/src/secure_boot.c.obj
[39/101] Building C object esp-idf/bootloader_support/CMakeFiles/__idf_bootloader_support.dir/src/bootloader_flash.c.obj
[40/101] Building C object esp-idf/bootloader_support/CMakeFiles/__idf_bootloader_support.dir/src/flash_partitions.c.obj
[41/101] Building C object esp-idf/bootloader_support/CMakeFiles/__idf_bootloader_support.dir/src/bootloader_flash_config_esp32s3.c.obj
[42/101] Building C object esp-idf/bootloader_support/CMakeFiles/__idf_bootloader_support.dir/src/esp_image_format.c.obj
[43/101] Building C object esp-idf/bootloader_support/CMakeFiles/__idf_bootloader_support.dir/src/bootloader_efuse_esp32s3.c.obj
[44/101] Building C object esp-idf/bootloader_support/CMakeFiles/__idf_bootloader_support.dir/src/bootloader_clock_loader.c.obj
[45/101] Building C object esp-idf/bootloader_support/CMakeFiles/__idf_bootloader_support.dir/src/bootloader_console_loader.c.obj
[46/101] Building C object esp-idf/bootloader_support/CMakeFiles/__idf_bootloader_support.dir/src/flash_qio_mode.c.obj
[47/101] Building C object esp-idf/bootloader_support/CMakeFiles/__idf_bootloader_support.dir/src/bootloader_init.c.obj
[48/101] Building C object esp-idf/bootloader_support/CMakeFiles/__idf_bootloader_support.dir/src/esp32s3/bootloader_soc.c.obj
[49/101] Building C object esp-idf/bootloader_support/CMakeFiles/__idf_bootloader_support.dir/src/bootloader_panic.c.obj
[50/101] Building C object esp-idf/bootloader_support/CMakeFiles/__idf_bootloader_support.dir/src/bootloader_console.c.obj
[51/101] Building C object esp-idf/bootloader_support/CMakeFiles/__idf_bootloader_support.dir/src/esp32s3/bootloader_sha.c.obj
[52/101] Building C object esp-idf/efuse/CMakeFiles/__idf_efuse.dir/esp32s3/esp_efuse_fields.c.obj
[53/101] Building C object esp-idf/bootloader_support/CMakeFiles/__idf_bootloader_support.dir/src/bootloader_utility.c.obj
[54/101] Building C object esp-idf/bootloader_support/CMakeFiles/__idf_bootloader_support.dir/src/esp32s3/bootloader_esp32s3.c.obj
[55/101] Building C object esp-idf/efuse/CMakeFiles/__idf_efuse.dir/esp32s3/esp_efuse_table.c.obj
[56/101] Building C object esp-idf/efuse/CMakeFiles/__idf_efuse.dir/src/esp_efuse_api.c.obj
[57/101] Building C object esp-idf/efuse/CMakeFiles/__idf_efuse.dir/esp32s3/esp_efuse_rtc_calib.c.obj
[58/101] Building C object esp-idf/efuse/CMakeFiles/__idf_efuse.dir/esp32s3/esp_efuse_utility.c.obj
[59/101] Building C object esp-idf/esp_system/CMakeFiles/__idf_esp_system.dir/esp_err.c.obj
[60/101] Building C object esp-idf/efuse/CMakeFiles/__idf_efuse.dir/src/esp_efuse_fields.c.obj
[61/101] Building C object esp-idf/esp_hw_support/CMakeFiles/__idf_esp_hw_support.dir/compare_set.c.obj
[62/101] Building C object esp-idf/esp_hw_support/CMakeFiles/__idf_esp_hw_support.dir/cpu_util.c.obj
[63/101] Building C object esp-idf/esp_hw_support/CMakeFiles/__idf_esp_hw_support.dir/port/esp32s3/rtc_pm.c.obj
[64/101] Building C object esp-idf/efuse/CMakeFiles/__idf_efuse.dir/src/esp_efuse_api_key_esp32xx.c.obj
[65/101] Building C object esp-idf/esp_hw_support/CMakeFiles/__idf_esp_hw_support.dir/port/esp32s3/rtc_clk_init.c.obj
[66/101] Building C object esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/eri.c.obj
[67/101] Building C object esp-idf/esp_hw_support/CMakeFiles/__idf_esp_hw_support.dir/port/esp32s3/rtc_wdt.c.obj
[68/101] Building C object esp-idf/micro-ecc/CMakeFiles/__idf_micro-ecc.dir/uECC_verify_antifault.c.obj
[69/101] Building C object esp-idf/efuse/CMakeFiles/__idf_efuse.dir/src/esp_efuse_utility.c.obj
[70/101] Building C object esp-idf/esp_hw_support/CMakeFiles/__idf_esp_hw_support.dir/port/esp32s3/chip_info.c.obj
[71/101] Building C object esp-idf/esp_hw_support/CMakeFiles/__idf_esp_hw_support.dir/port/esp32s3/rtc_time.c.obj
[72/101] Building C object esp-idf/esp_rom/CMakeFiles/__idf_esp_rom.dir/patches/esp_rom_crc.c.obj
[73/101] Building C object esp-idf/esp_rom/CMakeFiles/__idf_esp_rom.dir/patches/esp_rom_sys.c.obj
[74/101] Building C object esp-idf/xtensa/CMakeFiles/__idf_xtensa.dir/xt_trax.c.obj
[75/101] Building C object esp-idf/esp_rom/CMakeFiles/__idf_esp_rom.dir/patches/esp_rom_uart.c.obj
[76/101] Building C object esp-idf/esp_rom/CMakeFiles/__idf_esp_rom.dir/patches/esp_rom_tjpgd.c.obj
[77/101] Building C object esp-idf/esp_common/CMakeFiles/__idf_esp_common.dir/src/esp_err_to_name.c.obj
[78/101] Building ASM object esp-idf/esp_rom/CMakeFiles/__idf_esp_rom.dir/patches/esp_rom_longjmp.S.obj
[79/101] Building C object esp-idf/esp_hw_support/CMakeFiles/__idf_esp_hw_support.dir/port/esp32s3/rtc_init.c.obj
[80/101] Building C object esp-idf/esp_hw_support/CMakeFiles/__idf_esp_hw_support.dir/port/esp32s3/rtc_sleep.c.obj
[81/101] Building C object esp-idf/esp_hw_support/CMakeFiles/__idf_esp_hw_support.dir/port/esp32s3/rtc_clk.c.obj
[82/101] Building C object esp-idf/main/CMakeFiles/__idf_main.dir/bootloader_start.c.obj
[83/101] Building C object esp-idf/log/CMakeFiles/__idf_log.dir/log_noos.c.obj
[84/101] Building C object esp-idf/log/CMakeFiles/__idf_log.dir/log.c.obj
[85/101] Building C object esp-idf/log/CMakeFiles/__idf_log.dir/log_buffers.c.obj
[86/101] Linking C static library esp-idf/log/liblog.a
[87/101] Linking C static library esp-idf/esp_rom/libesp_rom.a
[88/101] Linking C static library esp-idf/esp_common/libesp_common.a
[89/101] Linking C static library esp-idf/xtensa/libxtensa.a
[90/101] Linking C static library esp-idf/esp_hw_support/libesp_hw_support.a
[91/101] Linking C static library esp-idf/esp_system/libesp_system.a
[92/101] Linking C static library esp-idf/efuse/libefuse.a
[93/101] Linking C static library esp-idf/bootloader_support/libbootloader_support.a
[94/101] Linking C static library esp-idf/spi_flash/libspi_flash.a
[95/101] Linking C static library esp-idf/micro-ecc/libmicro-ecc.a
[96/101] Linking C static library esp-idf/soc/libsoc.a
[97/101] Linking C static library esp-idf/hal/libhal.a
[98/101] Linking C static library esp-idf/main/libmain.a
[99/101] Linking C executable bootloader.elf
[100/101] Generating binary image from built executable
esptool.py v3.3.1-dev
Creating esp32s3 image...
Merged 1 ELF section
Successfully created esp32s3 image.
Generated /home/buzz/ardupilot/build/esp32s3buzz_periph/esp-idf_build/bootloader/bootloader.bin
[101/101] cd /home/buzz/ardupilot/build/esp32s3buzz_periph/esp-idf_build/bootloader/esp-idf/esptool_py && python /home/buzz/ardupilot/modules/esp_idf/components/partition_table/check_sizes.py --offset 0x10000 bootloader 0x0 /home/buzz/ardupilot/build/esp32s3buzz_periph/esp-idf_build/bootloader/bootloader.bin
Bootloader binary size 0x3560 bytes. 0xcaa0 bytes (79%) free.
[705/706] Generating binary image from built executable
esptool.py v3.3.1-dev
Creating esp32s3 image...
Merged 2 ELF sections
Successfully created esp32s3 image.
Generated /home/buzz/ardupilot/build/esp32s3buzz_periph/esp-idf_build/ardupilot.bin
[706/706] cd /home/buzz/ardupilot/build/esp32s3buzz_periph/esp-i...z/ardupilot/build/esp32s3buzz_periph/esp-idf_build/ardupilot.bin
ardupilot.bin binary size 0xd6e50 bytes. Smallest app partition is 0x200000 bytes. 0x1291b0 bytes (58%) free.
[477/477] CMake Build esp-idf flash
BUZZ BLD CMD: ['/usr/bin/cmake', '--build', '/home/buzz/ardupilot/build/esp32s3buzz_periph/esp-idf_build', '--target', 'flash']
[1/5] cd /home/buzz/ardupilot/build/esp32s3buzz_periph/esp-idf_b...z/ardupilot/build/esp32s3buzz_periph/esp-idf_build/ardupilot.bin
ardupilot.bin binary size 0xd6e50 bytes. Smallest app partition is 0x200000 bytes. 0x1291b0 bytes (58%) free.
[2/5] Performing build step for 'bootloader'
[1/1] cd /home/buzz/ardupilot/build/esp32s3buzz_periph/esp-idf_build/bootloader/esp-idf/esptool_py && python /home/buzz/ardupilot/modules/esp_idf/components/partition_table/check_sizes.py --offset 0x10000 bootloader 0x0 /home/buzz/ardupilot/build/esp32s3buzz_periph/esp-idf_build/bootloader/bootloader.bin
Bootloader binary size 0x3560 bytes. 0xcaa0 bytes (79%) free.
[2/3] cd /home/buzz/ardupilot/modules/esp_idf/component...les/esp_idf/components/esptool_py/run_serial_tool.cmake
esptool.py esp32s3 --before=default_reset --after=hard_reset write_flash --flash_mode dio --flash_freq 40m --flash_size 8MB 0x0 bootloader/bootloader.bin 0x20000 ardupilot.bin 0x10000 partition_table/partition-table.bin
esptool.py v3.3.1-dev
Found 1 serial ports
Serial port /dev/ttyACM0
Connecting...
Chip is ESP32-S3
Features: WiFi, BLE
Crystal is 40MHz
MAC: 84:f7:03:c0:88:04
Uploading stub...
Running stub...
Stub running...
Configuring flash size...
Flash will be erased from 0x00000000 to 0x00003fff...
Flash will be erased from 0x00020000 to 0x000f6fff...
Flash will be erased from 0x00010000 to 0x00010fff...
Compressed 13664 bytes to 9502...
Writing at 0x00000000... (100 %)
Wrote 13664 bytes (9502 compressed) at 0x00000000 in 0.3 seconds (effective 392.8 kbit/s)...
Hash of data verified.
Compressed 880208 bytes to 528915...
Writing at 0x00020000... (3 %)
Writing at 0x0002d825... (6 %)
Writing at 0x00038b75... (9 %)
Writing at 0x00042acd... (12 %)
Writing at 0x0004898f... (15 %)
Writing at 0x0004e47b... (18 %)
Writing at 0x000547c6... (21 %)
Writing at 0x0005b086... (24 %)
Writing at 0x00061822... (27 %)
Writing at 0x00067904... (30 %)
Writing at 0x0006d870... (33 %)
Writing at 0x000733b2... (36 %)
Writing at 0x0007907a... (39 %)
Writing at 0x0007f0e9... (42 %)
Writing at 0x000857b3... (45 %)
Writing at 0x0008c100... (48 %)
Writing at 0x00092869... (51 %)
Writing at 0x00098de6... (54 %)
Writing at 0x0009fa81... (57 %)
Writing at 0x000a5645... (60 %)
Writing at 0x000aa520... (63 %)
Writing at 0x000af694... (66 %)
Writing at 0x000b4a11... (69 %)
Writing at 0x000b9d40... (72 %)
Writing at 0x000bf574... (75 %)
Writing at 0x000c52a2... (78 %)
Writing at 0x000ca588... (81 %)
Writing at 0x000d28a2... (84 %)
Writing at 0x000db6a2... (87 %)
Writing at 0x000e1657... (90 %)
Writing at 0x000e7ce7... (93 %)
Writing at 0x000ee152... (96 %)
Writing at 0x000f48d2... (100 %)
Wrote 880208 bytes (528915 compressed) at 0x00020000 in 9.9 seconds (effective 709.3 kbit/s)...
Hash of data verified.
Compressed 3072 bytes to 118...
Writing at 0x00010000... (100 %)
Wrote 3072 bytes (118 compressed) at 0x00010000 in 0.0 seconds (effective 517.4 kbit/s)...
Hash of data verified.

Leaving...
Hard resetting via RTS pin...
Note: python /home/buzz/ardupilot/modules/esp_idf/components/esptool_py/esptool/esptool.py --chip esp32s3 will search for a serial port. To specify a port, set the ESPPORT environment variable.
Note: python /home/buzz/ardupilot/modules/esp_idf/components/esptool_py/esptool/esptool.py --chip esp32s3 will attempt to set baud rate automatically. To specify a baud rate, set the ESPBAUD environment variable.

Waf: Leaving directory `/home/buzz/ardupilot/build/esp32s3buzz_periph'

Build commands will be stored in build/esp32s3buzz_periph/compile_commands.json
'AP_Periph' finished successfully (1m51.863s)
buzz@buzz-metabox:~/ardupilot$ 
