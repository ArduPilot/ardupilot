
# Ardupilot port to the esp32 series mcu


## Building instructions
0. Build currently tested on linux
1. Checkout this branch https://github.com/ardupilot/ardupilot/tree/master
2. Use this to install ardupilot requirements:
```
Tools/environment_install/install-prereqs-ubuntu.sh
```
or
```
Tools/environment_install/install-prereqs-arch.sh
```

3. install esp-idf python deps:

```
# from: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-setup.html
sudo apt-get install git wget flex bison gperf python-setuptools cmake ninja-build ccache libffi-dev libssl-dev dfu-util
sudo apt-get install python3 python3-pip python3-setuptools
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 10

#or
sudo pacman -S --needed gcc git make flex bison gperf python-pip cmake ninja ccache dfu-util libusb


cd ardupilot
git submodule update --init --recursive
# ensure the idf exists in modules/esp_idf (supports pre-existing submodules and/or cloning it for you)
./Tools/scripts/esp32_get_idf.sh
cd modules/esp_idf
# use it:
./install.sh
unset IDF_PATH
source ./export.sh
cd ../../..

```

4. Configure and run build:
```bash
cd ardupilot
./waf configure
./waf configure --board=esp32buzz --debug
[ or ./waf configure --board=esp32diy --debug ]
./waf plane
or
./waf copter
```

```
Do NOT use "./waf build", it's broken right now.
```

TIPS:

 - ninja: error: loading 'build.ninja': No such file or directory
   You need to run a plain './waf configure' FIRST, THEN run it with the --board type you want.

 -  we use toolchain and esp-idf from espressif , as a 'git submodule', so no need to preinstall etc.
https://docs.espressif.com/projects/esp-idf/en/latest/get-started/ -
 (note we currently use https://github.com/espressif/esp-idf/tree/release/v4.0 )


 -   if you get compile error/s to do with CONFIG... such as
in expansion of macro 'configSUPPORT_STATIC_ALLOCATION'
warning: "CONFIG_SUPPORT_STATIC_ALLOCATION" is not defined

this means your 'sdkconfig' file that the IDF relies on is perhaps a bit out of date or out of sync with your IDF.
So double check you are using the correct IDF version ( buzz's branch uses v3.3 , sh83's probably does not.. and then if you are sure:
```
cd build/esp32{BOARD}/esp-idf_build
ninja menuconfig
```
navigate to [save]  (tab,tab,tab,enter)
press [tab] then [ok] to update the sdkconfig file
'config written' press [enter] to exit this dialog
press [tab] then enter on the [exit]  box to exit the app
done.    the 'sdkconfig' file in this folder should have been updated
cd ../../../..

OR locate the 'libraries/AP_HAL_ESP32/targets/esp-idf/sdkconfig' and delete it, as it should call back to the 'sdkconfig.defaults' file if its not there.

'cd libraries/AP_HAL_ESP32/targets/esp-idf ; idf.py defconfig' is the command that updates it, but that shouldn't be needed manually, we don't think.

... try ./waf plane"

5. Recommanded way to flash the firmware :
```
ESPBAUD=921600 ./waf plane --upload
```

6. The espressif esp-idf original project is built at `cd build/esp32{BOARD}/esp-idf_build/`.
You can use your default build system (make or ninja) to build other esp-idf target.

For example :
- ninja flash
- ninja monitor

If you want to specify the port, specify before any command:
```
ESPTOOL_PORT=/dev/ttyUSB1
```

If you want to specify a wanted baudrate :
```
ESPTOOL_BAUD=57600
ESPTOOL_BAUD=921600
```

You can find more info here : [ESPTOOL](https://github.com/espressif/esptool)

You can also find the cmake esp-idf project at `libraries/AP_HAL_ESP32/targets/esp-idf` for idf.py command. But see next section to understand how ardupilot is compiled on ESP32.


For flashing from another machine you need the following files:
```
build/<board>/esp-idf_build/bootloader/bootloader.bin 
build/<board>/esp-idf_build/ardupilot.bin
build/<board>/esp-idf_build/partition_table/partition-table.bin 
```
see build/<board>/esp-idf_build/flash_project_args (after building) for hints on what arguments to use

---
OLD

Alternatively, the "./waf plane' build outputs a python command that y can cut-n-paste to flash... buzz found that but using that command with a slower baudrate of 921600 instead of its recommended 2000000 worked for him:
cd ardupilot
python ./modules/esp_idf/components/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect 0xf000 ./build/esp32buzz/idf-plane/ota_data_initial.bin 0x1000 ./build/esp32buzz/idf-plane/bootloader/bootloader.bin 0x20000 ./build/esp32buzz/idf-plane/arduplane.bin 0x8000 ./build/esp32buzz/idf-plane/partitions.bin
---

## How is compiled Ardupilot on esp32

The ESP-IDF os is compiled as a library to the project.

First waf configure the esp-idf cmake and run a target to get all esp-idf includes.

Waf compile all ardupilot parts in libardusub.a and libArduSub_libs.a (for example for subs, but similarly for planes, copter, rover).

Then waf execute the esp-idf cmake to compile esp-idf itself.
And the cmake link the final ardupilot.elf against previous built libraries.

The ardupilot.elf contains all symbol. The cmake provide a stripped version as ardupilot.bin.


## Test hardware
Currently esp32 dev board with connected gy-91 10dof sensor board is supported. Pinout (consult UARTDriver.cpp and SPIDevice.cpp for reference):

### Uart connecion/s
Internally connected on most devboards, just for reference.

After flashing the esp32 , u can connect with a terminal app of your preference to the same COM port  ( eg /dev/ttyUSB0) at a baud rate of 115200, software flow control, 8N1 common uart settings, and get to see the output from hal.console->printf("...") and other console messages.

### Console/usb/boot-messages/mavlink telem aka serial0/uart0:

| ESP32 | CP2102 |
| ---   | ---    |
| GPIO3 | UART_TX |  AKA UART0_RX |
| GPIO1 | UART_RX |  AKA UART0_TX |

### GPS aka serial1/uart1:

| ESP32       | GPS       |
| ---         | ---       |
| GPIO17 (TX) | GPS (RX)  |
| GPIO16 (RX) | GPS (TX)  |
| GND         |      GND  |
| 5v          |      Pwr  |

### RC receiver connection:

|ESP32| RC Receiver |
| --- |    ---      |
| D4  |  CPPM-out   |
| GND |       GND   |
| 5v  |       Pwr   |


###  I2C connection ( for gps with leds/compass-es/etc onboard, or digital airspeed sensorrs, etc):

| ESP32   | I2C       |
| ---     | ---       |
| GPIO12  | SCL       |
| GPIO13  | SDA       |
| GND     | GND       |
| 5v      | Pwr       |


### Compass (using i2c)
 - u need to set the ardupilot params, and connected a GPS that has at least one i2c compass on it.. tested this with a HMC5883 and/or LIS3MDL
COMPASS_ENABLE=1
COMPASS_EXTERNAL=1
COMPASS_EXTERN2=1
COMPASS_EXTERN3=1

### Analog input/s

2nd column is the ardupilot _PIN number and matches what u specify in the third column of HAL_ESP32_ADC_PINS #define elsewhere :

if HAL_ESP32_ADC_PINS == HAL_ESP32_ADC_PINS_OPTION1:
| ESP32   | AnalogIn  |
| ---     | ---       |
| GPIO35  | 1         |
| GPIO34  | 2         |
| GPIO39  | 3         |
| GPIO36  | 4         |
| GND     | GND       |

eg, set ardupilot params like this:
RSSI_ANA_PIN  = 3  - and it will attempt to read the adc value on GPIO39 for rssi data
BATT_CURR_PIN = 2  - and it will attempt to read the adc value on GPIO34 for battery current
BATT_VOLT_PIN = 1  - and it will attempt to read the adc value on GPIO35 for  battery voltage
ARSPD_PIN =     4  - and it will attempt to read the adc value on GPIO36 for analog airspeed data


if HAL_ESP32_ADC_PINS == HAL_ESP32_ADC_PINS_OPTION2:
| ESP32   | AnalogIn   |
| ---     | ---        |
| GPIO35  | 35         |
| GPIO34  | 34         |
| GPIO39  | 39         |
| GPIO36  | 36         |
| GND     | GND        |

eg, set ardupilot params like this:
RSSI_ANA_PIN =  39  - and it will attempt to read the adc value on GPIO39 for rssi data
BATT_CURR_PIN = 34  - and it will attempt to read the adc value on GPIO34 for battery current
BATT_VOLT_PIN = 35  - and it will attempt to read the adc value on GPIO35 for  battery voltage
ARSPD_PIN =     36  - and it will attempt to read the adc value on GPIO36 for analog airspeed data



### RC Servo connection/s

| BuzzsPcbHeader|ESP32|  RCOUT   |TYPICAL |
|     ---       | --- |   ---    | ---    |
|  servo1       |PIN25|SERVO-OUT1|AILERON |
|  servo2       |PIN27|SERVO-OUT2|ELEVATOR|
|  servo3       |PIN33|SERVO-OUT3|THROTTLE|
|  servo4       |PIN32|SERVO-OUT4| RUDDER |
|  servo5       |PIN22|SERVO-OUT5| avail  |
|  servo6       |PIN21|SERVO-OUT6| avail  |

If you don't get any PWM output on any/some/one of the pins while ardupilot is running, be sure you have set all of these params:
//ail
SERVO1_FUNCTION = 4
// ele
SERVO2_FUNCTION = 19
//thr
SERVO3_FUNCTION = 70
//rud
SERVO4_FUNCTION = 21
// for now make it a copy of ail, in case u have two ail servos and no Y lead
SERVO5_FUNCTION = 4
// for now make it a copy of ail,for testing etc.
SERVO6_FUNCTION = 4
// right now, we only have 6 channels of output due to pin limitations..


(If the RTC source is not required, then Pin12 32K_XP and Pin13 32K_XN can be used as digital GPIOs, so we do, and it works)


### GY-91 connection
This is how buzz has the GY91 wired ATM, but its probable that connecting external 3.3V supply to the VIN is better than connecting a 5V supply, and then the 3V3 pin on the sensor board can be left disconnected, as it's actually 3.3v out from the LDO onboard.

|ESP32|GY-91|
|---|---|
|GND|GND|
|5V|VIN|
|3.3V|3V3|
|IO5|NCS|
|IO23|SDA|
|IO19|SDO/SAO|
|IO18|SCL|
|IO26|CSB|

## debugger connection
Currently used debugger is called a 'TIAO USB Multi Protocol Adapter' which is a red PCB with a bunch of jtag headers on it and doesn't cost too much. https://www.amazon.com/TIAO-Multi-Protocol-Adapter-JTAG-Serial/dp/B0156ML5LY

|ESP32| 20PINJTAG|
| --- | --- |
|D12  | TDI(PIN5)|
|D13  | SWCLK/TCLK(PIN9)|
|D14  | SWDIO/TMS(PIN7)|
|D15  | SWO/TDO(PIN13)|
|3.3v | -- ( powered via usb, not programmer, or PIN1)|
|GND  | GND(any of PIN4,PIN6,or PIN8 , all GND)|
|EN   | TRST(PIN3)|

## SDCARD connection

|ESP32|  SDCARD  |
| --- |     ---  |
|D2   | D0/PIN7  |
|D14  | CLK/PIN5 |
|D15  | CMD/PIN2 |
|GND  | Vss1/PIN3 and Vss2/PIN6 |
|3.3v | Vcc/PIN4 |

## Current progress
### Main tasks
- [x] Build system
- [x] Scheduler and semaphores
- [x] SPI driver
- [x] WiFi driver ( connect with MissionPlanner or mavproxy on TCP to host: 192.168.4.1 and port: 5760 )
- [x] Uart driver ( non-mavlink console messages and ardupilot Serial0 with mavlink or gps )
- [X] RCIN driver ( PPMSUM INPUT on GPIO4 )
- [X] GPS testing/integration ( Serial ublox GPS, as ardupilot SERIAL0 on RX2/TX2 aka GPIO16 and GPIO17 )
- [X] PWM output driver
- [x] RCOUT driver ( 4 channels working right now)
- [x] I2C driver
- [x] Storage
- [X] OTA update of the fw
- [X] SdCard
- [x] Mavlink on console/usb as well as wifi.
- [x] UDP mavlink over wifi
- [x] TCP mavlink over wifi (choose tcp or udp at compile time , not both)
- [x] parameter storage in a esp32 flash partition area
- [x] Custom boards build
- [x] Perfomance optimization
- [x] SdCard mounts but ardupilot logging to SD does not
- [x] waf can upload to your board with --upload now
- [X] PWM output driver works, but it appears that throttle supression when disarmed does not.
- [X] AnalogIn driver - partial progress not really tested or documented
- [X] Finish waf commands to build seamlessly and wrap all function of esp-idf without command voodoo

### Known Bugs

- [ ] slow but functional wifi implementation for tcp & udp. 

### Future development
- [ ] Automatic board header listing
- [ ] UDP mavlink over wifi does not automatically stream to client/s when they connect to wifi AP.
- [ ] Fix parameters loading in wifi both udp and tcp (slow and not reliable)
- [ ] Pin remapping via parameters
- [ ] GPIO driver
- [ ] DShot driver
- [ ] 4way pass
- [ ] esc telemetry
- [ ] ws2812b ws2815 led strip
- [ ] INA219 driver
- [ ] GSD
- [ ] Buzzer


### analysing a 'coredump' from the uart...

Save the log where coredump appears to a file, i'll call it core.txt
================= CORE DUMP START =================
<body of base64-encoded core dump, save it to file on disk>
================= CORE DUMP END ===================
cat > core.txt
...
ctrl-d
The CORE DUMP START and CORE DUMP END lines must not be included in core dump text file.

cp build/esp32buzz/idf-plane/arduplane.elf .
espcoredump.py dbg_corefile arduplane.elf -c core.txt -t b64

.. this will give u a 'gdb' of the core dump, from which you can type..
bt
..short for 'backtrace of current thread' or ..
thread apply all bt full
..short for 'backtrace of all threads' or ..
info var
.. which lists "All global and static variable names".
info locals
.. lists "All global and static variable names"..
 info args
..to list "Arguments of the current stack frame" (names and values)..

other things to sue..
print varname
ptype varname
select-frame 5

ctrl-c to exit gdb

# storage tips - not generally needed, as u can update params with missionplanenner over mavlink etc.


# determine offset and size of 'storage' partition in flash
parttool.py --partition-table-file partitions.csv get_partition_info --partition-name storage
>0x3e0000 0x20000

# then backup ardupilot 'storage' area (its a partition, see partitions.csv) to a file on disk:
esptool.py read_flash 0x3e0000 0x20000 storage.bin

# restore the storage.bin to your device... basiclly the same flash command as used in esp32.py but different offset and file:
esptool.py --chip esp32 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect 0x3e0000 storage.bin


### example log of boot messages:

[22:51:20:226] ets Jun  8 2016 00:22:57
[22:51:20:228]
[22:51:20:228] rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
[22:51:20:234] configsip: 0, SPIWP:0xee
[22:51:20:236] clk_drv:0x00,q_drv:0x00,d_drv:0x00,cs0_drv:0x00,hd_drv:0x00,wp_drv:0x00
[22:51:20:242] mode:DIO, clock div:1
[22:51:20:244] load:0x3fff0018,len:4
[22:51:20:248] load:0x3fff001c,len:6784
[22:51:20:248] ho 0 tail 12 room 4
[22:51:20:252] load:0x40078000,len:12116
[22:51:20:252] load:0x40080400,len:7420
[22:51:20:255] entry 0x40080784
[22:51:20:260] <0x1b>[0;32mI (30) boot: ESP-IDF v3.3-beta2-152-geed94b87e 2nd stage bootloader<0x1b>[0m
[22:51:20:267] <0x1b>[0;32mI (30) boot: compile time 21:12:33<0x1b>[0m
[22:51:20:269] <0x1b>[0;32mI (40) boot: Enabling RNG early entropy source...<0x1b>[0m
[22:51:20:275] <0x1b>[0;32mI (40) qio_mode: Enabling QIO for flash chip WinBond<0x1b>[0m
[22:51:20:281] <0x1b>[0;32mI (41) boot: SPI Speed      : 80MHz<0x1b>[0m
[22:51:20:286] <0x1b>[0;32mI (46) boot: SPI Mode       : QIO<0x1b>[0m
[22:51:20:290] <0x1b>[0;32mI (50) boot: SPI Flash Size : 4MB<0x1b>[0m
[22:51:20:294] <0x1b>[0;32mI (54) boot: Partition Table:<0x1b>[0m
[22:51:20:297] <0x1b>[0;32mI (57) boot: ## Label            Usage          Type ST Offset   Length<0x1b>[0m
[22:51:20:303] <0x1b>[0;32mI (65) boot:  0 nvs              WiFi data        01 02 00009000 00006000<0x1b>[0m
[22:51:20:309] <0x1b>[0;32mI (72) boot:  1 phy_init         RF data          01 01 0000f000 00001000<0x1b>[0m
[22:51:20:317] <0x1b>[0;32mI (79) boot:  2 factory          factory app      00 00 00010000 00300000<0x1b>[0m
[22:51:20:324] <0x1b>[0;32mI (87) boot:  3 storage          unknown          45 00 00310000 00040000<0x1b>[0m
[22:51:20:334] <0x1b>[0;32mI (94) boot:  4 coredump         Unknown data     01 03 00350000 00010000<0x1b>[0m
[22:51:20:339] <0x1b>[0;32mI (102) boot: End of partition table<0x1b>[0m
[22:51:20:343] <0x1b>[0;32mI (106) esp_image: segment 0: paddr=0x00010020 vaddr=0x3f400020 size=0x367d4 (223188) map<0x1b>[0m
[22:51:20:400] <0x1b>[0;32mI (174) esp_image: segment 1: paddr=0x000467fc vaddr=0x3ffb0000 size=0x030fc ( 12540) load<0x1b>[0m
[22:51:20:409] <0x1b>[0;32mI (178) esp_image: segment 2: paddr=0x00049900 vaddr=0x40080000 size=0x00400 (  1024) load<0x1b>[0m
[22:51:20:418] <0x1b>[0;32mI (181) esp_image: segment 3: paddr=0x00049d08 vaddr=0x40080400 size=0x06308 ( 25352) load<0x1b>[0m
[22:51:20:427] <0x1b>[0;32mI (198) esp_image: segment 4: paddr=0x00050018 vaddr=0x400d0018 size=0x128738 (1214264) map<0x1b>[0m
[22:51:20:746] <0x1b>[0;32mI (520) esp_image: segment 5: paddr=0x00178758 vaddr=0x40086708 size=0x0b3fc ( 46076) load<0x1b>[0m
[22:51:20:771] <0x1b>[0;32mI (545) boot: Loaded app from partition at offset 0x10000<0x1b>[0m
[22:51:20:778] <0x1b>[0;32mI (546) boot: Disabling RNG early entropy source...<0x1b>[0m
[22:51:20:788] <0x1b>[0;32mI (546) cpu_start: Pro cpu up.<0x1b>[0m
[22:51:20:788] <0x1b>[0;32mI (550) cpu_start: Application information:<0x1b>[0m
[22:51:20:796] <0x1b>[0;32mI (554) cpu_start: Project name:     arduplane<0x1b>[0m
[22:51:20:802] <0x1b>[0;32mI (560) cpu_start: App version:      APMrover2-3.1.0-10435-g68525424<0x1b>[0m
[22:51:20:806] <0x1b>[0;32mI (567) cpu_start: Compile time:     Apr  5 2019 22:41:05<0x1b>[0m
[22:51:20:811] <0x1b>[0;32mI (573) cpu_start: ELF file SHA256:  3fcbd70b13aca47c...<0x1b>[0m
[22:51:20:816] <0x1b>[0;32mI (579) cpu_start: ESP-IDF:          v3.3-beta2-152-geed94b87e<0x1b>[0m
[22:51:20:822] <0x1b>[0;32mI (585) cpu_start: Starting app cpu, entry point is 0x400811d4<0x1b>[0m
[22:51:20:829] <0x1b>[0;32mI (0) cpu_start: App cpu up.<0x1b>[0m
[22:51:20:833] <0x1b>[0;32mI (596) heap_init: Initializing. RAM available for dynamic allocation:<0x1b>[0m
[22:51:20:840] <0x1b>[0;32mI (602) heap_init: At 3FFAE6E0 len 00001920 (6 KiB): DRAM<0x1b>[0m
[22:51:20:846] <0x1b>[0;32mI (609) heap_init: At 3FFC1FF8 len 0001E008 (120 KiB): DRAM<0x1b>[0m
[22:51:20:852] <0x1b>[0;32mI (615) heap_init: At 3FFE0440 len 00003AE0 (14 KiB): D/IRAM<0x1b>[0m
[22:51:20:858] <0x1b>[0;32mI (621) heap_init: At 3FFE4350 len 0001BCB0 (111 KiB): D/IRAM<0x1b>[0m
[22:51:20:865] <0x1b>[0;32mI (628) heap_init: At 40091B04 len 0000E4FC (57 KiB): IRAM<0x1b>[0m
[22:51:20:871] <0x1b>[0;32mI (634) cpu_start: Pro cpu start user code<0x1b>[0m
[22:51:20:880] <0x1b>[0;32mI (206) cpu_start: Starting scheduler on PRO CPU.<0x1b>[0m
[22:51:20:885] <0x1b>[0;32mI (0) cpu_start: Starting scheduler on APP CPU.<0x1b>[0m
[22:51:20:904] I (21) wifi: wifi driver task: 3ffce8ec, prio:23, stack:3584, core=0
[22:51:20:910] I (253) wifi: wifi firmware version: 9b1a13b
[22:51:20:915] I (253) wifi: config NVS flash: disabled
[22:51:20:922] I (255) wifi: config nano formating: disabled
[22:51:20:923] <0x1b>[0;32mI (259) system_api: Base MAC address is not set, read default base MAC address from BLK0 of EFUSE<0x1b>[0m
[22:51:20:931] <0x1b>[0;32mI (269) system_api: Base MAC address is not set, read default base MAC address from BLK0 of EFUSE<0x1b>[0m
[22:51:20:941] I (279) wifi: Init dynamic tx buffer num: 16
[22:51:20:946] I (282) wifi: Init data frame dynamic rx buffer num: 16
[22:51:20:951] I (287) wifi: Init management frame dynamic rx buffer num: 16
[22:51:20:955] I (293) wifi: Init management short buffer num: 32
[22:51:20:960] I (297) wifi: Init static rx buffer size: 1600
[22:51:20:964] I (301) wifi: Init static rx buffer num: 2
[22:51:20:969] I (305) wifi: Init dynamic rx buffer num: 16
[22:51:21:701] <0x1b>[0;32mI (1049) phy: phy_version: 4100, 6fa5e27, Jan 25 2019, 17:02:06, 0, 0<0x1b>[0m
[22:51:21:710] I (1050) wifi: mode : softAP (24:0a:c4:32:48:f5)
[22:51:21:712] I (1051) wifi: Init max length of beacon: 752/752
[22:51:21:716] I (1054) wifi: Init max length of beacon: 752/752
[22:51:22:023] spi device constructed SPI:BMP280:0:2
[22:51:22:041] MS4525: no sensor found
[22:51:22:044] spi device constructed SPI:MPU9250:0:1
[22:51:23:878] Sensor failure: INS: unable to initialise driver
[22:51:26:878] Sensor failure: INS: unable to initialise driver


