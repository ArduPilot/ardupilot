


# Ardupilot port to the esp32 series mcu



## Building instructions
0. Build currently tested on linux
1. Install toolchain from espressif https://docs.espressif.com/projects/esp-idf/en/latest/get-started/
1.5 No need to get esp-idf, as it's a submodule under "buzzs" ardupilot repo. 
Please use lastest stable version of the esp-idf (https://github.com/espressif/esp-idf/tree/release/v3.3)
2. Checkout this branch https://github.com/davidbuzz/ardupilot/tree/esp32_wip
3. Use script Tools/scripts/install-prereqs-* to install ardupilot requirements
4. edit the "source_this_file.sh" to point to the checked-out ardupilot/modules/esp_idf folder.
5. Configure and run build:
```bash
cd ardupilot
source source_this_file.sh
./waf configure --board=esp32diy --debug
./waf plane
or
./waf copter

Do NOT use "./waf build", it's broken right now.


```
5. To flash binary use espressif flash tool via `make flash` inside `libraries/AP_HAL_ESP32/targets/plane/` directory. Also other make targets are avaliable (`make monitor` , `make size` and so on) 

## Test hardware
Currently esp32 dev board with connected gy-91 10dof sensor board is supported. Pinout (consult UARTDriver.cpp and SPIDevice.cpp for reference):

### Uart connecion
Internally connected on most devboards, just for reference.

After flashing the esp32 , u can connect with a terminal app of your preference to the same COM port  ( eg /dev/ttyUSB0) at a baud rate of 115200, software flow control, 8N1 common uart settings, and get to see the output from hal.console->printf("...") and other console messages. 

| ESP32 | CP2102 |
| --- | --- |
| GPIO3 | UART_TX |  AKA UART0_RX
| GPIO1 | UART_RX |  AKA UART0_TX


|ESP32|  RCOUT   |TYPICAL |
| --- |   ---    | ---    | 
|PIN33|SERVO-OUT1|AILERON |
|PIN32|SERVO-OUT2|ELEVATOR|
|PIN21|SERVO-OUT3|THROTTLE|
|PIN22|SERVO-OUT4|RUDDER  |
|PIN25|SERVO-OUT5|  ?     |
|PIN27|SERVO-OUT6|  ?     |


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
- [x] WiFi driver ( TCP mavlink over 192.168.4.1 port 5760 )
- [x] Uart driver ( non-mavlink console messages and ardupilot Serial0 with mavlink or gps )
- [X] RCIN driver ( PPMSUM INPUT on GPIO4 )
- [X] GPS testing/integration ( Serial ublox GPS, as ardupilot SERIAL0 on RX2/TX2 aka GPIO16 and GPIO17 )
- [ ] PWM driver
- [ ] RCOUT driver
- [x] I2C driver
- [x] Storage
- [X] OTA update of the fw
- [X] SdCard
- [ ] Buzzer

- [x] Custom boards build
- [x] Perfomance optimization

### Future development
- [ ] Pin remapping via parameters
- [ ] GPIO driver
- [ ] AnalogIn driver
- [ ] DShot driver / 4way pass / esc telemetry / ws2812b led
- [ ] INA219 driver
- [ ] GSD

example log of boot messages:
 
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


