buzz ESP32-S3 pinout 

how?
setup idf/submodule, run it, etc then..
...
source ~/ardupilot/modules/esp_idf/export.sh 
rm -rf build
./waf configure --board=esp32s3buzz --debug
./waf plane -v --upload

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

ADC1 can only used pins GPIO1 - GPIO10, so we'll put the analog-in on these at 4,8,9,10, as they don't conflict with other things in that range, 
   and that moves the servo-out to 35,36,37,16,17,18 as they are 'free' otherwise.  we don't really support ADC2 at this stage, as it can affect wifi, and the code isn't aswell tested.

key spi pins classic/othernames/s3
classic-gpio26=csb=bmp280.cs=GPIO33-S3
classic-gpio5=ncs=mpu9250.cs=GPIO21-S3
classic-gpio19=sdo/sao=miso=GPIO13-S3
classic-gpio23=sda=mosi=GPIO11-S3
classic-gpio18=scl=sclk=GPIO12-S3


----------------------------------------------------------------------------------------------
buzzs S3 has 45 GPIO PIN assignments  0-21 and 26-48:
//0.   0,3,45,46 as GPIO0, GPIO3, GPIO45 and GPIO46 are strapping pins
1 //i2c-SCL  - see pin5 for i2c pair.
2 //SDCARD-MMC-D0
//3.   0,3,45,46 as GPIO0, GPIO3, GPIO45 and GPIO46 are strapping pins//
4  //AnalogIn1  (--input only on classic, bi-dir on -S3 )
5 -//i2c-SDA - see pin1 for i2c pair.
6 //uart1-rx serial1/gps?
7 //uart1-tx serial1/gps?
8  //AnalogIn2  (--input only on classic, bi-dir on -S3 )
9  //AnalogIn3  (--input only on classic, bi-dir on -S3 )
10  //AnalogIn4  (--input only on classic, bi-dir on -S3 )
t-3
11 // SPI3 'D' / mosi  / SDO - IMU sensor/s,  other SPI devs etc. see 11,12,13,21,33.   GY-91 SDA 
12 // SPI3 'clk' / clk / SCL - IMU sensor/s,  other SPI devs etc. see 11,12,13,21,33    GY-91 SCL
13 // SPI3 'Q' / miso  / SDA - IMU sensor/s,  other SPI devs etc. see 11,12,13,21,33    GY-91 SDO/SAO
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
34 RC-INPUT, PPM ?
35  //servo-out-1
36  //servo-out-2
37  //servo-out-3
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


