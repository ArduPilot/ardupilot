SD card layer

works via HAL SPI so should be at this level
Much faster than old version and supports much more formats


based on:

* Arduino's SD support for STM32F4  (fixed bugs in read() with data size, in open() with "mode==" and so on, twice reduced memory usage  and cured Arduinism)
* FatFs SDIO for STM32F1

uses FatFS (c) Chan


formatting from command-line:

mkfs.fat -I -F 12 -M 0xF9 -s 128 /dev/sd*
