#copter
python esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect 0xf000 copter/ota_data_initial.bin 0x1000 copter/bootloader.bin 0x20000 copter/arducopter.bin 0x8000 copter/partitions.bin

#plane
python esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect 0xf000 plane/ota_data_initial.bin 0x1000 plane/bootloader.bin 0x20000 plane/arducopter.bin 0x8000 plane/partitions.bin

