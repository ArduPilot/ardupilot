makerfabs imu module v1.1 - available on tindie
https://www.tindie.com/products/makerfabs/esp32-6-axis-imu-module/ (I'm not affiliated, and I purchased myself)

see libraries/AP_HAL_ESP32/boards/esp32imu_module_v11.h for pinout

hardware info available here:
https://github.com/ArduPilot/Schematics/tree/master/esp32/ESP32_IMU_Module_v1.1/hardware

chip erase is sometime necessary before flashing new firmware
`esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 921600 erase_flash`

