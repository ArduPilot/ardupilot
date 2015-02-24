cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_HAL_AVR
  GPIO
  HAL_AVR_APM1_Class
  HAL_AVR_APM2_Class
  AP_HAL_AVR_Main
  AP_HAL_AVR_Namespace
  AP_HAL_AVR_private
  AP_HAL_AVR
  AnalogIn_ADC
  AnalogIn_Common
  I2CDriver
  memcheck
  RCInput_APM1
  RCInput_APM2
  RCOutput_APM1
  RCOutput_APM2
  Scheduler
  Scheduler_Timer
  Semaphores
  SPIDevice_SPI0
  SPIDevice_SPI2
  SPIDevice_SPI3
  SPIDeviceManager_APM1
  SPIDeviceManager_APM2
  Storage
  UARTDriver
)

ADD_LIBRARY ( AP_HAL_AVR STATIC ${AP_HAL_AVR} )