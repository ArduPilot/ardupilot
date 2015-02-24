cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_HAL_Linux
  AnalogIn
  dsm
  GPIO
  GPIO_BBB
  GPIO_RPI
  HAL_Linux_Class
  NavioAnalogIn
  RCInput
  RCInput_Navio
  RCInput_PRU
  RCInput_ZYNQ
  RCOutput_Navio
  RCOutput_PRU
  RCOutput_ZYNQ
  RCInput
  Scheduler
  Semaphores
  SPIDriver
  SPIUARTDriver
  Storage
  Storage_FRAM
  UARTDriver
  Util
)

ADD_LIBRARY ( AP_HAL_Linux STATIC ${AP_HAL_Linux} )