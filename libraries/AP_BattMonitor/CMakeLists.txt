cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_BattMonitor
  AP_BattMonitor
  AP_BattMonitor_Analog
  AP_BattMonitor_Backend
  AP_BattMonitor_SMBus
  AP_BattMonitor_SMBus_I2C
  AP_BattMonitor_SMBus_PX4
)

ADD_LIBRARY ( AP_BattMonitor STATIC ${AP_BattMonitor} )