cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_Notify
  AP_BoardLED
  AP_Notify
  Buzzer
  ExternalLED
  NavioLED
  NavioLED_I2C
  RGBLed
  ToneAlarm_Linux
  ToshibaLED
  ToshibaLED_I2C
  ToshibaLED_PX4
  VRBoard_LED
)

ADD_LIBRARY (AP_Notify STATIC ${AP_Notify})