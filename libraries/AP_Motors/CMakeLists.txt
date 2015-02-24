cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_Motors
  AP_Motors_Class
  AP_MotorsCoax
  AP_MotorsHeli
  AP_MotorsHexa
  AP_MotorsMatrix
  AP_MotorsOcta
  AP_MotorsOctaQuad
  AP_MotorsQuad
  AP_MotorsSingle
  AP_MotorsTri
  AP_MotorsY6
)

ADD_LIBRARY ( AP_Motors STATIC ${AP_Motors} )