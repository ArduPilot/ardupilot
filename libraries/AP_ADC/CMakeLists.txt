cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_ADC
  AP_ADC
  AP_ADC_ADS1115
  AP_ADC_HIL
)

ADD_LIBRARY ( AP_ADC STATIC ${AP_ADC} )