cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( Filter
  LowPassFilter2p
  DerivativeFilter
)

ADD_LIBRARY (Filter STATIC ${Filter})