cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( GCS_Mavlink
  GCS_Common
  GCS_Logs
  GCS_MAVLink
  GCS_serial_control
  MAVLink_routing
)

ADD_LIBRARY ( GCS_Mavlink STATIC ${GCS_Mavlink} )