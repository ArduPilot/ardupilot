###########################################################################
# Declare the library module list
###########################################################################
SET( AP_List
  AC_AttitudeControl
  AC_Fence
  AC_PID
  AC_Sprayer
  AC_WPNav
  AP_ADC
  AP_ADSB
  AP_AHRS
  AP_Airspeed
  AP_Arming
  AP_Baro
  AP_BattMonitor
  AP_BoardConfig
  AP_Camera
  AP_Common
  AP_Compass
  AP_Curve
  AP_Declination
  AP_EPM
  AP_Frsky_Telem
  AP_GPS
  AP_HAL
  AP_HAL_Empty
  AP_InertialNav
  AP_InertialSensor
  AP_L1_Control
  AP_LandingGear
  AP_Math
  AP_Menu
  AP_Mission
  AP_Motors
  AP_Mount
  AP_NavEKF
  AP_NavEKF2
  AP_Notify
  AP_OpticalFlow
  AP_Parachute
  AP_Param
  AP_PerfMon
  AP_Rally
  AP_RangeFinder
  AP_RCMapper
  AP_Relay
  AP_RPM
  AP_RSSI
  AP_Scheduler
  AP_SerialManager
  AP_ServoRelayEvents
  AP_TECS
  AP_Terrain
  APM_Control
  APM_OBC
  APM_PI
  DataFlash
  Filter
  GCS_MAVLink
  #GCS_Console # library has mistakes
  RC_Channel
  PID
  SITL
  StorageManager
)

SET( AP_List_VRBRAIN
  AP_HAL_VRBRAIN
)

SET( AP_List_SITL
  AP_HAL_SITL
)

SET( AP_List_FLYMAPLE
  AP_HAL_FLYMAPLE
)

SET( AP_List_PX4
  AP_HAL_PX4
)

SET( AP_List_Linux
  AP_HAL_Linux
)

SET( AP_Headers
  AP_Buffer
  AP_Navigation
  AP_Progmem
  AP_SpdHgtControl
  AP_Vehicle
  GCS_MAVLink
)

SET( AP_Firmwares
  AntennaTracker
  APMrover2
  ArduCopter
  ArduPlane
) 
