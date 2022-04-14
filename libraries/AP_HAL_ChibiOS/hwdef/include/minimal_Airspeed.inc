# include file to reduce flash by including *fewer* Airspeed drivers

define AP_AIRSPEED_BACKEND_DEFAULT_ENABLED 0
define AP_AIRSPEED_MS4525_ENABLED 1
define AP_AIRSPEED_ANALOG_ENABLED 1
define AP_AIRSPEED_MS5525_ENABLED 1
define AP_AIRSPEED_SDP3X_ENABLED 1
define AP_AIRSPEED_NMEA_ENABLED 1  # additional checks for vehicle type in .cpp

define AP_AIRSPEED_UAVCAN_ENABLED HAL_ENABLE_LIBUAVCAN_DRIVERS
