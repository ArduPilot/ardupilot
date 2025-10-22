std_msgs/Header header

uint8 APM_ROVER = 1
uint8 APM_ARDUCOPTER = 2
uint8 APM_ARDUPLANE = 3
uint8 APM_ANTENNATRACKER = 4
uint8 APM_UNKNOWN = 5
uint8 APM_REPLAY = 6
uint8 APM_ARDUSUB = 7
uint8 APM_IOFIRMWARE = 8
uint8 APM_AP_PERIPH = 9
uint8 APM_AP_DAL_STANDALONE = 10
uint8 APM_AP_BOOTLOADER = 11
uint8 APM_BLIMP = 12
uint8 APM_HELI = 13
uint8 vehicle_type # From AP_Vehicle_Type.h

bool armed # true if vehicle is armed.
uint8 mode # Vehicle mode, enum depending on vehicle type.
bool flying # True if flying/driving/diving/tracking.
bool external_control # True is external control is enabled.

uint8 FS_RADIO = 21
uint8 FS_BATTERY = 22
uint8 FS_GCS = 23
uint8 FS_EKF = 24
uint8[] failsafe # Array containing all active failsafe.
