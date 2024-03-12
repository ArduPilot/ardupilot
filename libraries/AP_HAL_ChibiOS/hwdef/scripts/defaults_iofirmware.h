// this file is inserted (by chibios_hwdef.py) into hwdef.h when
// configuring for iofirmware builds

#define HAL_DSHOT_ALARM_ENABLED 0

#define HAL_LOGGING_ENABLED 0

// IOMCUs *definitely* don't use the FFT library:
#ifndef HAL_GYROFFT_ENABLED
#define HAL_GYROFFT_ENABLED 0
#endif

// by default IOMCUs don't use INS:
#ifndef AP_INERTIALSENSOR_ENABLED
#define AP_INERTIALSENSOR_ENABLED 0
#endif

// no RC_Channels library:
#ifndef AP_RC_CHANNEL_ENABLED
#define AP_RC_CHANNEL_ENABLED 0
#endif

#ifndef AP_VIDEOTX_ENABLED
#define AP_VIDEOTX_ENABLED 0
#endif

// make diagnosing Faults (e.g. HardFault) harder, but save bytes:
#ifndef AP_FAULTHANDLER_DEBUG_VARIABLES_ENABLED
#define AP_FAULTHANDLER_DEBUG_VARIABLES_ENABLED 0
#endif

// disable some protocols on iomcu:
#define AP_RCPROTOCOL_FASTSBUS_ENABLED 0

// no crossfire telemetry from iomcu!
#define HAL_CRSF_TELEM_ENABLED 0

// allow the IOMCU to have its allowed protocols to be set:
#define AP_RCPROTOCOL_ENABLE_SET_RC_PROTOCOLS 1

#ifndef AP_INTERNALERROR_ENABLED
#define AP_INTERNALERROR_ENABLED 0
#endif
