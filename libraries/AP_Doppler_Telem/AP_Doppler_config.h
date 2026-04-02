#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_Doppler_TELEM_ENABLED
#define AP_Doppler_TELEM_ENABLED 1
#endif

#ifndef AP_Doppler_D_TELEM_ENABLED
#define AP_Doppler_D_TELEM_ENABLED AP_Doppler_TELEM_ENABLED
#endif

#ifndef AP_Doppler_SPORT_TELEM_ENABLED
#define AP_Doppler_SPORT_TELEM_ENABLED AP_Doppler_TELEM_ENABLED
#endif

#ifndef AP_Doppler_SPORT_PASSTHROUGH_ENABLED
#define AP_Doppler_SPORT_PASSTHROUGH_ENABLED AP_Doppler_SPORT_TELEM_ENABLED
#endif

#ifndef HAL_WITH_Doppler_TELEM_BIDIRECTIONAL
#define HAL_WITH_Doppler_TELEM_BIDIRECTIONAL AP_Doppler_SPORT_PASSTHROUGH_ENABLED
#endif
