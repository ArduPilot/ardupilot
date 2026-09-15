#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <RC_Channel/RC_Channel_config.h>

#ifndef AP_RC_LOGIC_ENABLED
#define AP_RC_LOGIC_ENABLED AP_RC_CHANNEL_ENABLED
#endif

#ifndef AP_RC_LOGIC_NUM_TERMS
#define AP_RC_LOGIC_NUM_TERMS 12
#endif
