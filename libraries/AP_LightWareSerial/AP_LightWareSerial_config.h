#pragma once

#include <AP_RangeFinder/AP_RangeFinder_config.h>
#include <AP_Proximity/AP_Proximity_config.h>

#ifndef AP_LIGHTWARESERIAL_ENABLED
#define AP_LIGHTWARESERIAL_ENABLED (AP_PROXIMITY_LIGHTWARE_SF45B_ENABLED ||  AP_RANGEFINDER_LIGHTWARE_GRF_ENABLED)
#endif
