#pragma once

#include <AP_ELRS/AP_ELRS_config.h>
#include <AP_Radio/driver_sx1280_config.h>

#ifndef AP_PERIPH_ELRS_ENABLED
#define AP_PERIPH_ELRS_ENABLED 0
#endif

#if AP_PERIPH_ELRS_ENABLED && (!AP_ELRS_ENABLED || !AP_RADIO_SX1280_ENABLED)
#error "AP_Periph ELRS requires AP_ELRS and the SX1280 driver"
#endif
