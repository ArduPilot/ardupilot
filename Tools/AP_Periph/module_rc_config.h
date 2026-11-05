// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <AP_Module/AP_Module_config.h>
#include <AP_Radio/driver_sx1280_config.h>
#ifndef AP_PERIPH_MODULE_RC_ENABLED
#define AP_PERIPH_MODULE_RC_ENABLED 0
#endif
#if AP_PERIPH_MODULE_RC_ENABLED && (!AP_MODULE_ENABLED || !AP_MODULE_SX1280_ENABLED)
#error Module RC input requires AP_Module and the SX1280 module executor
#endif
