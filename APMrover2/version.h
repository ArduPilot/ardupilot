#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

#define THISFIRMWARE "ArduRover V3.4.2-dev"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 3,4,2,FIRMWARE_VERSION_TYPE_DEV

#define FW_MAJOR 3
#define FW_MINOR 4
#define FW_PATCH 2
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV
