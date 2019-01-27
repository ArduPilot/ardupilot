#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

#define THISFIRMWARE "ArduPlane V3.9.5"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 3,9,5,FIRMWARE_VERSION_TYPE_OFFICIAL

#define FW_MAJOR 3
#define FW_MINOR 9
#define FW_PATCH 5
#define FW_TYPE FIRMWARE_VERSION_TYPE_OFFICIAL






