#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

#define THISFIRMWARE "ArduRover V4.2.0-rc4"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,2,0,FIRMWARE_VERSION_TYPE_RC+4

#define FW_MAJOR 4
#define FW_MINOR 2
#define FW_PATCH 0
#define FW_TYPE FIRMWARE_VERSION_TYPE_RC

#include <AP_Common/AP_FWVersionDefine.h>
