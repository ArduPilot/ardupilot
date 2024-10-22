#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"


#define FW_MAJOR 4
#define FW_MINOR 6
#define FW_PATCH 0
#define FW_TYPE_STR "-dev"
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define THISFIRMWARE "ArduCopter V" TOSTRING(FW_MAJOR) "." TOSTRING(FW_MINOR) "." TOSTRING(FW_PATCH) FW_TYPE_STR

#include <AP_Common/AP_FWVersionDefine.h>
#include <AP_CheckFirmware/AP_CheckFirmwareDefine.h>

#undef STRINGIFY
#undef TOSTRING

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION FW_MAJOR,FW_MINOR,FW_PATCH,FW_TYPE
