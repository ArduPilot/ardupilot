#pragma once

/*
 this is a file intended to be included by examples and other tools
 which need to make the firmware version defines but really don't
 care how that is done.

 it is intended to be included in a .cpp file, never transitively, as
 it does create objects.
*/

#define THISFIRMWARE "GCSDummy V3.1.4-dev"

#define FW_MAJOR 3
#define FW_MINOR 1
#define FW_PATCH 4
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV

#ifndef APM_BUILD_DIRECTORY
#define APM_BUILD_DIRECTORY APM_BUILD_UNKNOWN
#endif

#define FORCE_VERSION_H_INCLUDE
#include <AP_Common/AP_FWVersionDefine.h>
#include <AP_CheckFirmware/AP_CheckFirmwareDefine.h>
#undef FORCE_VERSION_H_INCLUDE
