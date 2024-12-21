#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"
#include <AP_HAL/AP_HAL.h>

#define THISFIRMWARE "AP_Periph V1.9.0-dev"

// defines needed due to lack of GCS includes
#ifndef HAVE_ENUM_FIRMWARE_VERSION_TYPE
#define FIRMWARE_VERSION_TYPE_DEV 0
#define FIRMWARE_VERSION_TYPE_BETA 255
#define FIRMWARE_VERSION_TYPE_OFFICIAL 255
#endif

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 1,9,0,FIRMWARE_VERSION_TYPE_DEV

#define FW_MAJOR 1
#define FW_MINOR 9
#define FW_PATCH 0
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV

#include <AP_Common/AP_FWVersionDefine.h>
#include <AP_CheckFirmware/AP_CheckFirmwareDefine.h>
