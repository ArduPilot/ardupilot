#pragma once

#include "ap_version.h"

#define THISFIRMWARE "ArduPlane V3.8.5beta1"
#define FIRMWARE_VERSION 3,8,5,FIRMWARE_VERSION_TYPE_BETA

#ifndef GIT_VERSION
#define FIRMWARE_STRING THISFIRMWARE
#else
#define FIRMWARE_STRING THISFIRMWARE " (" GIT_VERSION ")"
#endif
