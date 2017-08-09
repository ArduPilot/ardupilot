#pragma once

#include "ap_version.h"

#define THISFIRMWARE "ArduPlane V3.8.1-dev"
#define FIRMWARE_VERSION 3,8,1,FIRMWARE_VERSION_TYPE_DEV

#ifndef GIT_VERSION
#define FIRMWARE_STRING THISFIRMWARE
#else
#define FIRMWARE_STRING THISFIRMWARE " (" GIT_VERSION ")"
#endif
