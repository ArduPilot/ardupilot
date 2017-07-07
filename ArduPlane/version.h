#pragma once

#include "ap_version.h"

#define THISFIRMWARE "ArduPlane V3.8.0beta6"
#define FIRMWARE_VERSION 3,8,0,FIRMWARE_VERSION_TYPE_BETA+5

#ifndef GIT_VERSION
#define FIRMWARE_STRING THISFIRMWARE
#else
#define FIRMWARE_STRING THISFIRMWARE " (" GIT_VERSION ")"
#endif
