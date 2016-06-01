#pragma once

#include "ap_version.h"

#define THISFIRMWARE "ArduRover v3.0.1beta1"
#define FIRMWARE_VERSION 3,0,1,FIRMWARE_VERSION_TYPE_BETA

#ifndef GIT_VERSION
#define FIRMWARE_STRING THISFIRMWARE
#else
#define FIRMWARE_STRING THISFIRMWARE " (" GIT_VERSION ")"
#endif
