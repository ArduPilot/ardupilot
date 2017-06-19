#pragma once

#include "ap_version.h"

#define THISFIRMWARE "AntennaTracker V1.0.0"
#define FIRMWARE_VERSION 1,0,0,FIRMWARE_VERSION_TYPE_BETA

#ifndef GIT_VERSION
#define FIRMWARE_STRING THISFIRMWARE
#else
#define FIRMWARE_STRING THISFIRMWARE " (" GIT_VERSION ")"
#endif
