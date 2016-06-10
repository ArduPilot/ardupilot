#pragma once

#include "ap_version.h"

#define THISFIRMWARE "AntennaTracker V0.7.8"
#define FIRMWARE_VERSION 0,7,8,FIRMWARE_VERSION_TYPE_DEV

#ifndef GIT_VERSION
#define FIRMWARE_STRING THISFIRMWARE
#else
#define FIRMWARE_STRING THISFIRMWARE " (" GIT_VERSION ")"
#endif
