#pragma once

#include "ap_version.h"

#define THISFIRMWARE "APM:Rover V3.2.0-dev"
#define FIRMWARE_VERSION 3,2,0,FIRMWARE_VERSION_TYPE_DEV

#ifndef GIT_VERSION
#define FIRMWARE_STRING THISFIRMWARE
#else
#define FIRMWARE_STRING THISFIRMWARE " (" GIT_VERSION ")"
#endif
