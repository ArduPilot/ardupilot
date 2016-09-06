#pragma once

#include "ap_version.h"

#define THISFIRMWARE "APM:Rover v3.1.0beta3"
#define FIRMWARE_VERSION 3,1,0,FIRMWARE_VERSION_TYPE_BETA

#ifndef GIT_VERSION
#define FIRMWARE_STRING THISFIRMWARE
#else
#define FIRMWARE_STRING THISFIRMWARE " (" GIT_VERSION ")"
#endif
