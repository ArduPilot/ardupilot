#pragma once

#include "ap_version.h"

#ifdef GIT_TAG
#define THISFIRMWARE "APM:Copter " GIT_TAG
#else
#define THISFIRMWARE "APM:Copter V3.4-dev"
#endif

#define FIRMWARE_VERSION 3,4,0,FIRMWARE_VERSION_TYPE_DEV

#ifndef GIT_VERSION
#define FIRMWARE_STRING THISFIRMWARE
#else
#define FIRMWARE_STRING THISFIRMWARE " (" GIT_VERSION ")"
#endif
