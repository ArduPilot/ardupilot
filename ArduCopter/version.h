#pragma once

#include "ap_version.h"

//OW
//#define THISFIRMWARE "APM:Copter V3.6-dev" //dev version of 2017-08-12
#define THISFIRMWARE "BetaCopter V3.6-dev 005"
//OWEND
#define FIRMWARE_VERSION 3,6,0,FIRMWARE_VERSION_TYPE_DEV

#ifndef GIT_VERSION
#define FIRMWARE_STRING THISFIRMWARE
#else
#define FIRMWARE_STRING THISFIRMWARE " (" GIT_VERSION ")"
#endif

/*
v0.01:
- merged in the 7 files of the PR AP_BattMonitor: UAVCAN support #6527,
  https://github.com/ArduPilot/ardupilot/pull/6527
  this seems to work!
- add GenericBatteryInfo
  dsdl definition simply added to modules\uavcan\dsdl\uavcan\equipment\power
  enable as BattMonitor_TYPE_UAVCAN_GenericBatteryInfo  = 10
 AP_UAVCAN.h: 3x
 AP_UAVCAN.cpp: 6x
 AP_BattMonitor_Backend.h: 1x
 AP_BattMonitor.h: 1x
 AP_BattMonitor.cpp: 1x
 AP_BattMonitor_UAVCAN.h: 2x
 AP_BattMonitor_UAVCAN.cpp: 2x
v0.01-03:
 - flight tested! passed!
v0.02:
 - charge handling for GenericBatteryInfo added
 - flight tested! passed! 2017-08-13
v0.03:
start into uavcan mount
 - 01: first working demo of STorM32 UAVCAN mount, emits NodeSPecific with payload 'Hey'
 - 02: first working version with BP_STorM32 integrated, emits send_attitude()
 - 03:
 - if( charge == NAN ) replaced by if (uavcan::isNaN(charge)) in AP_BattMonitor_UAVCAN.cpp
 - sending arget angles in various formats works
 - storm32.Status works

#define THISFIRMWARE "APM:Copter V3.6-dev"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 3,6,0,FIRMWARE_VERSION_TYPE_DEV

#define FW_MAJOR 3
#define FW_MINOR 6
#define FW_PATCH 0
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV
