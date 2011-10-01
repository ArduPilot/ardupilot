// Libraries
#include <FastSerial.h>
#include <AP_Common.h>
#include <APM_RC.h>
#include <AP_RangeFinder.h>
#include <GCS_MAVLink.h>
#include <AP_ADC.h>
#include <AP_DCM.h>
#include <AP_Compass.h>
#include <Wire.h>
#include <AP_GPS.h>
#include <AP_IMU.h>
#include <APM_BMP085.h>
#include <ModeFilter.h>
#include <APO.h>

// Vehicle Configuration
#include "CarStampede.h"

// ArduPilotOne Default Setup
#include "APO_DefaultSetup.h"

#include <WProgram.h>; int main(void) {init();setup();for(;;) loop(); return 0; }
