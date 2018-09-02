#pragma once

#define IS_PLANE 1  // plane functions
#define IS_COPTER 1 // copter functions
// sets.model_type plane=0 copter=1


// EEPROM Version number, incrementing this will erase/upload factory settings. Only devs should increment this
#define VER 79


#define VERSION "2.4" 

//#define DEBUG

#define SLAVE_BUILD // not separate device
#define HARDWARE_TYPE 2

#define STARTUP_SCREEN 0
//#define AUTOBAUD    // no port
//#define USE_SENSORS // no sensors

#define FONT_UPLOAD 0 // no way
#define USE_SETUP 1
#define MAVLINK_CONFIG 1
#define MAVLINK_READ_EEPROM 1
#define MAVLINK_FONT_UPLOAD 1
#define MAVLINK_PARAMS 1

#define USE_MAVLINK 1
#define MAV_REQUEST 1
#define PROTOCOL "MAVLink"


#define TELEMETRY_SPEED  57600  // initial speed of Serial port for CT

// enter to loop() at ~1700ms
#define BOOTTIME         3700   // Time in milliseconds that we show boot loading bar and wait user input


// no LED
 #define LED_BLINK {}
 #define LED_ON {}
 #define LED_OFF {}

/////////////////////////////////////////////////

#define OSD_MODEL "builtin"


#define on 1
#define off 0


#define RC_NEUTRAL 1500     // PWM pulse width for center stick

#include "eeprom.h"
#include "version.h"


#ifdef DEBUG
  #define DBG_PRINTLN(x)     { console.print_P(PSTR(x)); Serial.println();  Serial.wait();  }
  #define DBG_PRINTVARLN(x)  { console.print_P(PSTR(#x)); Serial.print_P(PSTR(": ")); Serial.println(x);  Serial.wait(); }
  #define DBG_PRINTVAR(x)    { console.print_P(PSTR(#x)); Serial.print_P(PSTR(": ")); Serial.print(x); Serial.print(" "); Serial.wait();  }
  #define DBG_PRINTF(x,...)  { console.printf_P(PSTR(x),## __VA_ARGS__); Serial.wait();  }
#else
  #define DBG_PRINTLN(x)     {}
  #define DBG_PRINTVAR(x)    {}
  #define DBG_PRINTVARLN(x)  {}
  #define DBG_PRINTF(x,...) {}
#endif

extern void osd_queue(uint8_t c);

static const uint8_t chan=0;

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#include "../osd_namespace.h"
using namespace OSDns;

