// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// This file is just a placeholder for your configuration file.  If
// you wish to change any of the setup parameters from their default
// values, place the appropriate #define statements here.

// If you used to define your CONFIG_APM_HARDWARE setting here, it is no
// longer valid! You should switch to using CONFIG_HAL_BOARD via the HAL_BOARD
// flag in your local config.mk instead.

// The following are the recommended settings for Xplane
// simulation. Remove the leading "/* and trailing "*/" to enable:

//#define HIL_MODE            HIL_MODE_DISABLED

/*
 *  // HIL_MODE SELECTION
 *  //
 *  // Mavlink supports
 *  // 2. HIL_MODE_SENSORS: full sensor simulation
 *
 */

//////////////////////////////////////////////////////////////////////////////
// Range Finder
// Will Baldwin
//

#ifndef CONFIG_RANGE_FINDER
 #define CONFIG_RANGE_FINDER ENABLED
#endif

#ifndef CONFIG_RANGE_FINDER_SOURCE
 #define CONFIG_RANGE_FINDER_SOURCE RANGE_FINDER_SOURCE_ANALOG_PINS
#endif

#ifndef CONFIG_RANGE_FINDER_SOURCE_ANALOG_PIN
#define CONFIG_RANGE_FINDER_SOURCE_ANALOG_PIN 2
#endif


#ifndef AP_RANGEFINDER_GP2Y0A02YK0F
#define AP_RANGEFINDER_GP2Y0A02YK0F 4
#endif


#ifndef RANGE_FINDER_ALT_HEALTH_MAX
 #define RANGE_FINDER_ALT_HEALTH_MAX 3            // number of good reads that indicates a healthy RANGE_FINDER
#endif

#ifndef RANGE_FINDER_RELIABLE_DISTANCE_PCT
 #define RANGE_FINDER_RELIABLE_DISTANCE_PCT 1.0f // we trust the RANGE_FINDER out to 70% of it's maximum range
#endif

#ifndef RANGE_FINDER_GAIN_DEFAULT
 #define RANGE_FINDER_GAIN_DEFAULT 2.0            // gain for controlling how quickly RANGE_FINDER range adjusts target altitude (lower means slower reaction)
#endif

#ifndef THR_SURFACE_TRACKING_VELZ_MAX
 #define THR_SURFACE_TRACKING_VELZ_MAX 150 		// max vertical speed change while surface tracking with RANGE_FINDER
#endif

//////////////////////////////////////////////////////////////////////////////
// Flash Sensor
// Joel Cranmer
//

#ifndef CONFIG_FLASH_SENSOR
 #define CONFIG_FLASH_SENSOR ENABLED
#endif

#ifndef CONFIG_FLASH_SENSOR_PIN
 #define CONFIG_FLASH_SENSOR_PIN 5
#endif

#ifndef FLASH_SENSOR_HEALTHY_MAX
 #define FLASH_SENSOR_HEALTHY_MAX 3		// Number of good reads that indicates a healthy FLASH_SENSOR
#endif

#ifndef FLASH_SENSOR_THRESHOLD
 #define FLASH_SENSOR_THRESHOLD 3		// threshold voltage on the pin
#endif
