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

#ifndef CONFIG_RANGE_FINDER_SOURCE
 # define CONFIG_RANGE_FINDER_SOURCE RANGE_FINDER_SOURCE_ANALOG_PINS
#endif

#if CONFIG_RANGE_FINDER_SOURCE == RANGE_FINDER_SOURCE_ANALOG_PINS
 # ifndef RANGE_FINDER_SOURCE_ANALOG_PINS
  #  define CONFIG_RANGE_FINDER_SOURCE_LONG_ANALOG_PIN 0
  #  define CONFIG_RANGE_FINDER_SOURCE_SHORT_ANALOG_PIN 1
 # endif
#else
 # warning Invalid value for CONFIG_RANGE_FINDER_SOURCE, disabling RANGE_FINDER
 # define CONFIG_RANGE_FINDER DISABLED
#endif

#ifndef CONFIG_RANGE_FINDER
 # define CONFIG_RANGE_FINDER ENABLED
#endif

#ifndef RANGE_FINDER_ALT_HEALTH_MAX
 # define RANGE_FINDER_ALT_HEALTH_MAX 3            // number of good reads that indicates a healthy RANGE_FINDER
#endif

#ifndef RANGE_FINDER_RELIABLE_DISTANCE_PCT
 # define RANGE_FINDER_RELIABLE_DISTANCE_PCT 0.70f // we trust the RANGE_FINDER out to 70% of it's maximum range
#endif

#ifndef RANGE_FINDER_GAIN_DEFAULT
 # define RANGE_FINDER_GAIN_DEFAULT 2.0            // gain for controlling how quickly RANGE_FINDER range adjusts target altitude (lower means slower reaction)
#endif

#ifndef THR_SURFACE_TRACKING_VELZ_MAX
 # define THR_SURFACE_TRACKING_VELZ_MAX 150 		// max vertical speed change while surface tracking with RANGE_FINDER
#endif