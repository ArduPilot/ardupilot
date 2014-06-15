// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// This file is just a placeholder for your configuration file.  If
// you wish to change any of the setup parameters from their default
// values, place the appropriate #define statements here.

// If you used to define your CONFIG_APM_HARDWARE setting here, it is no
// longer valid! You should switch to using CONFIG_HAL_BOARD via the HAL_BOARD
// flag in your local config.mk instead.

// The following are the recommended settings for Xplane
// simulation. Remove the leading "/* and trailing "*/" to enable:

// Select Megapirate board type:
//#define MPNG_BOARD_TYPE   CRIUS_V1
/*
  RCTIMER_CRIUS_V2    -- (DEFAULT!!!) Use ONLY for RCTimer CRIUS V2 board
  CRIUS_V1            -- Use this define for RCTimer CRIUS V1(1.1) board and all HobbyKing AIOP boards
  HK_RED_MULTIWII_PRO -- HobbyKing MultiWii Pro board with ITG3205 and BMA180, BMP085 sensors
  BLACK_VORTEX
  MULTIWII_PRO_EZ3_BLACK  -- ReadyToFlyQuads - MultiWii PRO Ez3.0 Blacked MAG Editon Flight Controller w/ GPS Option (NO COMPASS) 
 */
 
 // GPS port speed (Serial2) 38400 by default
//#define SERIAL2_BAUD 38400

// GPS driver selection
//#define GPS_PROTOCOL GPS_PROTOCOL_NONE
/*
	GPS_PROTOCOL_AUTO   (Default)
	GPS_PROTOCOL_NONE
	GPS_PROTOCOL_NMEA
	GPS_PROTOCOL_SIRF
	GPS_PROTOCOL_UBLOX
	GPS_PROTOCOL_IMU
	GPS_PROTOCOL_MTK
	GPS_PROTOCOL_HIL
	GPS_PROTOCOL_MTK19
*/


//#define HIL_MODE            HIL_MODE_DISABLED

/*
 *  // HIL_MODE SELECTION
 *  //
 *  // Mavlink supports
 *  // 2. HIL_MODE_SENSORS: full sensor simulation
 *
 */

