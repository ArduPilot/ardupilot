/*
 * gcs_mavlink_test_unit.h
 *
 *  Created on: 22 mai 2014
 *      Author: valentin
 */

#ifndef GCS_MAVLINK_TEST_UNIT_H_
#define GCS_MAVLINK_TEST_UNIT_H_

#include <stdint.h>
#include <cstdarg>
#include "../../libraries/GCS_MAVLink/include/mavlink/v1.0/mavlink_types.h"

#include "../../libraries/AP_Math/rotations.h"
#include "../../libraries/AP_Math/vector3.h"
#include "stubs/rc_channel_stub.h"
#include "stubs/ap_int16_stub.h"
#include "stubs/parameters_stub.h"
#include "stubs/ap_gps_stub.h"
#include "stubs/ap_ahrs_stub.h"
#include "stubs/ap_compass_stub.h"
#include "stubs/gps_glitch_stub.h"
#include "stubs/ac_poscontrol_stub.h"
#include "stubs/ap_battmonitor_stub.h"
#include "stubs/ap_inertialsensor_stub.h"
#include "stubs/ap_scheduler_stub.h"
#include "stubs/sitl_stub.h"
#include "stubs/analogin_stub.h"
#include "stubs/rc_input_stub.h"
#include "stubs/rc_output_stub.h"
#include "stubs/i2cdriver_stub.h"
#include "stubs/hal_scheduler_stub.h"
#include "stubs/gpio_stubs.h"
#include "stubs/util_stub.h"
#include "stubs/ap_hal_stub.h"
#include "stubs/ap_baro_stub.h"
#include "stubs/ap_mission_stub.h"
#include "stubs/uartdriver_stub.h"
#include "stubs/dataflask_class_stub.h"
#include "stubs/ap_param_stub.h"
#include "stubs/ac_wpnav_stub.h"
#include "stubs/ap_servorelayevents_stub.h"
#include "stubs/ac_fence_stub.h"
#include "stubs/ap_notify_stub.h"
#include "stubs/ap_camera_stub.h"
#include "stubs/ap_mount_stub.h"
#include "stubs/ap_rally_stub.h"
#include "stubs/ap_motors_stub.h"

#define NOINLINE
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define GRAVITY_MSS 9.80665f
#define LOCATION_MASK_OPTIONS_RELATIVE_ALT      (1<<0)
#define MASK_LOG_PM                     (1<<3)
#define PSTR(text) " "
#define MAX_RALLYPOINTS 6

// Auto Pilot modes
#define STABILIZE 0                     // hold level position
#define ACRO 1                          // rate control
#define ALT_HOLD 2                      // AUTO control
#define AUTO 3                          // AUTO control
#define GUIDED 4                        // AUTO control
#define LOITER 5                        // Hold a single location
#define RTL 6                           // AUTO control
#define CIRCLE 7                        // AUTO control
#define LAND 9                          // AUTO control
#define OF_LOITER 10                    // Hold a single location using optical flow sensor
#define DRIFT 11                        // DRIFT mode (Note: 12 is no longer used)
#define SPORT 13                        // earth frame rate control
#define FLIP        14                  // flip the vehicle on the roll axis
#define AUTOTUNE    15                  // autotune the vehicle's roll and pitch gains
#define HYBRID      16                  // hybrid - position hold with manual override
#define NUM_MODES   17

// Channel definition
#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7
#define CH_9 8
#define CH_10 9
#define CH_11 10
#define CH_12 11
#define CH_13 12
#define CH_14 13
#define CH_15 14
#define CH_16 15
#define CH_17 16
#define CH_18 17

#define AP_MAX_NAME_SIZE 16
#define PROGMEM
#define AP_GROUPEND
#define AP_GROUPINFO(name, idx, class, element, def) {'a'}

#endif /* GCS_MAVLINK_TEST_UNIT_H_ */
