/*
 * Arming_Failure.h
 *
 *  Created on: 2015-11-06
 *      Author: rmackay9
 */

#ifndef ARMING_FAILURE_H_
#define ARMING_FAILURE_H_

    enum {
        // baro checks
        k_arming_check_baro_0 = 0,

        // compass checks
        k_arming_check_compass_3 = 3
    };

#define ARMING_FAILURE_BARO_0 "Barometer not healthy"
#define ARMING_FAILURE_BARO_1 "Altitude disparity"
#define ARMING_FAILURE_AIRSPEED_3 "airspeed not healthy"
#define ARMING_FAILURE_LOGGING_4 "logging not available"
#define ARMING_FAILURE_INS_5 "gyros not healthy"
#define ARMING_FAILURE_INS_6 "gyros not calibrated"
#define ARMING_FAILURE_INS_7 "accels not healthy"
#define ARMING_FAILURE_INS_8 "3D accel cal needed"
#define ARMING_FAILURE_INS_9 "inconsistent Accelerometers"
#define ARMING_FAILURE_INS_10 "inconsistent gyros"
#define ARMING_FAILURE_COMPASS_11 "Compass not healthy"
#define ARMING_FAILURE_COMPASS_12 "Compass not calibrated"
#define ARMING_FAILURE_COMPASS_13 "Compass calibration running"
#define ARMING_FAILURE_COMPASS_14 "Compass calibrated requires reboot"
#define ARMING_FAILURE_COMPASS_15 "Compass offsets too high"
#define ARMING_FAILURE_COMPASS_16 "Check mag field"

#endif /* ARMING_FAILURE_H_ */
