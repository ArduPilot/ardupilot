/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>
#include <AP_ADC.h>
#include <AP_Declination.h>
#include <AP_ADC_AnalogSource.h>
#include <Filter.h>
#include <AP_Buffer.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AP_Notify.h>
#include <DataFlash.h>
#include <GCS_MAVLink.h>
#include <AP_GPS.h>
#include <AP_GPS_Glitch.h>
#include <AP_AHRS.h>
#include <SITL.h>
#include <AP_Compass.h>
#include <AP_Baro.h>
#include <AP_InertialSensor.h>
#include <AP_InertialNav.h>
#include <AP_NavEKF.h>
#include <stdio.h>

#include "LogReader.h"

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

static AP_InertialSensor_HIL ins;
static AP_Baro_HIL barometer;
static AP_GPS_HIL gps_driver;
static GPS *g_gps = &gps_driver;
static AP_Compass_HIL compass;
static AP_AHRS_NavEKF ahrs(ins, barometer, g_gps);
static GPS_Glitch gps_glitch(g_gps);
static AP_InertialNav inertial_nav(ahrs, barometer, g_gps, gps_glitch);
static AP_Vehicle::FixedWing aparm;
static AP_Airspeed airspeed(aparm);

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
SITL sitl;
#endif

static const NavEKF &NavEKF = ahrs.get_NavEKF();

static LogReader LogReader(ins, barometer, compass, g_gps, airspeed);

static FILE *plotf;
static FILE *plotf2;
static FILE *ekf1f;
static FILE *ekf2f;
static FILE *ekf3f;
static FILE *ekf4f;

static bool done_baro_init;
static bool done_home_init;

void setup()
{
    ::printf("Starting\n");

    const char *filename = "log.bin";
    uint8_t argc;
    char * const *argv;
    hal.util->commandline_arguments(argc, argv);
    if (argc > 1) {
        filename = argv[1];
    }

    hal.console->printf("Processing log %s\n", filename);
    LogReader.open_log(filename);

    LogReader.wait_type(LOG_GPS_MSG);
    LogReader.wait_type(LOG_IMU_MSG);
    LogReader.wait_type(LOG_GPS_MSG);
    LogReader.wait_type(LOG_IMU_MSG);

    ahrs.set_compass(&compass);
    ahrs.set_fly_forward(true);
    ahrs.set_wind_estimation(true);
    ahrs.set_correct_centrifugal(true);
    
    barometer.init();
    barometer.setHIL(0);
    barometer.read();
    compass.init();
    inertial_nav.init();
    ins.init(AP_InertialSensor::WARM_START, AP_InertialSensor::RATE_50HZ);

    plotf = fopen("plot.dat", "w");
    plotf2 = fopen("plot2.dat", "w");
    ekf1f = fopen("EKF1.dat", "w");
    ekf2f = fopen("EKF2.dat", "w");
    ekf3f = fopen("EKF3.dat", "w");
    ekf4f = fopen("EKF4.dat", "w");

    fprintf(plotf, "time SIM.Roll SIM.Pitch SIM.Yaw BAR.Alt FLIGHT.Roll FLIGHT.Pitch FLIGHT.Yaw FLIGHT.dN FLIGHT.dE FLIGHT.Alt DCM.Roll DCM.Pitch DCM.Yaw EKF.Roll EKF.Pitch EKF.Yaw INAV.dN INAV.dE INAV.Alt EKF.dN EKF.dE EKF.Alt\n");
    fprintf(plotf2, "time E1 E2 E3 VN VE VD PN PE PD GX GY GZ WN WE MN ME MD MX MY MZ E1ref E2ref E3ref\n");
    fprintf(ekf1f, "timestamp TimeMS Roll Pitch Yaw VN VE VD PN PE PD GX GY GZ\n");
    fprintf(ekf2f, "timestamp TimeMS AX AY AZ VWN VWE MN ME MD MX MY MZ\n");
    fprintf(ekf3f, "timestamp TimeMS IVN IVE IVD IPN IPE IPD IMX IMY IMZ IVT\n");
    fprintf(ekf4f, "timestamp TimeMS SVN SVE SVD SPN SPE SPD SMX SMY SMZ SVT\n");

    ahrs.set_ekf_use(true);

    ::printf("Waiting for InertialNav to start\n");
    while (!ahrs.have_inertial_nav()) {
        uint8_t type;
        if (!LogReader.update(type)) break;
        read_sensors(type);
        if (type == LOG_GPS_MSG && g_gps->status() >= GPS::GPS_OK_FIX_3D && done_baro_init && !done_home_init) {
            ::printf("GPS Lock at %.7f %.7f %.2fm time=%.1f seconds\n", 
                     g_gps->latitude*1.0e-7f, 
                     g_gps->longitude*1.0e-7f,
                     g_gps->altitude_cm*0.01f,
                     hal.scheduler->millis()*0.001f);
            ahrs.set_home(g_gps->latitude, g_gps->longitude, g_gps->altitude_cm);
            compass.set_initial_location(g_gps->latitude, g_gps->longitude);
            inertial_nav.setup_home_position();
            done_home_init = true;
        }
    }

    ::printf("InertialNav started\n");

    if (!ahrs.have_inertial_nav()) {
        ::printf("Failed to start NavEKF\n");
        exit(1);
    }
}

static void read_sensors(uint8_t type)
{
    if (type == LOG_GPS_MSG) {
        g_gps->update();
        if (g_gps->status() >= GPS::GPS_OK_FIX_3D) {
            ahrs.estimate_wind();
        }
    } else if (type == LOG_IMU_MSG) {
        ahrs.update();
        if (ahrs.get_home().lat != 0) {
            inertial_nav.update(ins.get_delta_time());
        }
    } else if ((type == LOG_PLANE_COMPASS_MSG && LogReader.vehicle == LogReader::VEHICLE_PLANE) ||
               (type == LOG_COPTER_COMPASS_MSG && LogReader.vehicle == LogReader::VEHICLE_COPTER)) {
        compass.read();
    } else if (type == LOG_PLANE_AIRSPEED_MSG && LogReader.vehicle == LogReader::VEHICLE_PLANE) {
        ahrs.set_airspeed(&airspeed);
    } else if (type == LOG_BARO_MSG) {
        barometer.read();
        if (!done_baro_init) {
            done_baro_init = true;
            ::printf("Barometer initialised\n");
            barometer.update_calibration();
        }
    }
}

void loop()
{
    while (true) {
        uint8_t type;
        if (!LogReader.update(type)) {
            ::printf("End of log at %.1f seconds\n", hal.scheduler->millis()*0.001f);
            fclose(plotf);
            exit(0);
        }
        read_sensors(type);

        if ((type == LOG_PLANE_ATTITUDE_MSG && LogReader.vehicle == LogReader::VEHICLE_PLANE) ||
            (type == LOG_COPTER_ATTITUDE_MSG && LogReader.vehicle == LogReader::VEHICLE_COPTER)) {

            Vector3f ekf_euler;
            Vector3f velNED;
            Vector3f posNED;
            Vector3f gyroBias;
            Vector3f accelBias;
            Vector3f windVel;
            Vector3f magNED;
            Vector3f magXYZ;
            Vector3f DCM_attitude;
            Vector3f ekf_relpos;
            Vector3f velInnov;
            Vector3f posInnov;
            Vector3f magInnov;
            float    tasInnov;
            Vector3f velVar;
            Vector3f posVar;
            Vector3f magVar;
            float    tasVar;

            ahrs.get_secondary_attitude(DCM_attitude);
            NavEKF.getEulerAngles(ekf_euler);
            NavEKF.getVelNED(velNED);
            NavEKF.getPosNED(posNED);
            NavEKF.getGyroBias(gyroBias);
            NavEKF.getAccelBias(accelBias);
            NavEKF.getWind(windVel);
            NavEKF.getMagNED(magNED);
            NavEKF.getMagXYZ(magXYZ);
            NavEKF.getInnovations(velInnov, posInnov, magInnov, tasInnov);
            NavEKF.getVariances(velVar, posVar, magVar, tasVar);
            ahrs.get_relative_position_NED(ekf_relpos);
            Vector3f inav_pos = inertial_nav.get_position() * 0.01f;
            float temp = degrees(ekf_euler.z);

            if (temp < 0.0f) temp = temp + 360.0f;
            fprintf(plotf, "%.3f %.1f %.1f %.1f %.2f %.1f %.1f %.1f %.2f %.2f %.2f %.1f %.1f %.1f %.1f %.1f %.1f %.2f %.2f %.2f %.2f %.2f %.2f\n",
                    hal.scheduler->millis() * 0.001f,
                    LogReader.get_sim_attitude().x,
                    LogReader.get_sim_attitude().y,
                    LogReader.get_sim_attitude().z,
                    barometer.get_altitude(),
                    LogReader.get_attitude().x,
                    LogReader.get_attitude().y,
                    LogReader.get_attitude().z,
                    LogReader.get_inavpos().x,
                    LogReader.get_inavpos().y,
                    LogReader.get_relalt(),
                    degrees(DCM_attitude.x),
                    degrees(DCM_attitude.y),
                    degrees(DCM_attitude.z),
                    degrees(ekf_euler.x),
                    degrees(ekf_euler.y),
                    degrees(ekf_euler.z),
                    inav_pos.x,
                    inav_pos.y,
                    inav_pos.z,
                    ekf_relpos.x,
                    ekf_relpos.y,
                    -ekf_relpos.z);
            fprintf(plotf2, "%.3f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f\n",
                    hal.scheduler->millis() * 0.001f,
                    degrees(ekf_euler.x),
                    degrees(ekf_euler.y),
                    temp,
                    velNED.x, 
                    velNED.y, 
                    velNED.z, 
                    posNED.x, 
                    posNED.y, 
                    posNED.z, 
                    60*degrees(gyroBias.x), 
                    60*degrees(gyroBias.y), 
                    60*degrees(gyroBias.z), 
                    windVel.x, 
                    windVel.y, 
                    magNED.x, 
                    magNED.y, 
                    magNED.z, 
                    magXYZ.x, 
                    magXYZ.y, 
                    magXYZ.z,
                    LogReader.get_attitude().x,
                    LogReader.get_attitude().y,
                    LogReader.get_attitude().z);

            // define messages for EKF1 data packet
            int16_t     roll  = (int16_t)(100*degrees(ekf_euler.x)); // roll angle (centi-deg)
            int16_t     pitch = (int16_t)(100*degrees(ekf_euler.y)); // pitch angle (centi-deg)
            uint16_t    yaw   = (uint16_t)wrap_360_cd(100*degrees(ekf_euler.z)); // yaw angle (centi-deg)
            float       velN  = (float)(velNED.x); // velocity North (m/s)
            float       velE  = (float)(velNED.y); // velocity East (m/s)
            float       velD  = (float)(velNED.z); // velocity Down (m/s)
            float       posN  = (float)(posNED.x); // metres North
            float       posE  = (float)(posNED.y); // metres East
            float       posD  = (float)(posNED.z); // metres Down
            int16_t     gyrX  = (int16_t)(6000*degrees(gyroBias.x)); // centi-deg/min
            int16_t     gyrY  = (int16_t)(6000*degrees(gyroBias.y)); // centi-deg/min
            int16_t     gyrZ  = (int16_t)(6000*degrees(gyroBias.z)); // centi-deg/min

            // print EKF1 data packet
            fprintf(ekf1f, "%.3f %u %d %d %u %.2f %.2f %.2f %.2f %.2f %.2f %d %d %d\n",
                    hal.scheduler->millis() * 0.001f,
                    hal.scheduler->millis(),
                    roll, 
                    pitch, 
                    yaw, 
                    velN, 
                    velE, 
                    velD, 
                    posN, 
                    posE, 
                    posD, 
                    gyrX,
                    gyrY,
                    gyrZ);

            // define messages for EKF2 data packet
            int8_t  accX  = (int8_t)(100*accelBias.x);
            int8_t  accY  = (int8_t)(100*accelBias.y);
            int8_t  accZ  = (int8_t)(100*accelBias.z);
            int16_t windN = (int16_t)(100*windVel.x);
            int16_t windE = (int16_t)(100*windVel.y);
            int16_t magN  = (int16_t)(magNED.x);
            int16_t magE  = (int16_t)(magNED.y);
            int16_t magD  = (int16_t)(magNED.z);
            int16_t magX  = (int16_t)(magXYZ.x);
            int16_t magY  = (int16_t)(magXYZ.y);
            int16_t magZ  = (int16_t)(magXYZ.z);

            // print EKF2 data packet
            fprintf(ekf2f, "%.3f %d %d %d %d %d %d %d %d %d %d %d %d\n",
                    hal.scheduler->millis() * 0.001f,
                    hal.scheduler->millis(),
                    accX, 
                    accY, 
                    accZ, 
                    windN, 
                    windE, 
                    magN, 
                    magE, 
                    magD, 
                    magX,
                    magY,
                    magZ);

            // define messages for EKF3 data packet
            int16_t innovVN = (int16_t)(100*velInnov.x);
            int16_t innovVE = (int16_t)(100*velInnov.y);
            int16_t innovVD = (int16_t)(100*velInnov.z);
            int16_t innovPN = (int16_t)(100*posInnov.x);
            int16_t innovPE = (int16_t)(100*posInnov.y);
            int16_t innovPD = (int16_t)(100*posInnov.z);
            int16_t innovMX = (int16_t)(magInnov.x);
            int16_t innovMY = (int16_t)(magInnov.y);
            int16_t innovMZ = (int16_t)(magInnov.z);
            int16_t innovVT = (int16_t)(100*tasInnov);

            // print EKF3 data packet
            fprintf(ekf3f, "%.3f %d %d %d %d %d %d %d %d %d %d %d\n",
                    hal.scheduler->millis() * 0.001f,
                    hal.scheduler->millis(),
                    innovVN, 
                    innovVE, 
                    innovVD, 
                    innovPN, 
                    innovPE, 
                    innovPD, 
                    innovMX, 
                    innovMY, 
                    innovMZ, 
                    innovVT);

            // define messages for EKF4 data packet
            int16_t sqrtvarVN = (int16_t)(100*sqrtf(velVar.x));
            int16_t sqrtvarVE = (int16_t)(100*sqrtf(velVar.y));
            int16_t sqrtvarVD = (int16_t)(100*sqrtf(velVar.z));
            int16_t sqrtvarPN = (int16_t)(100*sqrtf(posVar.x));
            int16_t sqrtvarPE = (int16_t)(100*sqrtf(posVar.y));
            int16_t sqrtvarPD = (int16_t)(100*sqrtf(posVar.z));
            int16_t sqrtvarMX = (int16_t)(sqrtf(magVar.x));
            int16_t sqrtvarMY = (int16_t)(sqrtf(magVar.y));
            int16_t sqrtvarMZ = (int16_t)(sqrtf(magVar.z));
            int16_t sqrtvarVT = (int16_t)(100*sqrtf(tasVar));

            // print EKF4 data packet
            fprintf(ekf4f, "%.3f %d %d %d %d %d %d %d %d %d %d %d\n",
                    hal.scheduler->millis() * 0.001f,
                    hal.scheduler->millis(),
                    sqrtvarVN, 
                    sqrtvarVE, 
                    sqrtvarVD, 
                    sqrtvarPN, 
                    sqrtvarPE, 
                    sqrtvarPD, 
                    sqrtvarMX, 
                    sqrtvarMY, 
                    sqrtvarMZ,
                    sqrtvarVT);
        }
    }
}

AP_HAL_MAIN();
