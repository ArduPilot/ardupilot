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

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
SITL sitl;
#endif

static const NavEKF &NavEKF = ahrs.get_NavEKF();

static LogReader LogReader(ins, barometer, compass, g_gps);

static FILE *plotf;
static FILE *plotf2;

static bool done_baro_init;
static bool done_home_init;

void setup()
{
    ::printf("Starting\n");

    LogReader.open_log("log.bin");

    LogReader.wait_type(LOG_GPS_MSG);
    LogReader.wait_type(LOG_IMU_MSG);
    LogReader.wait_type(LOG_GPS_MSG);
    LogReader.wait_type(LOG_IMU_MSG);

    ahrs.set_compass(&compass);
    ahrs.set_fly_forward(true);
    ahrs.set_wind_estimation(true);
    
    barometer.init();
    barometer.setHIL(0);
    barometer.read();
    compass.init();
    inertial_nav.init();
    ins.init(AP_InertialSensor::WARM_START, AP_InertialSensor::RATE_50HZ);

    plotf = fopen("plot.dat", "w");
    plotf2 = fopen("plot2.dat", "w");
    fprintf(plotf, "time SIM.Roll SIM.Pitch SIM.Yaw BAR.Alt FLIGHT.Roll FLIGHT.Pitch FLIGHT.Yaw FLIGHT.dN FLIGHT.dE FLIGHT.Alt DCM.Roll DCM.Pitch DCM.Yaw EKF.Roll EKF.Pitch EKF.Yaw INAV.dN INAV.dE INAV.Alt EKF.dN EKF.dE EKF.Alt\n");
    fprintf(plotf2, "time E1 E2 E3 VN VE VD PN PE PD GX GY GZ AX AY AZ MN ME MD MX MY MZ E1ref E2ref E3ref\n");

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

    if (!ahrs.have_inertial_nav()) {
        ::printf("Failed to start NavEKF\n");
        exit(1);
    }
}

static void read_sensors(uint8_t type)
{
    if (type == LOG_GPS_MSG) {
        g_gps->update();
    } else if (type == LOG_IMU_MSG) {
        ahrs.update();
        if (ahrs.get_home().lat != 0) {
            inertial_nav.update(ins.get_delta_time());
        }
    } else if ((type == LOG_PLANE_COMPASS_MSG && LogReader.vehicle == LogReader::VEHICLE_PLANE) ||
               (type == LOG_COPTER_COMPASS_MSG && LogReader.vehicle == LogReader::VEHICLE_COPTER)) {
        compass.read();
    } else if (type == LOG_PLANE_NTUN_MSG && LogReader.vehicle == LogReader::VEHICLE_PLANE) {
        barometer.read();
        if (!done_baro_init) {
            done_baro_init = true;
            barometer.update_calibration();
        }
    } else if (type == LOG_COPTER_CONTROL_TUNING_MSG && LogReader.vehicle == LogReader::VEHICLE_COPTER) {
        barometer.read();
        if (!done_baro_init) {
            done_baro_init = true;
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
            Vector3f magNED;
            Vector3f magXYZ;
            Vector3f DCM_attitude;
            Vector3f ekf_relpos;
            ahrs.get_secondary_attitude(DCM_attitude);
            NavEKF.getEulerAngles(ekf_euler);
            NavEKF.getVelNED(velNED);
            NavEKF.getPosNED(posNED);
            NavEKF.getGyroBias(gyroBias);
            NavEKF.getAccelBias(accelBias);
            NavEKF.getMagNED(magNED);
            NavEKF.getMagXYZ(magXYZ);
            ekf_relpos = ahrs.get_relative_position_NED();
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
                    LogReader.get_inavpos().z,
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
            fprintf(plotf2, "%.3f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.3f %.3f %.3f %.4f %.4f %.4f %.4f %.4f %.4f %.1f %.1f %.1f\n",
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
                    gyroBias.x, 
                    gyroBias.y, 
                    gyroBias.z, 
                    accelBias.x, 
                    accelBias.y, 
                    accelBias.z, 
                    magNED.x, 
                    magNED.y, 
                    magNED.z, 
                    magXYZ.x, 
                    magXYZ.y, 
                    magXYZ.z,
                    LogReader.get_attitude().x,
                    LogReader.get_attitude().y,
                    LogReader.get_attitude().z);
        }
    }
}

AP_HAL_MAIN();
