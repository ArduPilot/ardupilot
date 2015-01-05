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
#include <StorageManager.h>
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
#include <AP_Baro_Glitch.h>
#include <AP_InertialSensor.h>
#include <AP_InertialNav.h>
#include <AP_NavEKF.h>
#include <AP_Mission.h>
#include <AP_Rally.h>
#include <AP_BattMonitor.h>
#include <AP_Terrain.h>
#include <AP_OpticalFlow.h>
#include <Parameters.h>
#include <stdio.h>
#include <getopt.h>
#include <errno.h>
#include <fenv.h>

#ifndef INT16_MIN
#define INT16_MIN -32768
#define INT16_MAX 32767
#endif

#include "LogReader.h"

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

static Parameters g;

static AP_InertialSensor ins;
static AP_Baro barometer;
static AP_GPS gps;
static AP_Compass_HIL compass;
static AP_AHRS_NavEKF ahrs(ins, barometer, gps);
static GPS_Glitch gps_glitch(gps);
static Baro_Glitch baro_glitch(barometer);
static AP_InertialNav inertial_nav(ahrs, barometer, gps_glitch, baro_glitch);
static AP_Vehicle::FixedWing aparm;
static AP_Airspeed airspeed(aparm);
static DataFlash_File dataflash("logs");

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
SITL sitl;
#endif

static const NavEKF &NavEKF = ahrs.get_NavEKF();

static LogReader LogReader(ahrs, ins, barometer, compass, gps, airspeed, dataflash);

static FILE *plotf;
static FILE *plotf2;
static FILE *ekf1f;
static FILE *ekf2f;
static FILE *ekf3f;
static FILE *ekf4f;

static bool done_parameters;
static bool done_baro_init;
static bool done_home_init;
static uint16_t update_rate = 50;
static uint32_t arm_time_ms;

static uint8_t num_user_parameters;
static struct {
    char name[17];
    float value;
} user_parameters[100];

static const struct LogStructure log_structure[] PROGMEM = {
    LOG_COMMON_STRUCTURES
};

// setup the var_info table
AP_Param param_loader(var_info);

static void usage(void)
{
    ::printf("Options:\n");
    ::printf(" -rRATE     set IMU rate in Hz\n");
    ::printf(" -pNAME=VALUE set parameter NAME to VALUE\n");
    ::printf(" -aMASK     set accel mask (1=accel1 only, 2=accel2 only, 3=both)\n");
    ::printf(" -gMASK     set gyro mask (1=gyro1 only, 2=gyro2 only, 3=both)\n");
    ::printf(" -A time    arm at time milliseconds)\n");
}

void setup()
{
    ::printf("Starting\n");

    const char *filename = "log.bin";
    uint8_t argc;
    char * const *argv;
    int opt;

    hal.util->commandline_arguments(argc, argv);

	while ((opt = getopt(argc, argv, "r:p:ha:g:A:")) != -1) {
		switch (opt) {
        case 'h':
            usage();
            exit(0);

        case 'r':
			update_rate = strtol(optarg, NULL, 0);
            break;

        case 'g':
            LogReader.set_gyro_mask(strtol(optarg, NULL, 0));
            break;

        case 'a':
            LogReader.set_accel_mask(strtol(optarg, NULL, 0));
            break;

        case 'A':
            arm_time_ms = strtoul(optarg, NULL, 0);
            break;

        case 'p':
            char *eq = strchr(optarg, '=');
            if (eq == NULL) {
                ::printf("Usage: -p NAME=VALUE\n");
                exit(1);
            }
            *eq++ = 0;
            strncpy(user_parameters[num_user_parameters].name, optarg, 16);
            user_parameters[num_user_parameters].value = atof(eq);
            num_user_parameters++;
            if (num_user_parameters >= sizeof(user_parameters)/sizeof(user_parameters[0])) {
                ::printf("Too many user parameters\n");
                exit(1);
            }
            break;
        }
    }

	argv += optind;
	argc -= optind;

    if (argc > 0) {
        filename = argv[0];
    }

    hal.console->printf("Processing log %s\n", filename);
    if (update_rate != 0) {
        hal.console->printf("Using an update rate of %u Hz\n", update_rate);
    }

    load_parameters();

    if (!LogReader.open_log(filename)) {
        perror(filename);
        exit(1);
    }

    dataflash.Init(log_structure, sizeof(log_structure)/sizeof(log_structure[0]));
    dataflash.StartNewLog();

    LogReader.wait_type(LOG_GPS_MSG);
    LogReader.wait_type(LOG_IMU_MSG);
    LogReader.wait_type(LOG_GPS_MSG);
    LogReader.wait_type(LOG_IMU_MSG);

    feenableexcept(FE_INVALID | FE_OVERFLOW);

    ahrs.set_compass(&compass);
    ahrs.set_fly_forward(true);
    ahrs.set_wind_estimation(true);
    ahrs.set_correct_centrifugal(true);

    if (arm_time_ms != 0) {
        ahrs.set_armed(false);
    }

    barometer.init();
    barometer.setHIL(0);
    barometer.update();
    compass.init();
    inertial_nav.init();
    ins.set_hil_mode();

    switch (update_rate) {
    case 0:
    case 50:
        ins.init(AP_InertialSensor::WARM_START, AP_InertialSensor::RATE_50HZ);
        break;
    case 100:
        ins.init(AP_InertialSensor::WARM_START, AP_InertialSensor::RATE_100HZ);
        break;
    case 200:
        ins.init(AP_InertialSensor::WARM_START, AP_InertialSensor::RATE_200HZ);
        break;
    case 400:
        ins.init(AP_InertialSensor::WARM_START, AP_InertialSensor::RATE_400HZ);
        break;
    }

    plotf = fopen("plot.dat", "w");
    plotf2 = fopen("plot2.dat", "w");
    ekf1f = fopen("EKF1.dat", "w");
    ekf2f = fopen("EKF2.dat", "w");
    ekf3f = fopen("EKF3.dat", "w");
    ekf4f = fopen("EKF4.dat", "w");

    fprintf(plotf, "time SIM.Roll SIM.Pitch SIM.Yaw BAR.Alt FLIGHT.Roll FLIGHT.Pitch FLIGHT.Yaw FLIGHT.dN FLIGHT.dE FLIGHT.Alt AHR2.Roll AHR2.Pitch AHR2.Yaw DCM.Roll DCM.Pitch DCM.Yaw EKF.Roll EKF.Pitch EKF.Yaw INAV.dN INAV.dE INAV.Alt EKF.dN EKF.dE EKF.Alt\n");
    fprintf(plotf2, "time E1 E2 E3 VN VE VD PN PE PD GX GY GZ WN WE MN ME MD MX MY MZ E1ref E2ref E3ref\n");
    fprintf(ekf1f, "timestamp TimeMS Roll Pitch Yaw VN VE VD PN PE PD GX GY GZ\n");
    fprintf(ekf2f, "timestamp TimeMS AX AY AZ VWN VWE MN ME MD MX MY MZ\n");
    fprintf(ekf3f, "timestamp TimeMS IVN IVE IVD IPN IPE IPD IMX IMY IMZ IVT\n");
    fprintf(ekf4f, "timestamp TimeMS SV SP SH SMX SMY SMZ SVT OFN EFE FS DS\n");

    ahrs.set_ekf_use(true);

    ::printf("Waiting for InertialNav to start\n");
    while (!ahrs.have_inertial_nav()) {
        uint8_t type;
        if (!LogReader.update(type)) break;
        read_sensors(type);
        if (type == LOG_GPS_MSG && 
            gps.status() >= AP_GPS::GPS_OK_FIX_3D && 
            done_baro_init && !done_home_init) {
            const Location &loc = gps.location();
            ::printf("GPS Lock at %.7f %.7f %.2fm time=%.1f seconds\n", 
                     loc.lat * 1.0e-7f, 
                     loc.lng * 1.0e-7f,
                     loc.alt * 0.01f,
                     hal.scheduler->millis()*0.001f);
            ahrs.set_home(loc);
            compass.set_initial_location(loc.lat, loc.lng);
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


/*
  setup user -p parameters
 */
static void set_user_parameters(void)
{
    for (uint8_t i=0; i<num_user_parameters; i++) {
        if (!LogReader.set_parameter(user_parameters[i].name, user_parameters[i].value)) {
            ::printf("Failed to set parameter %s to %f\n", user_parameters[i].name, user_parameters[i].value);
            exit(1);
        }
    }
}

static void read_sensors(uint8_t type)
{
    if (!done_parameters && type != LOG_FORMAT_MSG && type != LOG_PARAMETER_MSG) {
        done_parameters = true;
        set_user_parameters();
    }
    if (type == LOG_GPS_MSG) {
        gps.update();
        if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
            ahrs.estimate_wind();
        }
    } else if (type == LOG_IMU_MSG) {
        uint32_t update_delta_usec = 1e6 / update_rate;
        uint8_t update_count = update_rate>0?update_rate/50:1;
        for (uint8_t i=0; i<update_count; i++) {
            ahrs.update();
            if (ahrs.get_home().lat != 0) {
                inertial_nav.update(ins.get_delta_time());
            }
            hal.scheduler->stop_clock(hal.scheduler->micros() + (i+1)*update_delta_usec);
            dataflash.Log_Write_EKF(ahrs,false);
            dataflash.Log_Write_AHRS2(ahrs);
            ins.set_gyro(0, ins.get_gyro());
            ins.set_accel(0, ins.get_accel());
        }
    } else if ((type == LOG_PLANE_COMPASS_MSG && LogReader.vehicle == LogReader::VEHICLE_PLANE) ||
               (type == LOG_COPTER_COMPASS_MSG && LogReader.vehicle == LogReader::VEHICLE_COPTER) ||
               (type == LOG_ROVER_COMPASS_MSG && LogReader.vehicle == LogReader::VEHICLE_ROVER)) {
        compass.read();
    } else if (type == LOG_PLANE_AIRSPEED_MSG && LogReader.vehicle == LogReader::VEHICLE_PLANE) {
        ahrs.set_airspeed(&airspeed);
    } else if (type == LOG_BARO_MSG) {
        barometer.update();
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

        if (arm_time_ms != 0 && hal.scheduler->millis() > arm_time_ms) {
            if (!ahrs.get_armed()) {
                ahrs.set_armed(true);
                ::printf("Arming at %u ms\n", (unsigned)hal.scheduler->millis());
            }
        }

        if (!LogReader.update(type)) {
            ::printf("End of log at %.1f seconds\n", hal.scheduler->millis()*0.001f);
            fclose(plotf);
            exit(0);
        }
        read_sensors(type);

        if ((type == LOG_PLANE_ATTITUDE_MSG && LogReader.vehicle == LogReader::VEHICLE_PLANE) ||
            (type == LOG_COPTER_ATTITUDE_MSG && LogReader.vehicle == LogReader::VEHICLE_COPTER) ||
            (type == LOG_ROVER_ATTITUDE_MSG && LogReader.vehicle == LogReader::VEHICLE_ROVER)) {

            Vector3f ekf_euler;
            Vector3f velNED;
            Vector3f posNED;
            Vector3f gyroBias;
            float accelWeighting;
            float accelZBias1;
            float accelZBias2;
            Vector3f windVel;
            Vector3f magNED;
            Vector3f magXYZ;
            Vector3f DCM_attitude;
            Vector3f ekf_relpos;
            Vector3f velInnov;
            Vector3f posInnov;
            Vector3f magInnov;
            float    tasInnov;
            float velVar;
            float posVar;
            float hgtVar;
            Vector3f magVar;
            float tasVar;
            Vector2f offset;
            uint8_t faultStatus;

            const Matrix3f &dcm_matrix = ((AP_AHRS_DCM)ahrs).get_dcm_matrix();
            dcm_matrix.to_euler(&DCM_attitude.x, &DCM_attitude.y, &DCM_attitude.z);
            NavEKF.getEulerAngles(ekf_euler);
            NavEKF.getVelNED(velNED);
            NavEKF.getPosNED(posNED);
            NavEKF.getGyroBias(gyroBias);
            NavEKF.getIMU1Weighting(accelWeighting);
            NavEKF.getAccelZBias(accelZBias1, accelZBias2);
            NavEKF.getWind(windVel);
            NavEKF.getMagNED(magNED);
            NavEKF.getMagXYZ(magXYZ);
            NavEKF.getInnovations(velInnov, posInnov, magInnov, tasInnov);
            NavEKF.getVariances(velVar, posVar, hgtVar, magVar, tasVar, offset);
            NavEKF.getFilterFaults(faultStatus);
            NavEKF.getPosNED(ekf_relpos);
            Vector3f inav_pos = inertial_nav.get_position() * 0.01f;
            float temp = degrees(ekf_euler.z);

            if (temp < 0.0f) temp = temp + 360.0f;
            fprintf(plotf, "%.3f %.1f %.1f %.1f %.2f %.1f %.1f %.1f %.2f %.2f %.2f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.2f %.2f %.2f %.2f %.2f %.2f\n",
                    hal.scheduler->millis() * 0.001f,
                    LogReader.get_sim_attitude().x,
                    LogReader.get_sim_attitude().y,
                    LogReader.get_sim_attitude().z,
                    barometer.get_altitude(),
                    LogReader.get_attitude().x,
                    LogReader.get_attitude().y,
                    wrap_180_cd(LogReader.get_attitude().z*100)*0.01f,
                    LogReader.get_inavpos().x,
                    LogReader.get_inavpos().y,
                    LogReader.get_relalt(),
                    LogReader.get_ahr2_attitude().x,
                    LogReader.get_ahr2_attitude().y,
                    wrap_180_cd(LogReader.get_ahr2_attitude().z*100)*0.01f,
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
            float       gyrX  = (float)(6000*degrees(gyroBias.x)); // centi-deg/min
            float       gyrY  = (float)(6000*degrees(gyroBias.y)); // centi-deg/min
            float       gyrZ  = (float)(6000*degrees(gyroBias.z)); // centi-deg/min

            // print EKF1 data packet
            fprintf(ekf1f, "%.3f %u %d %d %u %.2f %.2f %.2f %.2f %.2f %.2f %.0f %.0f %.0f\n",
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
            int8_t  accWeight  = (int8_t)(100*accelWeighting);
            int8_t  acc1  = (int8_t)(100*accelZBias1);
            int8_t  acc2  = (int8_t)(100*accelZBias2);
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
                    accWeight, 
                    acc1, 
                    acc2, 
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
            int16_t sqrtvarV = (int16_t)(constrain_float(100*velVar,INT16_MIN,INT16_MAX));
            int16_t sqrtvarP = (int16_t)(constrain_float(100*posVar,INT16_MIN,INT16_MAX));
            int16_t sqrtvarH = (int16_t)(constrain_float(100*hgtVar,INT16_MIN,INT16_MAX));
            int16_t sqrtvarMX = (int16_t)(constrain_float(100*magVar.x,INT16_MIN,INT16_MAX));
            int16_t sqrtvarMY = (int16_t)(constrain_float(100*magVar.y,INT16_MIN,INT16_MAX));
            int16_t sqrtvarMZ = (int16_t)(constrain_float(100*magVar.z,INT16_MIN,INT16_MAX));
            int16_t sqrtvarVT = (int16_t)(constrain_float(100*tasVar,INT16_MIN,INT16_MAX));
            int16_t offsetNorth = (int8_t)(constrain_float(offset.x,INT16_MIN,INT16_MAX));
            int16_t offsetEast = (int8_t)(constrain_float(offset.y,INT16_MIN,INT16_MAX));

            // print EKF4 data packet
            fprintf(ekf4f, "%.3f %u %d %d %d %d %d %d %d %d %d %d\n",
                    hal.scheduler->millis() * 0.001f,
                    (unsigned)hal.scheduler->millis(),
                    (int)sqrtvarV,
                    (int)sqrtvarP,
                    (int)sqrtvarH,
                    (int)sqrtvarMX, 
                    (int)sqrtvarMY, 
                    (int)sqrtvarMZ,
                    (int)sqrtvarVT,
                    (int)offsetNorth,
                    (int)offsetEast,
                    (int)faultStatus);
        }
    }
}

AP_HAL_MAIN();
