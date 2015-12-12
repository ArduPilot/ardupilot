// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//
// test harness for vibration testing
//

#include <stdarg.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_ADC/AP_ADC.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <Filter/Filter.h>
#include <DataFlash/DataFlash.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Mission/AP_Mission.h>
#include <StorageManager/StorageManager.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_NavEKF/AP_NavEKF.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Rally/AP_Rally.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include <drivers/drv_accel.h>
#include <drivers/drv_hrt.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static int accel_fd[INS_MAX_INSTANCES];
static int gyro_fd[INS_MAX_INSTANCES];
static uint32_t total_samples[INS_MAX_INSTANCES];
static uint64_t last_accel_timestamp[INS_MAX_INSTANCES];
static uint64_t last_gyro_timestamp[INS_MAX_INSTANCES];
static uint32_t accel_deltat_min[INS_MAX_INSTANCES];
static uint32_t accel_deltat_max[INS_MAX_INSTANCES];
static uint32_t gyro_deltat_min[INS_MAX_INSTANCES];
static uint32_t gyro_deltat_max[INS_MAX_INSTANCES];
static DataFlash_File DataFlash("/fs/microsd/VIBTEST");

static const struct LogStructure log_structure[] = {
    LOG_COMMON_STRUCTURES,
    LOG_EXTRA_STRUCTURES
};

void setup(void)
{
    for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
        char accel_path[] = ACCEL_BASE_DEVICE_PATH "n";
        char gyro_path[] = GYRO_BASE_DEVICE_PATH "n";
        accel_path[strlen(accel_path)-1] = '0'+i;
        gyro_path[strlen(gyro_path)-1] = '0'+i;
        accel_fd[i] = open(accel_path, O_RDONLY);
        gyro_fd[i] = open(gyro_path, O_RDONLY);
    }
    if (accel_fd[0] == -1 || gyro_fd[0] == -1) {
        AP_HAL::panic("Failed to open accel/gyro 0");
    }

    ioctl(gyro_fd[0], SENSORIOCSPOLLRATE, 1000);
    ioctl(gyro_fd[0], GYROIOCSLOWPASS, 0);
    ioctl(gyro_fd[0], GYROIOCSHWLOWPASS, 256);
    ioctl(gyro_fd[0], GYROIOCSSAMPLERATE, 1000);
    ioctl(gyro_fd[0], SENSORIOCSQUEUEDEPTH, 100);

    ioctl(gyro_fd[1], SENSORIOCSPOLLRATE, 800);
    ioctl(gyro_fd[1], GYROIOCSLOWPASS, 0);
    ioctl(gyro_fd[1], GYROIOCSHWLOWPASS, 100);
    ioctl(gyro_fd[1], GYROIOCSSAMPLERATE, 800);
    ioctl(gyro_fd[1], SENSORIOCSQUEUEDEPTH, 100);

    ioctl(accel_fd[0], SENSORIOCSPOLLRATE, 1000);
    ioctl(accel_fd[0], ACCELIOCSLOWPASS, 0);
    ioctl(accel_fd[0], ACCELIOCSRANGE, 16);
    ioctl(accel_fd[0], ACCELIOCSHWLOWPASS, 256);
    ioctl(accel_fd[0], ACCELIOCSSAMPLERATE, 1000);
    ioctl(accel_fd[0], SENSORIOCSQUEUEDEPTH, 100);

    ioctl(accel_fd[1], SENSORIOCSPOLLRATE, 1600);
    ioctl(accel_fd[1], ACCELIOCSLOWPASS, 0);
    ioctl(accel_fd[1], ACCELIOCSRANGE, 16);
    ioctl(accel_fd[1], ACCELIOCSHWLOWPASS, 194);
    ioctl(accel_fd[1], ACCELIOCSSAMPLERATE, 1600);
    ioctl(accel_fd[1], SENSORIOCSQUEUEDEPTH, 100);

    DataFlash.Init(log_structure, ARRAY_SIZE(log_structure));
    DataFlash.StartNewLog();
}

void loop(void)
{
    bool got_sample = false;
    static uint32_t last_print;
    do {
        got_sample = false;
        for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
            struct accel_report	accel_report;
            struct gyro_report	gyro_report;
            
            if (accel_fd[i] != -1 && ::read(accel_fd[i], &accel_report, sizeof(accel_report)) == 
                sizeof(accel_report) &&
                accel_report.timestamp != last_accel_timestamp[i]) {        
                uint32_t deltat = accel_report.timestamp - last_accel_timestamp[i];
                if (deltat > accel_deltat_max[i]) {
                    accel_deltat_max[i] = deltat;
                }
                if (accel_deltat_min[i] == 0 || deltat < accel_deltat_max[i]) {
                    accel_deltat_min[i] = deltat;
                }
                last_accel_timestamp[i] = accel_report.timestamp;

                struct log_ACCEL pkt = {
                    LOG_PACKET_HEADER_INIT((uint8_t)(LOG_ACC1_MSG+i)),
                    time_us   : AP_HAL::micros64(),
                    sample_us : accel_report.timestamp,
                    AccX      : accel_report.x,
                    AccY      : accel_report.y,
                    AccZ      : accel_report.z
                };
                DataFlash.WriteBlock(&pkt, sizeof(pkt));
                got_sample = true;
                total_samples[i]++;
            }
            if (gyro_fd[i] != -1 && ::read(gyro_fd[i], &gyro_report, sizeof(gyro_report)) == 
                sizeof(gyro_report) &&
                gyro_report.timestamp != last_gyro_timestamp[i]) {        
                uint32_t deltat = gyro_report.timestamp - last_gyro_timestamp[i];
                if (deltat > gyro_deltat_max[i]) {
                    gyro_deltat_max[i] = deltat;
                }
                if (gyro_deltat_min[i] == 0 || deltat < gyro_deltat_max[i]) {
                    gyro_deltat_min[i] = deltat;
                }
                last_gyro_timestamp[i] = gyro_report.timestamp;

                struct log_GYRO pkt = {
                    LOG_PACKET_HEADER_INIT((uint8_t)(LOG_GYR1_MSG+i)),
                    time_us   : AP_HAL::micros64(),
                    sample_us : gyro_report.timestamp,
                    GyrX      : gyro_report.x,
                    GyrY      : gyro_report.y,
                    GyrZ      : gyro_report.z
                };
                DataFlash.WriteBlock(&pkt, sizeof(pkt));
                got_sample = true;
                total_samples[i]++;
            }
        }
        if (got_sample) {
            if (total_samples[0] % 2000 == 0 && last_print != total_samples[0]) {
                last_print = total_samples[0];
                hal.console->printf("t=%lu total_samples=%lu/%lu/%lu adt=%u:%u/%u:%u/%u:%u gdt=%u:%u/%u:%u/%u:%u\n",
                                    (unsigned long)AP_HAL::millis(), 
                                    (unsigned long)total_samples[0], 
                                    (unsigned long)total_samples[1],
                                    (unsigned long)total_samples[2],
                                    accel_deltat_min[0], 
                                    accel_deltat_max[0], 
                                    accel_deltat_min[1], 
                                    accel_deltat_max[1], 
                                    accel_deltat_min[2], accel_deltat_max[2], 
                                    gyro_deltat_min[0], gyro_deltat_max[0], 
                                    gyro_deltat_min[1], gyro_deltat_max[1], 
                                    gyro_deltat_min[2], gyro_deltat_max[2]);
#if 0
                ::printf("t=%lu total_samples=%lu/%lu/%lu adt=%u:%u/%u:%u/%u:%u gdt=%u:%u/%u:%u/%u:%u\n",
                         AP_HAL::millis(), 
                         total_samples[0], total_samples[1],total_samples[2],
                         accel_deltat_min[0], accel_deltat_max[0], 
                         accel_deltat_min[1], accel_deltat_max[1], 
                         accel_deltat_min[2], accel_deltat_max[2], 
                         gyro_deltat_min[0], gyro_deltat_max[0], 
                         gyro_deltat_min[1], gyro_deltat_max[1], 
                         gyro_deltat_min[2], gyro_deltat_max[2]);
#endif

                memset(accel_deltat_min, 0, sizeof(accel_deltat_min));
                memset(accel_deltat_max, 0, sizeof(accel_deltat_max));
                memset(gyro_deltat_min, 0, sizeof(gyro_deltat_min));
                memset(gyro_deltat_max, 0, sizeof(gyro_deltat_max));
            }
        }
    } while (got_sample);
    hal.scheduler->delay_microseconds(100);
}

#else
const AP_HAL::HAL& hal = AP_HAL::get_HAL();
void setup() {}
void loop() {}
#endif // CONFIG_HAL_BOARD

AP_HAL_MAIN();
