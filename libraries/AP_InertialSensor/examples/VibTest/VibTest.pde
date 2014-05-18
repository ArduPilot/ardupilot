// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//
// test harness for vibration testing
//

#include <stdarg.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Empty.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>
#include <AP_Notify.h>
#include <AP_GPS.h>
#include <AP_Baro.h>
#include <Filter.h>
#include <DataFlash.h>
#include <GCS_MAVLink.h>
#include <AP_Mission.h>
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_Notify.h>
#include <AP_NavEKF.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include <drivers/drv_accel.h>
#include <drivers/drv_hrt.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;


#define NUM_ACCELS 2

static int accel_fd[NUM_ACCELS];
static uint32_t total_samples[NUM_ACCELS];
static uint64_t last_accel_timestamp[NUM_ACCELS];
static DataFlash_File DataFlash("/fs/microsd/VIBTEST");

#define LOG_ACCEL0_MSG 215

struct PACKED log_Accel {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    float X, Y, Z;
};

static const struct LogStructure log_structure[] PROGMEM = {
    LOG_COMMON_STRUCTURES,
    { LOG_ACCEL0_MSG, sizeof(log_Accel),       
      "ACC0", "Ifff",        "TimeUS,X,Y,Z" },
    { LOG_ACCEL0_MSG+1, sizeof(log_Accel),       
      "ACC1", "Ifff",        "TimeUS,X,Y,Z" }
};

void setup(void)
{
    accel_fd[0] = open(ACCEL_DEVICE_PATH, O_RDONLY);
    accel_fd[1] = open(ACCEL_DEVICE_PATH "1", O_RDONLY);

    for (uint8_t i=0; i<NUM_ACCELS; i++) {
        if (accel_fd[i] == -1) {
            hal.console->printf("Failed to open accel[%u]\n", (unsigned)i);
            hal.scheduler->panic("Failed to open accel");
        }
        // disable software filtering
        ioctl(accel_fd[i], ACCELIOCSLOWPASS, 0);

        // max queue depth
        ioctl(accel_fd[i], SENSORIOCSQUEUEDEPTH, 100);
    }

    DataFlash.Init(log_structure, sizeof(log_structure)/sizeof(log_structure[0]));
    DataFlash.StartNewLog();
}

void loop(void)
{
    for (uint8_t i=0; i<NUM_ACCELS; i++) {
        struct accel_report	accel_report;
        while (::read(accel_fd[i], &accel_report, sizeof(accel_report)) == 
               sizeof(accel_report) &&
               accel_report.timestamp != last_accel_timestamp[i]) {        
            last_accel_timestamp[i] = accel_report.timestamp;

            struct log_Accel pkt = {
                LOG_PACKET_HEADER_INIT((uint8_t)(LOG_ACCEL0_MSG+i)),
                timestamp : (uint32_t)accel_report.timestamp,
                X         : accel_report.x,
                Y         : accel_report.y,
                Z         : accel_report.z
            };
            DataFlash.WriteBlock(&pkt, sizeof(pkt));
            total_samples[i]++;
            if (total_samples[i] % 2000 == 0) {
                hal.console->printf("t=%lu total_samples=%lu/%lu\n",
                                    hal.scheduler->millis(), 
                                    total_samples[0], total_samples[1]);
            }
        }
    }
    hal.scheduler->delay(1);
}

#else
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
void setup() {}
void loop() {}
#endif // CONFIG_HAL_BOARD

AP_HAL_MAIN();
