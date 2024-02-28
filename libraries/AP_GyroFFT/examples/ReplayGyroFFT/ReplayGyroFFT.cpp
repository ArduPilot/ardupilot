#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_Arming/AP_Arming.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_GyroFFT/AP_GyroFFT.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Arming/AP_Arming.h>
#include <SITL/SITL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
const AP_HAL::HAL &hal = AP_HAL::get_HAL();

static const uint32_t LOOP_RATE_HZ = 400;
static const uint32_t LOOP_DELTA_US = 1000000 / LOOP_RATE_HZ;
static const uint32_t RUN_TIME = 120;   // 2 mins
static const uint32_t LOOP_ITERATIONS = LOOP_RATE_HZ * RUN_TIME;

void setup();
void loop();

static AP_SerialManager serial_manager;
static AP_BoardConfig board_config;
static AP_InertialSensor ins;
static AP_Baro baro;
AP_Int32 logger_bitmask;
static AP_Logger logger;
#if HAL_EXTERNAL_AHRS_ENABLED
static AP_ExternalAHRS external_ahrs;
#endif
static SITL::SIM sitl;
static AP_Scheduler scheduler;

// create fake gcs object
GCS_Dummy _gcs;

const struct LogStructure log_structure[] = {
    LOG_COMMON_STRUCTURES
};

const AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};

class Arming : public AP_Arming {
public:
    Arming() : AP_Arming() {}
    bool arm(AP_Arming::Method method, bool do_arming_checks=true) override {
        armed = true;
        return true;
    }
};

static Arming arming;

class ReplayGyroFFT {
public:
    void init() {
        fft._enable.set(1);             // FFT_ENABLE
        fft._window_size.set(64);       // FFT_WINDOW_SIZE
        fft._snr_threshold_db.set(10);  // FFT_SNR_REF
        fft._fft_min_hz.set(50);        // FFT_MINHZ
        fft._fft_max_hz.set(450);       // FFT_MAXHZ

        fft.init(LOOP_RATE_HZ);
        fft.update_parameters();
    }

    void loop() {
        fft.sample_gyros();
        fft.update();
        // calibrate the FFT
        uint32_t now = AP_HAL::millis();
        if (!arming.is_armed()) {
            char buf[32];
            if (!fft.pre_arm_check(buf, 32)) {
                if (now - last_output_ms > 1000) {
                    hal.console->printf("%s\n", buf);
                    last_output_ms = now;
                }
            } else {
                logger.PrepForArming();
                arming.arm(AP_Arming::Method::RUDDER);
                logger.set_vehicle_armed(true);
                // apply throttle values to motors to make sure the fake IMU generates energetic enough data
                for (uint8_t i=0; i<4; i++) {
                    hal.rcout->write(i, 1500);
                }
            }
        } else {
            if (now - last_output_ms > 1000) {
                hal.console->printf(".");
                last_output_ms = now;
            }
        }
        fft.write_log_messages();
    }
    AP_GyroFFT fft;
    uint32_t last_output_ms;
};

static ReplayGyroFFT replay;

void setup()
{
    hal.console->printf("ReplayGyroFFT\n");
    board_config.init();   
    serial_manager.init();

    const bool generate = false;
    if (generate) {
        sitl.vibe_freq.set(Vector3f(250,250,250));  // SIM_VIB_FREQ
        sitl.drift_speed.set(0);    // SIM_DRIFT_SPEED
        sitl.drift_time.set(0);     // SIM_DRIFT_TIME
        sitl.gyro_noise[0].set(20); // SIM_GYR1_RND
    } else {
        sitl.speedup.set(100);      // SIM_SPEEDUP
        sitl.gyro_file_rw.set(SITL::SIM::INSFileMode::INS_FILE_READ_STOP_ON_EOF);   // SIM_GYR_FILE_RW
    }
    logger_bitmask.set(128);    // IMU
    logger.init(logger_bitmask, log_structure, ARRAY_SIZE(log_structure));
    ins.init(LOOP_RATE_HZ);
    baro.init();

    replay.init();
}

static uint32_t loop_iter = LOOP_ITERATIONS;

void loop()
{
    if (!hal.console->is_initialized()) {
        return;
    }

    ins.wait_for_sample();
    uint32_t sample_time_us = AP_HAL::micros();

    ins.update();
    ins.periodic();
    logger.periodic_tasks();
    ins.Write_IMU();
    replay.loop();

    uint32_t elapsed = AP_HAL::micros() - sample_time_us;
    if (elapsed < LOOP_DELTA_US) {
        hal.scheduler->delay_microseconds(LOOP_DELTA_US - elapsed);
    }

    if (sitl.gyro_file_rw != SITL::SIM::INSFileMode::INS_FILE_READ_STOP_ON_EOF && loop_iter-- == 0) {
        hal.console->printf("\n");
        exit(0);
    }
}

AP_HAL_MAIN();

#else

#include <stdio.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static void loop() { }
static void setup()
{
    printf("Board not currently supported\n");
}

AP_HAL_MAIN();

#endif
