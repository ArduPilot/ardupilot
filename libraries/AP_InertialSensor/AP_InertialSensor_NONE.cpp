#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor_NONE.h"
#include <SITL/SITL.h>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32 || CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR


static float sim_rand_float(void)
{
    return ((((unsigned)random()) % 2000000) - 1.0e6) / 1.0e6;
}

const extern AP_HAL::HAL& hal;

AP_InertialSensor_NONE::AP_InertialSensor_NONE(AP_InertialSensor &imu, const uint16_t sample_rates[]) :
    AP_InertialSensor_Backend(imu),
    gyro_sample_hz(sample_rates[0]),
    accel_sample_hz(sample_rates[1])
{
}

/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_NONE::detect(AP_InertialSensor &_imu, const uint16_t sample_rates[])
{
    AP_InertialSensor_NONE *sensor = NEW_NOTHROW AP_InertialSensor_NONE(_imu, sample_rates);
    if (sensor == nullptr) {

        return nullptr;
    }
    if (!sensor->init_sensor()) {
        delete sensor;

        return nullptr;
    }
    return sensor;
}


bool AP_InertialSensor_NONE::init_sensor(void)
{
  
    return true;
}



void AP_InertialSensor_NONE::accumulate()
{
    // generate samples on demand so wait_for_sample() doesn't block
    // when the timer thread hasn't run yet (e.g. during gyro cal)
    timer_update();
}


/*
  generate an accelerometer sample

  SLIMMED 2026-08-14: the original carried SITL-derived motor/vibration
  theater - per subsample, ~15 sinf() calls plus cross products - all
  scaled by 0.01-magnitude constants, so the output was indistinguishable
  from constants+noise. On ESP32-S3 (240 MHz Xtensa) that cost measured
  2.1 ms per 1 kHz timer tick, >100% of the timer thread's budget: the
  thread (PREEMPT(2)) pinned the core and the main loop (8) never ran -
  thread state "queued" forever, monitor reset, boot loop. This backend
  exists so a board with no IMU boots and loops; constant-plus-noise
  samples serve that purpose at ~1% of the cost.
 */
void AP_InertialSensor_NONE::generate_accel()
{
    Vector3f accel_accum;
    uint8_t nsamples = enable_fast_sampling(accel_instance) ? 4 : 1;
    for (uint8_t j = 0; j < nsamples; j++) {
        // constant bias plus ~2-bit sensor noise, same magnitudes the
        // original converged to after all its cancelling terms
        const float accel_noise = 0.01f;
        Vector3f accel(0.01f + accel_noise * sim_rand_float(),
                       0.01f + accel_noise * sim_rand_float(),
                       0.01f + accel_noise * sim_rand_float());

        _notify_new_accel_sensor_rate_sample(accel_instance, accel);

        accel_accum += accel;
    }

    accel_accum /= nsamples;
    _rotate_and_correct_accel(accel_instance, accel_accum);
    _notify_new_accel_raw_sample(accel_instance, accel_accum, AP_HAL::micros64());

    _publish_temperature(accel_instance, 23);
}

/*
  generate a gyro sample

  SLIMMED 2026-08-14 for the same reason as generate_accel() above - the
  original's per-subsample sinf() motor loops and double-precision drift
  math cost more than the whole 1 kHz timer budget on ESP32-S3 while
  producing 0.01-magnitude noise either way.
 */
void AP_InertialSensor_NONE::generate_gyro()
{
    Vector3f gyro_accum;
    uint8_t nsamples = enable_fast_sampling(gyro_instance) ? 8 : 1;

    for (uint8_t j = 0; j < nsamples; j++) {
        // constant bias plus sub-bit sensor noise
        const float gyro_noise = radians(0.04f);
        Vector3f gyro(radians(0.01f) + gyro_noise * sim_rand_float(),
                      radians(0.01f) + gyro_noise * sim_rand_float(),
                      radians(0.01f) + gyro_noise * sim_rand_float());

        gyro_accum += gyro;
        _notify_new_gyro_sensor_rate_sample(gyro_instance, gyro);
    }
    gyro_accum /= nsamples;
    _rotate_and_correct_gyro(gyro_instance, gyro_accum);
    _notify_new_gyro_raw_sample(gyro_instance, gyro_accum, AP_HAL::micros64());
}

void AP_InertialSensor_NONE::timer_update(void)
{
    uint64_t now = AP_HAL::micros64();

    if (now >= next_accel_sample) {
        {
            generate_accel();
            if (next_accel_sample == 0) {
                next_accel_sample = now + 1000000UL / accel_sample_hz;
            } else {
                while (now >= next_accel_sample) {
                    next_accel_sample += 1000000UL / accel_sample_hz;
                }
            }
        }
    }
    if (now >= next_gyro_sample) {
        {
            generate_gyro();
            if (next_gyro_sample == 0) {
                next_gyro_sample = now + 1000000UL / gyro_sample_hz;
            } else {
                while (now >= next_gyro_sample) {
                    next_gyro_sample += 1000000UL / gyro_sample_hz;
                }
            }
        }
    }
}

float AP_InertialSensor_NONE::gyro_drift(void)
{
 
    double period  = 0.01 * 2;
    double minutes = fmod(AP_HAL::micros64() / 60.0e6, period);
    if (minutes < period/2) {
        return minutes * radians(0.01);
    }
    return (period - minutes) * radians(0.01);
}


bool AP_InertialSensor_NONE::update(void) 
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

uint8_t AP_InertialSensor_NONE::bus_id = 0;

void AP_InertialSensor_NONE::start()
{
    if (!_imu.register_gyro(gyro_instance, gyro_sample_hz,
                            AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SITL, bus_id, 1, DEVTYPE_SITL)) ||
        !_imu.register_accel(accel_instance, accel_sample_hz,
                             AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SITL, bus_id, 2, DEVTYPE_SITL))) {
        return;
    }
    bus_id++;
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_NONE::timer_update, void));

}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_ESP32
