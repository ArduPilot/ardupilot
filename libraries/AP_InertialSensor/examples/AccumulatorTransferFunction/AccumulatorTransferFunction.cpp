// Example and MATLAB script for verifying the transfer function of the gyro accumulator
// as implemented in AP_InertialSensor_Backend::_notify_new_gyro_raw_sample

/* on Linux run with
    ./waf configure --board linux
    ./waf --targets examples/AccumulatorTransferFunction
    ./build/linux/examples/AccumulatorTransferFunction > test.csv
*/

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_InertialSensor/AP_InertialSensor_Backend.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Scheduler/AP_Scheduler.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  Thin test backend that exercises the real _notify_new_gyro_raw_sample path.

  sample_us=0 is passed so the function always uses dt = 1/sample_rate.
  After every N IMU samples the accumulator is published via _publish_gyro,
  mirroring the real system where the IMU runs faster than the EKF.
  The per-publish output is delta_angle (rad), the accumulated angle change
  over N IMU steps.  Its transfer function from gyro rate (rad/s) is:

    H(z) = (1/(2*sample_rate)) * (1+z^-1) * (1-z^-N) / (1-z^-1)

  where z is evaluated at the IMU sample rate (dt_imu = 1/sample_rate stays
  in the formula because the accumulator is not divided by dt on readout).

  The coning correction is zero for single-axis sinusoidal inputs (cross
  product of parallel X-axis vectors), so the output matches the analytic H(z).
*/
class TestBackend : public AP_InertialSensor_Backend
{
public:
    TestBackend(AP_InertialSensor &imu, float sample_rate_hz, uint16_t publish_every_n) :
        AP_InertialSensor_Backend(imu),
        _N(publish_every_n)
    {
        _imu.register_gyro(_instance, (uint16_t)sample_rate_hz, 1);
        gyro_instance = _instance;
    }

    bool update() override { return true; }

    // Reset the sensor rate estimator and publish counter between sweeps.
    void reset()
    {
        notify_gyro_fifo_reset(_instance);
        _sample_count = 0;
    }

    // Push one IMU sample. Returns true with rate_out set when N samples have
    // accumulated and the result is published to the EKF-facing fields.
    bool push_gyro(const Vector3f &gyro, Vector3f &rate_out)
    {
        _notify_new_gyro_raw_sample(_instance, gyro, 0);
        _sample_count += 1;

        if (_sample_count < _N) {
            return false;
        }
        // Accumulated enough samples, publish
        _sample_count = 0;

        // Publish: copies accumulator to _delta_angle/_delta_angle_dt and resets it.
        _publish_gyro(_instance, Vector3f{});

        Vector3f delta_angle;
        float dt;
        if (_imu.get_delta_angle(_instance, delta_angle, dt)) {
            rate_out = delta_angle;
            return true;
        }
        return false;
    }

private:
    uint16_t _N;
    uint16_t _sample_count = 0;
    uint8_t  _instance = 0;
};

static AP_InertialSensor ins;
static AP_BoardConfig BoardConfig;
static AP_Int32 log_bitmask;
static AP_Logger logger;
static AP_Baro baro;
static AP_Scheduler scheduler;

GCS_Dummy _gcs;

void setup();
void loop();
void sweep(TestBackend &backend, uint16_t num_output_samples, uint16_t max_freq,
           float sample_rate, float output_rate);

void loop() {}

void setup()
{
    hal.console->printf("Gyro Accumulator Transfer Function\n");
    hal.console->printf("Sweeping a range of frequencies in the form sin(2*pi*t*f)\n");

    // IMU runs at sample_rate; the EKF (publish) runs at output_rate.
    const float    sample_rate        = 4000.0;
    const float    output_rate        = 400.0;
    const uint16_t N                  = (uint16_t)(sample_rate / output_rate);
    const uint16_t num_output_samples = 1000;
    // Stay below the IMU Nyquist (sample_rate/2) to avoid aliasing inputs.
    const uint16_t max_freq           = (uint16_t)(sample_rate / 2) - 1;

    static TestBackend backend(ins, sample_rate, N);

    hal.console->printf("\n");
    hal.console->printf("GyroAccumulator\n");
    hal.console->printf("Sample rate: %.9f Hz\n", (double)sample_rate);
    hal.console->printf("Output rate: %.9f Hz\n", (double)output_rate);
    hal.console->printf("Combined FIR: H(z) = (1/(2*sample_rate))*(1+z^-1)*(1-z^-N)/(1-z^-1), N=%u\n", (unsigned)N);
    hal.console->printf("\n");

    sweep(backend, num_output_samples, max_freq, sample_rate, output_rate);

    hal.scheduler->delay(1000);
    exit(0);
}

void sweep(TestBackend &backend, uint16_t num_output_samples, uint16_t max_freq,
           float sample_rate, float output_rate)
{
    // Time axis is at the output (publish) rate.
    hal.console->printf("f(hz)");
    for (uint16_t i = 0; i < num_output_samples; i++) {
        hal.console->printf(", t = %.9f", (double)(i / output_rate));
    }
    hal.console->printf("\n");

    for (uint16_t f = 1; f <= max_freq; f++) {
        hal.console->printf("%u", (unsigned)f);

        backend.reset();

        uint32_t imu_idx = 0;
        for (uint16_t out_i = 0; out_i < num_output_samples; ) {
            const float t = imu_idx / sample_rate;
            const Vector3f gyro{sinf(M_2PI * t * f), 0.0f, 0.0f};
            Vector3f rate;
            if (backend.push_gyro(gyro, rate)) {
                hal.console->printf(", %+.9f", (double)rate.x);
                out_i++;
            }
            imu_idx++;
        }
        hal.console->printf("\n");

        // Avoid overflowing the print buffer.
        hal.scheduler->delay(100);
    }
}

AP_HAL_MAIN();
