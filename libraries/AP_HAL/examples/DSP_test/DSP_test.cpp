#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include "GyroFrame.h"

#if HAL_WITH_DSP
const AP_HAL::HAL &hal = AP_HAL::get_HAL();

static const uint16_t WINDOW_SIZE = 128;
static const uint16_t FRAME_SIZE = 1024;
static const float max_hz = 350;
static const float attenuation_power_db = 15;
static const float frequency1 = 120;
static const float frequency2 = 50;
static const float frequency3 = 350;
static float attenuation_cutoff;
static FloatBuffer fft_window {WINDOW_SIZE};

static const uint16_t last_bin = MIN(ceilf(max_hz / ((float)SAMPLE_RATE/ WINDOW_SIZE)), WINDOW_SIZE/2);

static AP_HAL::DSP::FFTWindowState* fft;

void setup();
void loop();
void update();
void do_fft(const float* data);

static AP_SerialManager serial_manager;
static AP_BoardConfig board_config;
static AP_InertialSensor ins;
AP_Int32 logger_bitmask;
static AP_Logger logger;

class DummyVehicle {
public:
};

class DSPTest : public AP_HAL::DSP {
public:
    virtual FFTWindowState* fft_init(uint16_t w, uint16_t sample_rate, uint8_t sliding_window_size) override { return nullptr; }
    virtual void fft_start(FFTWindowState* state, FloatBuffer& samples, uint16_t advance) override {}
    virtual uint16_t fft_analyse(FFTWindowState* state, uint16_t start_bin, uint16_t end_bin, float noise_att_cutoff) override { return 0; }
protected:
    virtual void vector_max_float(const float* vin, uint16_t len, float* maxValue, uint16_t* maxIndex) const override {}
    virtual void vector_scale_float(const float* vin, float scale, float* vout, uint16_t len) const override {}
    virtual float vector_mean_float(const float* vin, uint16_t len) const override { return 0.0f; };
    virtual void vector_add_float(const float* vin1, const float* vin2, float* vout, uint16_t len) const override {}
public:
    void run_tests();
} dsptest;

//static DummyVehicle vehicle;
// create fake gcs object
GCS_Dummy _gcs;

uint32_t frame_num = 0;

void setup()
{
    hal.console->printf("DSP test\n");
    board_config.init();   
    serial_manager.init();
    fft = hal.dsp->fft_init(WINDOW_SIZE, SAMPLE_RATE);
    attenuation_cutoff = powf(10.0f, -attenuation_power_db / 10.0f);

    for(uint16_t i = 0; i < WINDOW_SIZE; i++) {
        float sample = sinf(2.0f * M_PI * frequency1 * i / SAMPLE_RATE) * radians(20) * 2000;
        sample += sinf(2.0f * M_PI * frequency2 * i / SAMPLE_RATE) * radians(10) * 2000;
        sample += sinf(2.0f * M_PI * frequency3 * i / SAMPLE_RATE) * radians(10) * 2000;
        fft_window.push(sample);
    }

    dsptest.run_tests();

}

void DSPTest::run_tests() {
    float vals[] = {1, 1, 1, 10, 10, 10, 1, 1, 1, 1};
    fastsmooth(vals, 10, 3); 
    for (int i=0; i < 10; i++) {
        hal.console->printf("%.f ", vals[i]);
    }
    hal.console->printf("\n");
    // fastsmooth([1 1 1 10 10 10 1 1 1 1],3) => [0 1 4 7 10 7 4 1 1 0]
}


void do_fft(const float* data)
{
    fft_window.push(data, WINDOW_SIZE);
    hal.dsp->fft_start(fft, fft_window, WINDOW_SIZE);
    uint16_t max_bin = hal.dsp->fft_analyse(fft, 1, last_bin, attenuation_cutoff);

    if (max_bin <= 0) {
        hal.console->printf("FFT: could not detect frequency %.1f\n", frequency1);
    }

    const float max_energy = fft->_freq_bins[fft->_peak_data[AP_HAL::DSP::CENTER]._bin];

    for (uint16_t i = 0; i < 32; i++) {
        const uint16_t height = uint16_t(roundf(80.0f * fft->_freq_bins[i] / max_energy));
        hal.console->printf("[%3.f]", i * fft->_bin_resolution);
        for (uint16_t j = 0; j < height; j++) {
            hal.console->printf("\u2588");
        }
        hal.console->printf("\n");
    }

    hal.console->printf("FFT: detected frequencies %.1f/%d/[%.1f-%.1f] %.1f/%d/[%.1f-%.1f] %.1f/%d/[%.1f-%.1f]\n",
        fft->_peak_data[AP_HAL::DSP::CENTER]._freq_hz,
        fft->_peak_data[AP_HAL::DSP::CENTER]._bin,
        (fft->_peak_data[AP_HAL::DSP::CENTER]._bin - 0.5) * fft->_bin_resolution,
        (fft->_peak_data[AP_HAL::DSP::CENTER]._bin + 0.5) * fft->_bin_resolution,
        fft->_peak_data[AP_HAL::DSP::LOWER_SHOULDER]._freq_hz,
        fft->_peak_data[AP_HAL::DSP::LOWER_SHOULDER]._bin,
        (fft->_peak_data[AP_HAL::DSP::LOWER_SHOULDER]._bin - 0.5) * fft->_bin_resolution,
        (fft->_peak_data[AP_HAL::DSP::LOWER_SHOULDER]._bin + 0.5) * fft->_bin_resolution,
        fft->_peak_data[AP_HAL::DSP::UPPER_SHOULDER]._freq_hz,
        fft->_peak_data[AP_HAL::DSP::UPPER_SHOULDER]._bin,
        (fft->_peak_data[AP_HAL::DSP::UPPER_SHOULDER]._bin - 0.5) * fft->_bin_resolution,
        (fft->_peak_data[AP_HAL::DSP::UPPER_SHOULDER]._bin + 0.5) * fft->_bin_resolution);
}

void update()
{
    for (uint16_t i = 0; i < FRAME_SIZE / WINDOW_SIZE; i++) {
        do_fft(&gyro_frames[frame_num].x[i * WINDOW_SIZE]);
    }
    if (++frame_num > NUM_FRAMES) {
        exit(0);
    };
}

void loop()
{
    if (!hal.console->is_initialized()) {
        return;
    }
    uint32_t reference_time, run_time;

    hal.console->printf("--------------------\n");

    reference_time = AP_HAL::micros();
    update();
    run_time = AP_HAL::micros() - reference_time;
    if (run_time > 1000) {
        hal.console->printf("ran for %d\n", unsigned(run_time));
    }

    // delay before next display
    hal.scheduler->delay(1e3); // 1 second
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
