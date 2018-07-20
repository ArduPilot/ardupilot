#pragma once

#include <atomic>

#include "AP_HAL_Linux.h"

#define LINUX_RC_INPUT_NUM_CHANNELS 16

namespace Linux {

class RCInput : public AP_HAL::RCInput {
public:
    RCInput();

    static RCInput *from(AP_HAL::RCInput *rcinput) {
        return static_cast<RCInput*>(rcinput);
    }

    virtual void init();
    bool new_input();
    uint8_t num_channels();
    void set_num_channels(uint8_t num);
    uint16_t read(uint8_t ch);
    uint8_t read(uint16_t* periods, uint8_t len);

    int16_t get_rssi(void) override {
        return _rssi;
    }
    
    // default empty _timer_tick, this is overridden by board
    // specific implementations
    virtual void _timer_tick() {}

    // add some DSM input bytes, for RCInput over a serial port
    bool add_dsm_input(const uint8_t *bytes, size_t nbytes);

    // add some SBUS input bytes, for RCInput over a serial port
    void add_sbus_input(const uint8_t *bytes, size_t nbytes);

    // add some SUMD input bytes, for RCInput over a serial port
    bool add_sumd_input(const uint8_t *bytes, size_t nbytes);

    // add some st24 input bytes, for RCInput over a serial port
    bool add_st24_input(const uint8_t *bytes, size_t nbytes);

    // add some srxl input bytes, for RCInput over a serial port
    bool add_srxl_input(const uint8_t *bytes, size_t nbytes);

protected:
    void _process_rc_pulse(uint16_t width_s0, uint16_t width_s1);
    void _update_periods(uint16_t *periods, uint8_t len);

    std::atomic<unsigned int> rc_input_count;
    std::atomic<unsigned int> last_rc_input_count;

    uint16_t _pwm_values[LINUX_RC_INPUT_NUM_CHANNELS];
    uint8_t  _num_channels;

    void _process_ppmsum_pulse(uint16_t width);
    void _process_sbus_pulse(uint16_t width_s0, uint16_t width_s1);
    void _process_dsm_pulse(uint16_t width_s0, uint16_t width_s1);
    void _process_pwm_pulse(uint16_t channel, uint16_t width_s0, uint16_t width_s1);

    // state of ppm decoder
    struct {
        int8_t _channel_counter;
        uint16_t _pulse_capt[LINUX_RC_INPUT_NUM_CHANNELS];
    } ppm_state;

    // state of SBUS bit decoder
    struct {
	uint16_t bytes[25]; // including start bit, parity and stop bits
	uint16_t bit_ofs;
    } sbus_state;

    // state of DSM decoder
    struct {
        uint16_t bytes[16]; // including start bit and stop bit
        uint16_t bit_ofs;
    } dsm_state;

    // state of add_dsm_input
    struct {
        uint8_t frame[16];
        uint8_t partial_frame_count;
        uint32_t last_input_ms;
    } dsm;

    // state of add_sbus_input
    struct {
        uint8_t frame[25];
        uint8_t partial_frame_count;
        uint32_t last_input_ms;
    } sbus;

    int16_t _rssi = -1;
};

}
