
#ifndef __AP_HAL_LINUX_RCINPUT_H__
#define __AP_HAL_LINUX_RCINPUT_H__

#include "AP_HAL_Linux.h"

#define LINUX_RC_INPUT_NUM_CHANNELS 16

class Linux::RCInput : public AP_HAL::RCInput {
public:
    RCInput();

    static RCInput *from(AP_HAL::RCInput *rcinput) {
        return static_cast<RCInput*>(rcinput);
    }

    virtual void init(void* machtnichts);
    bool new_input();
    uint8_t num_channels();
    uint16_t read(uint8_t ch);
    uint8_t read(uint16_t* periods, uint8_t len);

    bool set_overrides(int16_t *overrides, uint8_t len);
    bool set_override(uint8_t channel, int16_t override);
    void clear_overrides();

    // default empty _timer_tick, this is overridden by board
    // specific implementations
    virtual void _timer_tick() {}

 protected:
    void _process_rc_pulse(uint16_t width_s0, uint16_t width_s1);
    void _update_periods(uint16_t *periods, uint8_t len);

 private:
    volatile bool new_rc_input;

    uint16_t _pwm_values[LINUX_RC_INPUT_NUM_CHANNELS];    
    uint8_t  _num_channels;

    void _process_ppmsum_pulse(uint16_t width);
    void _process_sbus_pulse(uint16_t width_s0, uint16_t width_s1);
    void _process_dsm_pulse(uint16_t width_s0, uint16_t width_s1);

    /* override state */
    uint16_t _override[LINUX_RC_INPUT_NUM_CHANNELS];

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
};

#include "RCInput_PRU.h"
#include "RCInput_ZYNQ.h"

#endif // __AP_HAL_LINUX_RCINPUT_H__
