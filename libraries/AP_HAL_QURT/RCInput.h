#pragma once

#include "AP_HAL_QURT.h"

#define QURT_RC_INPUT_NUM_CHANNELS 16

class QURT::RCInput : public AP_HAL::RCInput {
public:
    RCInput(const char *device_path);

    static RCInput *from(AP_HAL::RCInput *rcinput) {
        return static_cast<RCInput*>(rcinput);
    }

    void set_device_path(const char *path) {
        device_path = path;
    }
    
    void init();
    bool new_input();
    uint8_t num_channels();
    uint16_t read(uint8_t ch);
    uint8_t read(uint16_t* periods, uint8_t len);

    bool set_overrides(int16_t *overrides, uint8_t len);
    bool set_override(uint8_t channel, int16_t override);
    void clear_overrides();

    void read_callback(char *buf, size_t size);

 private:
    volatile bool new_rc_input;

    uint16_t _pwm_values[QURT_RC_INPUT_NUM_CHANNELS];    
    uint8_t  _num_channels;

    /* override state */
    uint16_t _override[QURT_RC_INPUT_NUM_CHANNELS];

    // add some DSM input bytes, for RCInput over a serial port
    void add_dsm_input(const uint8_t *bytes, size_t nbytes);
        
    const char *device_path;
    int32_t fd = -1;
    
    // state of add_dsm_input
    struct {
        uint8_t frame[16];
        uint8_t partial_frame_count;
        uint32_t last_input_ms;
    } dsm;
};

