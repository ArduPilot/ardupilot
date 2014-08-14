
#ifndef __AP_HAL_LINUX_RCINPUT_H__
#define __AP_HAL_LINUX_RCINPUT_H__

#include <AP_HAL_Linux.h>

#define LINUX_RC_INPUT_NUM_CHANNELS 8
#define PRUSS_SHAREDRAM_BASE        0x4a312000
#define NUM_RING_ENTRIES            200


class Linux::LinuxRCInput : public AP_HAL::RCInput {
public:
    LinuxRCInput();
    void init(void* machtnichts);
    bool new_input();
    uint8_t num_channels();
    uint16_t read(uint8_t ch);
    uint8_t read(uint16_t* periods, uint8_t len);

    bool set_overrides(int16_t *overrides, uint8_t len);
    bool set_override(uint8_t channel, int16_t override);
    void clear_overrides();
    
    void _timer_tick(void);
    
 private:
    bool new_rc_input;
    
    uint16_t _pulse_capt[LINUX_RC_INPUT_NUM_CHANNELS];
    uint8_t  _num_channels;
    /* override state */
    uint16_t _override[LINUX_RC_INPUT_NUM_CHANNELS];
    struct ring_buffer {
        volatile uint16_t ring_head;
        volatile uint16_t ring_tail;
        struct __attribute__((__packed__)) {
               uint16_t pin_value;
               uint16_t delta_t;
        } buffer[NUM_RING_ENTRIES];
    };
    volatile struct ring_buffer *ring_buffer;
};

#endif // __AP_HAL_LINUX_RCINPUT_H__
