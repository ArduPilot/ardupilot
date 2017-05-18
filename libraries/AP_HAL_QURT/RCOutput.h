#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_QURT.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_QURT

class QURT::RCOutput : public AP_HAL::RCOutput {
public:
    
    RCOutput(const char *_device_path) {
        device_path = _device_path;
    }

    void set_device_path(const char *path) {
        device_path = path;
    }

    void init();
    void set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void enable_ch(uint8_t ch);
    void disable_ch(uint8_t ch);
    void write(uint8_t ch, uint16_t period_us);
    uint16_t read(uint8_t ch);
    void read(uint16_t *period_us, uint8_t len);
    void cork(void) override;
    void push(void) override;

    void timer_update(void);
    
private:
    const char *device_path;
    const uint32_t baudrate = 115200;    
    static const uint8_t channel_count = 4;
    
    int fd = -1;
    uint16_t enable_mask;
    uint16_t period[channel_count];
    volatile bool need_write;
    bool corked;
};

#endif // CONFIG_HAL_BOARD
