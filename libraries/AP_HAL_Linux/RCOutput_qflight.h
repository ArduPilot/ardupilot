#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_Linux.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_QFLIGHT

class Linux::RCOutput_QFLIGHT : public AP_HAL::RCOutput {
public:
    void init();
    void set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void enable_ch(uint8_t ch);
    void disable_ch(uint8_t ch);
    void write(uint8_t ch, uint16_t period_us);
    uint16_t read(uint8_t ch);
    void read(uint16_t *period_us, uint8_t len);
    void set_device_path(const char *device);

private:
    const char *device = nullptr;
    const uint32_t baudrate = 115200;    
    static const uint8_t channel_count = 4;
    
    int32_t fd = -1;
    uint16_t enable_mask;
    uint16_t period[channel_count];
    volatile bool need_write;
    
    void timer_update(void);

    void check_rc_in(void);
    
    uint32_t last_read_check_ms;
    struct PACKED rcin_frame {
        uint8_t magic;
        uint16_t rcin[8];
        uint16_t crc;
    };
    union {
        struct rcin_frame rcin;
        uint8_t bytes[19];
    } rcu;
    uint8_t nrcin_bytes;
};

#endif // CONFIG_HAL_BOARD_SUBTYPE
