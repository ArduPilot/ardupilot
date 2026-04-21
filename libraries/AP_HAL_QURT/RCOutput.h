#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_QURT.h"
#include <AP_ESC_Telem/AP_ESC_Telem.h>

class QURT::RCOutput : public AP_HAL::RCOutput, AP_ESC_Telem_Backend
{
public:
    friend class QURT::Util;

    void init() override;
    void set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t ch) override;
    void enable_ch(uint8_t ch) override;
    void disable_ch(uint8_t ch) override;
    void write(uint8_t ch, uint16_t period_us) override;
    uint16_t read(uint8_t ch) override;
    void read(uint16_t *period_us, uint8_t len) override;
    void cork(void) override;
    void push(void) override;

    float get_voltage(void) const
    {
        return esc_voltage;
    }
    float get_current(void) const
    {
        return esc_current;
    }

    /*
      force the safety switch on, disabling output from the ESCs/servos
     */
    bool force_safety_on(void) override { safety_on = true; return true; }

    /*
      force the safety switch off, enabling output from the ESCs/servos
     */
    void force_safety_off(void) override { safety_on = false; }

private:
    const uint32_t baudrate = 2000000;

    void send_receive(void);
    void check_response(void);
    void send_esc_packet(uint8_t type, uint8_t *data, uint16_t size);

    struct PACKED esc_response_v2 {
        uint8_t  id_state;     // bits 0:3 = state, bits 4:7 = ID

        uint16_t rpm;          // Current RPM of the motor
        uint8_t  cmd_counter;  // Number of commands received by the ESC
        uint8_t  power;        // Applied power [0..100]

        uint16_t voltage;      // Voltage measured by the ESC in mV
        int16_t  current;      // Current measured by the ESC in 8mA resolution
        int16_t  temperature;  // Temperature measured by the ESC in 0.01 degC resolution
    };

    struct PACKED esc_power_status {
        uint8_t  id;       //ESC Id (could be used as system ID elsewhere)
        uint16_t voltage;  //Input voltage (Millivolts)
        int16_t  current;  //Total Current (8mA resolution)
    };

    void handle_esc_feedback(const struct esc_response_v2 &pkt);
    void handle_power_status(const struct esc_power_status &pkt);

    int fd = -1;
    uint16_t enable_mask;
    static const uint8_t channel_count = 4;
    uint16_t period[channel_count];
    volatile bool need_write;
    bool corked;
    HAL_Semaphore mutex;
    uint8_t last_fb_idx;
    uint32_t last_fb_req_ms;

    float esc_voltage;
    float esc_current;

    // start with safety on, gets disabled by AP_BoardConfig
    bool safety_on = true;
};
