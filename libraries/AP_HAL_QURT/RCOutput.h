#pragma once

#include <string>
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
    bool force_safety_on(void) override
    {
        safety_on = true;
        return true;
    }

    /*
      force the safety switch off, enabling output from the ESCs/servos
     */
    void force_safety_off(void) override
    {
        safety_on = false;
    }

private:
    enum {
        IO_BAUDRATE = 921600,
        ESC_BAUDRATE = 2000000
    } baudrate;

    enum class HWType {
        UNKNOWN = 0,            // Unknown board type
        ESC = 1,                // ESC
        IO = 2,                 // IO board
    };
    HWType hw_type = HWType::UNKNOWN;

    void scan_for_hardware(void);
    void send_receive(void);
    void check_response(void);
    void send_pwm_output(void);
    void send_io_command(void);
    void send_esc_command(void);
    void send_packet(uint8_t type, uint8_t *data, uint16_t size);

    struct PACKED extended_version_info {
        uint8_t  id;
        uint16_t sw_version;
        uint16_t hw_version;
        uint8_t  unique_id[12];
        char     firmware_git_version[12];
        char     bootloader_git_version[12];
        uint16_t bootloader_version;
        uint16_t crc;
    };

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

    void handle_version_feedback(const struct extended_version_info &pkt);
    void handle_esc_feedback(const struct esc_response_v2 &pkt);
    void handle_power_status(const struct esc_power_status &pkt);

    std::string board_id_to_name(uint16_t board_id);

    int fd = -1;
    uint16_t enable_mask;
    static const uint8_t max_channel_count = 8;
    static const uint8_t esc_channel_count = 4;
    static const uint8_t io_channel_count = 8;
    uint16_t period[max_channel_count];
    uint16_t pwm_output[max_channel_count];
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
