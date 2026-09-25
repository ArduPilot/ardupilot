#pragma once

#include "elrs_config.h"

#include <AP_Radio/driver_sx1280.h>
#include <AP_ELRS/elrs_tracker.h>
#include <AP_ELRS/elrs_led.h>

#if AP_PERIPH_ELRS_ENABLED

// Worker-thread adapter for the experimental bench tracker.
class AP_ELRSReceiver
{
public:
    enum class Error : uint8_t {
        NONE,
        CONFIGURATION,
        UNBOUND,
        RADIO_INIT,
        IRQ,
        RADIO_CONFIG,
        THREAD,
        RADIO_IO,
    };
    void init(int32_t uid1, int32_t uid2, int16_t model_id, bool bind);
    void update();
    const char *error_message();
    bool binding_active() const
    {
        return _started && _binding;
    }
    bool read_bound_uid(uint8_t uid[ELRS::UID_LENGTH]);
    bool read_input(uint16_t values[ELRS::Tracker::OUTPUT_CHANNELS], uint8_t &count,
                    uint8_t &quality, bool &failsafe);

private:
#ifdef HAL_ELRS_LED_PIN
    ELRS::StatusLED _led;
    bool _led_on = false;
#endif
    AP_SX1280 _radio;
    ELRS::Tracker _tracker;
    uint32_t _frequency = 0;
    uint8_t _configured_rate = 0;
    bool _invert_iq = false;
    uint32_t _sync_word = 0;
    uint16_t _crc_seed = 0;
    void configure_radio();
    uint32_t _last_report_ms = 0;
    uint32_t _overruns = 0;
    uint32_t _last_output_rc = 0;
    ELRS::InputStatus _input_status; // main thread only
    bool _transmitting = false; // worker only
    uint32_t _tx_started_us = 0;
    uint32_t _tx_count = 0;
    uint32_t _tx_done = 0;
    uint32_t _tx_timeout = 0;
    bool _binding = false; // immutable after init
    bool _binding_received = false; // worker only
    uint8_t _binding_uid[ELRS::UID_LENGTH] {};
    bool _binding_consumed = false; // main thread only
    bool _started = false; // only accessed by the main thread
    struct Report {
        ELRS::Tracker::Stats stats;
        AP_SX1280::Diagnostics radio;
        ELRS::Tracker::State state = ELRS::Tracker::State::DISABLED;
        uint16_t seen = 0;
        uint16_t primary_channels[4] {};
        uint16_t output_channels[ELRS::Tracker::OUTPUT_CHANNELS] {};
        uint32_t last_input_us = 0;
        uint8_t output_count = 0;
        uint8_t rate = 21;
        uint32_t frame_sequence = 0;
        uint32_t output_expires_us = 0;
        ELRS::SwitchMode mode = ELRS::SwitchMode::WIDE;
        uint32_t overruns = 0;
        uint32_t tx_count = 0;
        uint32_t tx_done = 0;
        uint32_t tx_timeout = 0;
        uint8_t telemetry_denom = 1;
        uint8_t lq = 0;
        Error error = Error::NONE;
        bool binding_complete = false;
        uint8_t binding_uid[ELRS::UID_LENGTH] {};
    };
    HAL_Semaphore _report_sem;
    Report _report;
    void worker();
    void publish_report(Error error);
    void fail(Error error);
    void packet_received(const uint8_t *packet);
};

#endif // AP_PERIPH_ELRS_ENABLED
