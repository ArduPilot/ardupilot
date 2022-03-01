#pragma once

#include "AP_Generator_Backend.h"

#if HAL_GENERATOR_ENABLED

class AP_Generator_IE_FuelCell : public AP_Generator_Backend
{

public:
    // Constructor
    using AP_Generator_Backend::AP_Generator_Backend;

    // Initialize the fuel cell driver
    void init(void) override;

    // Check if readings are healthy
    bool healthy(void) const override { return _healthy; }

    // Check for arming
    bool pre_arm_check(char *failmsg, uint8_t failmsg_len) const override;

    // Update fuel cell, expected to be called at 20hz
    void update(void) override;

protected:

    // Pointer to serial uart
    AP_HAL::UARTDriver *_uart = nullptr;

    // IE fuel cell running states
    enum class State {
        STARTING       = 0,
        READY          = 1,
        RUNNING        = 2,
        FAULT          = 3,
        BATTERY_ONLY   = 8,
    };

    // State enum to string lookup
    struct Lookup_State {
        State option;
        const char *msg_txt;
    };

    static const Lookup_State lookup_state[];

    uint32_t _err_code;       // The error code from fuel cell
    uint32_t _last_err_code;  // The previous error code from fuel cell
    State _state;          // The PSU state
    State _last_state;     // The previous PSU state
    uint32_t _last_time_ms;   // Time we got a reading
    bool _healthy;        // Is the driver working
    uint32_t _health_warn_last_ms; // Time to persist warning message without spamming

    // Temporary state params
    struct ParsedValue {
        float tank_pct;
        uint16_t tank_bar;
        float battery_pct;
        float battery_volt;
        int16_t pwr_out;
        uint16_t spm_pwr;
        int16_t battery_pwr;
        uint8_t state;
        uint32_t err_code;
    } _parsed;

    // Constants
    static const uint8_t TERM_BUFFER = 12; // Max length of term we expect
    static const uint16_t HEALTHY_TIMEOUT_MS = 5000; // Time for driver to be marked un-healthy

    // Decoding vars
    char _term[TERM_BUFFER];    // Term buffer
    bool _sentence_valid;       // Is current sentence valid
    bool _data_valid;           // Is data within expected limits
    uint8_t _term_number;       // Term index within the current sentence
    uint8_t _term_offset;       // Offset within the _term buffer where the next character should be placed
    bool _in_string;            // True if we should be decoding

    // Assigns the unit specific measurements once a valid sentence is obtained
    virtual void assign_measurements(const uint32_t now) = 0;

    virtual void log_write(void) {}

    // Add a single character to the buffer and attempt to decode.
    // Returns true if a complete sentence was successfully decoded or if the buffer is full.
    bool decode(char c);

    // Unit specific decoding to process characters recieved and build sentence
    virtual void decode_latest_term(void) = 0;

    // Check if we should notify on any change of fuel cell state
    void check_status(const uint32_t now);

    // Check error codes and populate message with error code
    virtual bool check_for_err_code(char* msg_txt, uint8_t msg_len) const = 0;

    // Only check the error code if it has changed since we last checked
    bool check_for_err_code_if_changed(char* msg_txt, uint8_t msg_len);

};
#endif
