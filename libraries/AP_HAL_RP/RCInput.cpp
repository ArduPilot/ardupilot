#include "RCInput.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_InternalError/AP_InternalError.h>

using namespace RP;

extern const AP_HAL::HAL& hal;

RCInput::RCInput() : _init(false), _uart(nullptr)
{}

void RCInput::init() {
    if (_init) {
        return;
    }

    // Initialize the ArduPilot general protocol manager
    // It will automatically check the incoming data for CRSF, S.Bus, etc.
#if AP_RCPROTOCOL_ENABLED
    AP::RC().init();
#endif

#if ERLS_BUILT_IN_MODULE_ENABLED
#ifdef ELRS_UART
    _uart = ELRS_UART;
#else
    _uart = hal.serial(0); // Serial0 (UART0) is used by default
#endif
    if (_uart != nullptr) {
        // Default speed for ELRS/CRSF.
        // AP_RCProtocol can change it itself as needed.
        _uart->begin(420000);
    }
#endif

    // For other RP2350 boards that use PPM (via PIO)
#ifdef HAL_RP_PULSE_INPUT_ENABLED
    sig_reader.init();
#endif

    _init = true;
}

uint8_t RCInput::num_channels() {
#if AP_RCPROTOCOL_ENABLED
    return AP::RC().num_channels();
#else
    return 0;
#endif
}

uint16_t RCInput::read(uint8_t ch) {
#if AP_RCPROTOCOL_ENABLED
    return AP::RC().read(ch);
#else
    return 0;
#endif
}

uint8_t RCInput::read(uint16_t* periods, uint8_t len) {
#if AP_RCPROTOCOL_ENABLED
    AP::RC().read(periods, len);
    uint8_t n = AP::RC().num_channels();
    return (n > len) ? len : n;
#else
    return 0;
#endif
}

bool RCInput::new_input() {
    _process_uart();

#ifdef HAL_RP_PULSE_INPUT_ENABLED
    sig_reader.process();
#endif

#if AP_RCPROTOCOL_ENABLED
    return AP::RC().new_input();
#else
    return false;
#endif
}

void RCInput::_process_uart() {
    if (_uart == nullptr) return;

    uint32_t n = _uart->available();
    if (n > 0) {
        for (uint32_t i = 0; i < n; i++) {
            uint8_t b = _uart->read();
#if AP_RCPROTOCOL_ENABLED
            // Pass the byte to AP_RCProtocol, which knows all about ELRS/CRSF
            // Parameter 0 means the first RC port
            AP::RC().process_byte(b, 0); 
#endif
        }
    }
}
