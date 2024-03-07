#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_SITL_ENABLED

#include "AP_RCProtocol_SITL.h"

#include <AP_HAL_SITL/AP_HAL_SITL.h>
#include <AP_HAL_SITL/SITL_State.h>

#include <SITL/SITL.h>
extern const HAL_SITL& hal_sitl;

void AP_RCProtocol_SITL::process_bytes(uint8_t *bytes, uint16_t count, uint32_t baudrate)
{
    const uint32_t now = AP_HAL::millis();
    if (now - last_byte_ms > 10) {
        // could be a framing failure.  Might also just be first byte
        // in packet Discard.
        search_buf_ofs = 0;
    }
    last_byte_ms = now;

    // if we get 32 bytes that's 16 values. 16 is 8 values.  1 byte is
    // "the RCProtocol library is searching"
    switch (count) {
    case 32:
    case 16:
        add_input(count/2, (uint16_t*)bytes, false);
        search_success_correct_size_start_ms = 0;
        return;
    case 1:
        // constant shift of bytes down
        if (search_buf_ofs >= ARRAY_SIZE(search_buf)) {
            memmove(&search_buf[1], &search_buf[0], search_buf_ofs-1);
        }
        search_buf[search_buf_ofs++] = bytes[0];
        if (search_buf_ofs == 8 || search_buf_ofs == 16) {
            search_success_correct_size_start_ms = now;
        } else {
            search_success_correct_size_start_ms = 0;
        }
        break;
    default:
        AP_HAL::panic("unexpected SITL byte count");
    }
}

void AP_RCProtocol_SITL::update()
{
    const auto *sitl = AP::sitl();
    if (sitl->state.rcin_chan_count) {
        const uint16_t *pwm_input = hal_sitl.get_pwm_input();
        add_input(sitl->state.rcin_chan_count, pwm_input, false);
        return;
    }

    // frame-gap protection:
    const uint32_t now = AP_HAL::millis();
    if (now - search_success_correct_size_start_ms) {
        add_input(search_buf_ofs/2, (uint16_t*)search_buf, false);
        search_success_correct_size_start_ms = 0;
    }
}

#endif // AP_RCPROTOCOL_SITL_ENABLED

