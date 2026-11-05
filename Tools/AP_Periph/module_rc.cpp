// SPDX-License-Identifier: GPL-3.0-or-later
#include "AP_Periph.h"
#include <AP_Radio/driver_sx1280_module.h>

#if AP_PERIPH_MODULE_RC_ENABLED
extern const AP_HAL::HAL &hal;
// Main-thread sink for module publications. RF policy lives in the applet.
#include <AP_Module/RCInputSource.h>
// Report ABI v1, shared with the native radio callback (applets/elrs/ABI.md).
namespace RadioReport
{
constexpr uint16_t SIZE = 256;
constexpr uint16_t ERROR = 2;
constexpr uint16_t FLAGS = 3;
constexpr uint16_t ERROR_TEXT = 44;
constexpr uint16_t ERROR_TEXT_LENGTH = 251;
constexpr uint8_t ERROR_TEXT_MAX = 62;
constexpr uint8_t MAINTENANCE = 1;
constexpr uint16_t CHANNEL_COUNT = 112;
constexpr uint16_t QUALITY = 113;
constexpr uint16_t FRAME = 116;
constexpr uint16_t VALID_UNTIL = 120;
constexpr uint16_t LAST_INPUT = 124;
constexpr uint16_t FAILSAFE_REPEAT = 128;
constexpr uint16_t FAILSAFE_TIMEOUT = 132;
constexpr uint16_t CHANNELS = 136;
}

static uint32_t source_u32(const uint8_t *p)
{
    return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
}
static AP_Module::RCInputSource rc_source;

bool AP_Periph_FW::module_rc_maintenance() const
{
    uint8_t report[RadioReport::SIZE];
    SX1280Module::report(report);
    return report[RadioReport::FLAGS] & RadioReport::MAINTENANCE;
}

uint8_t AP_Periph_FW::module_rc_error(char *message, uint8_t size) const
{
    uint8_t report[RadioReport::SIZE];
    SX1280Module::report(report);
    const uint8_t error = report[RadioReport::ERROR];
    if (message != nullptr && size > 0) {
        const uint8_t length = report[RadioReport::ERROR_TEXT_LENGTH];
        if (error == SX1280Module::ERROR_DETACHED) {
            hal.util->snprintf(message, size, "Radio module callback detached");
        } else if (error == 7) {
            hal.util->snprintf(message, size, "Radio worker thread creation failed");
        } else if (error != 0 && length > 0 && length <= RadioReport::ERROR_TEXT_MAX) {
            const uint8_t count = MIN(length, size - 1U);
            memcpy(message, report + RadioReport::ERROR_TEXT, count);
            message[count] = '\0';
        } else {
            hal.util->snprintf(message, size, "Radio module error %u", unsigned(error));
        }
    }
    return error;
}

void AP_Periph_FW::module_rc_update()
{
    SX1280Module::update();
    uint8_t report[RadioReport::SIZE];
    SX1280Module::report(report);
    const uint32_t now_ms = AP_HAL::millis(), now = AP_HAL::micros();
    const uint8_t rate = g_rcin.rcin_rate_hz;
    if (rate == 0 || now_ms - rcin_last_sent_RCInput_ms < 1000U / rate) {
        return;
    }
    const uint32_t frame = source_u32(report + RadioReport::FRAME);
    const uint32_t stamp = source_u32(report + RadioReport::LAST_INPUT);
    uint16_t channels[16] {};
    const uint8_t count = report[RadioReport::CHANNEL_COUNT];
    if (report[RadioReport::ERROR] == 0 &&
        rc_source.accept(now, count, frame, stamp,
                         source_u32(report + RadioReport::VALID_UNTIL),
                         source_u32(report + RadioReport::FAILSAFE_TIMEOUT),
                         source_u32(report + RadioReport::FAILSAFE_REPEAT))) {
        for (uint8_t i = 0; i < count; i++) {
            const uint8_t *channel = report + RadioReport::CHANNELS + 2 * i;
            channels[i] = uint16_t(channel[0]) | uint16_t(channel[1]) << 8;
        }
        rcin_last_sent_RCInput_ms = now_ms;
        can_send_RCInput(report[RadioReport::QUALITY], channels, count, false, true);
    } else if (rc_source.failsafe_due(now)) {
        rcin_last_sent_RCInput_ms = now_ms;
        can_send_RCInput(0, channels, 0, true, true);
    }
}
#endif // AP_PERIPH_MODULE_RC_ENABLED
