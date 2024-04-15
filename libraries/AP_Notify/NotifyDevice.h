#pragma once

#include <AP_Common/AP_Common.h>
#include "AP_Notify_config.h"

class AP_Notify;

class NotifyDevice {
public:
    virtual ~NotifyDevice() {}
    // init - initialised the device
    virtual bool init(void) = 0;
    // update - updates device according to timed_updated.  Should be
    // called at 50Hz
    virtual void update() = 0;

#if AP_NOTIFY_MAVLINK_LED_CONTROL_SUPPORT_ENABLED
    // handle a LED_CONTROL message, by default device ignore message
    virtual void handle_led_control(const mavlink_message_t &msg) {}
#endif

    // play a MML tune
    virtual void play_tune(const char *tune) {}

    // RGB control
    // give RGB and flash rate, used with scripting
    virtual void rgb_control(uint8_t r, uint8_t g, uint8_t b, uint8_t rate_hz) {}

    // RGB control multiple leds independently
    // give RGB value for single led
    virtual void rgb_set_id(uint8_t r, uint8_t g, uint8_t b, uint8_t id) {}

    // Allow text to be sent or removed from a display
    virtual void send_text_blocking(const char *text, uint8_t line) {}
    virtual void release_text(uint8_t line) {}

    // this pointer is used to read the parameters relative to devices
    const AP_Notify *pNotify;
};
