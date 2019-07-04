/*
 * AP_ServoRelayEvent.h
 *
 * handle DO_SET_SERVO, DO_REPEAT_SERVO, DO_SET_RELAY and
 * DO_REPEAT_RELAY commands
 */
#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Relay/AP_Relay.h>

class AP_ServoRelayEvents {
public:
    AP_ServoRelayEvents()
        : type(EVENT_TYPE_RELAY)
    {
        _singleton = this;
    }

    /* Do not allow copies */
    AP_ServoRelayEvents(const AP_ServoRelayEvents &other) = delete;
    AP_ServoRelayEvents &operator=(const AP_ServoRelayEvents&) = delete;

    // get singleton instance
    static AP_ServoRelayEvents *get_singleton() {
        return _singleton;
    }

    bool do_set_servo(uint8_t channel, uint16_t pwm);
    bool do_set_relay(uint8_t relay_num, uint8_t state);
    bool do_repeat_servo(uint8_t channel, uint16_t servo_value, int16_t repeat, uint16_t delay_time_ms);
    bool do_repeat_relay(uint8_t relay_num, int16_t count, uint32_t period_ms);
    void update_events(void);

private:

    static AP_ServoRelayEvents *_singleton;

    // event control state
    enum event_type { 
        EVENT_TYPE_RELAY=0,
        EVENT_TYPE_SERVO=1
    };

    enum event_type type;

	// when the event was started in ms
    uint32_t start_time_ms;

	// how long to delay the next firing of event in millis
    uint16_t delay_ms;

	// how many times to cycle : -1 (or -2) = forever, 2 = do one cycle, 4 = do two cycles
    int16_t repeat;

    // RC channel for servos, relay number for relays
    uint8_t channel;

	// PWM for servos
	uint16_t servo_value;
};

namespace AP {
    AP_ServoRelayEvents *servorelayevents();
};
