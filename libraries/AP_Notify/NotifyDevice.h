#ifndef __NOTIFYDEVICE_H__
#define __NOTIFYDEVICE_H__

#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

class NotifyDevice {
public:
    virtual ~NotifyDevice() {}
    // init - initialised the device
    virtual bool init(void) = 0;
    // update - updates device according to timed_updated.  Should be
    // called at 50Hz
    virtual void update() = 0;
    // handle a LED_CONTROL message, by default device ignore message
    virtual void handle_led_control(mavlink_message_t *msg) {}
};

#endif
