#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_ENABLED

#include "AP_RangeFinder_Backend.h"

class AP_RangeFinder_Backend_Serial : public AP_RangeFinder_Backend
{
public:
    // constructor
    AP_RangeFinder_Backend_Serial(RangeFinder::RangeFinder_State &_state,
                                  AP_RangeFinder_Params &_params);

    void init_serial(uint8_t serial_instance) override;

protected:

    // baudrate used during object construction:
    virtual uint32_t initial_baudrate(uint8_t serial_instance) const;

    // the value 0 is special to the UARTDriver - it's "use default"
    virtual uint16_t rx_bufsize() const { return 0; }
    virtual uint16_t tx_bufsize() const { return 0; }

    AP_HAL::UARTDriver *uart = nullptr;

    // update state; not all backends call this!
    virtual void update(void) override;

    // it is essential that anyone relying on the base-class update to
    // implement this:
    virtual bool get_reading(float &reading_m) = 0;

    // returns 0-100 or -1. This virtual method is for
    // serial drivers and is a companion to the previous method get_reading().
    // Like get_reading() this method is called in the base-class update() method.
    virtual int8_t get_signal_quality_pct() const WARN_IF_UNUSED
    { return RangeFinder::SIGNAL_QUALITY_UNKNOWN; }

    // maximum time between readings before we change state to NoData:
    virtual uint16_t read_timeout_ms() const { return 200; }
};

#endif  // AP_RANGEFINDER_ENABLED
