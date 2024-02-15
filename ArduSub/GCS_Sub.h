#pragma once

#include <GCS_MAVLink/GCS.h>
#include "GCS_Mavlink.h"

class GCS_Sub : public GCS
{
    friend class Sub; // for access to _chan in parameter declarations

public:

    // the following define expands to a pair of methods to retrieve a
    // pointer to an object of the correct subclass for the link at
    // offset ofs.  These are of the form:
    // GCS_MAVLINK_XXXX *chan(const uint8_t ofs) override;
    // const GCS_MAVLINK_XXXX *chan(const uint8_t ofs) override const;
    GCS_MAVLINK_CHAN_METHOD_DEFINITIONS(GCS_MAVLINK_Sub);

    void update_vehicle_sensor_status_flags() override;

    uint32_t custom_mode() const override;
    MAV_TYPE frame_type() const override;

    bool vehicle_initialised() const override;

protected:

    uint8_t sysid_this_mav() const override;

    // minimum amount of time (in microseconds) that must remain in
    // the main scheduler loop before we are allowed to send any
    // mavlink messages.  We want to prioritise the main flight
    // control loop over communications
    uint16_t min_loop_time_remaining_for_message_send_us() const override {
        return 250;
    }

    GCS_MAVLINK_Sub *new_gcs_mavlink_backend(GCS_MAVLINK_Parameters &params,
                                             AP_HAL::UARTDriver &uart) override {
        return new GCS_MAVLINK_Sub(params, uart);
    }

};
