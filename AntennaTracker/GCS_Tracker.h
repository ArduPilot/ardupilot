#pragma once

#include <GCS_MAVLink/GCS.h>
#include "GCS_Mavlink.h"

class GCS_Tracker : public GCS
{
    friend class Tracker; // for access to _chan in parameter declarations
    friend class GCS_MAVLINK_Tracker;

public:

    // the following define expands to a pair of methods to retrieve a
    // pointer to an object of the correct subclass for the link at
    // offset ofs.  These are of the form:
    // GCS_MAVLINK_XXXX *chan(const uint8_t ofs) override;
    // const GCS_MAVLINK_XXXX *chan(const uint8_t ofs) override const;
    GCS_MAVLINK_CHAN_METHOD_DEFINITIONS(GCS_MAVLINK_Tracker);

    void update_vehicle_sensor_status_flags() override;

    uint32_t custom_mode() const override;
    MAV_TYPE frame_type() const override;

protected:

    uint8_t sysid_this_mav() const override;

    GCS_MAVLINK_Tracker *new_gcs_mavlink_backend(GCS_MAVLINK_Parameters &params,
                                                 AP_HAL::UARTDriver &uart) override {
        return new GCS_MAVLINK_Tracker(params, uart);
    }

private:

    void request_datastream_position(uint8_t sysid, uint8_t compid);
    void request_datastream_airpressure(uint8_t sysid, uint8_t compid);

};
