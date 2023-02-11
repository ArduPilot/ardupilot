#pragma once

#include <GCS_MAVLink/GCS.h>
#include "GCS_Mavlink.h"

class GCS_Plane : public GCS
{
    friend class Plane;  // for access to _chan in parameter declarations

public:

    // the following define expands to a pair of methods to retrieve a
    // pointer to an object of the correct subclass for the link at
    // offset ofs.  These are of the form:
    // GCS_MAVLINK_XXXX *chan(const uint8_t ofs) override;
    // const GCS_MAVLINK_XXXX *chan(const uint8_t ofs) override const;
    GCS_MAVLINK_CHAN_METHOD_DEFINITIONS(GCS_MAVLINK_Plane);

protected:

    uint8_t sysid_this_mav() const override;
    void update_vehicle_sensor_status_flags(void) override;
    uint32_t custom_mode() const override;
    MAV_TYPE frame_type() const override;

    GCS_MAVLINK_Plane *new_gcs_mavlink_backend(GCS_MAVLINK_Parameters &params,
                                               AP_HAL::UARTDriver &uart) override {
        return new GCS_MAVLINK_Plane(params, uart);
    }

    AP_GPS::GPS_Status min_status_for_gps_healthy() const override {
        // NO_FIX simply excludes NO_GPS
        return AP_GPS::GPS_OK_FIX_3D;
    }
};
