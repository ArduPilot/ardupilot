#pragma once

#include <GCS_MAVLink/GCS_config.h>

#if HAL_GCS_ENABLED

#include <GCS_MAVLink/GCS.h>
#include "GCS_Mavlink.h"

class GCS_GenericVehicle : public GCS
{
    friend class GenericVehicle;  // for access to _chan in parameter declarations

public:

    // the following define expands to a pair of methods to retrieve a
    // pointer to an object of the correct subclass for the link at
    // offset ofs.  These are of the form:
    // GCS_MAVLINK_XXXX *chan(const uint8_t ofs) override;
    // const GCS_MAVLINK_XXXX *chan(const uint8_t ofs) override const;
    GCS_MAVLINK_CHAN_METHOD_DEFINITIONS(GCS_MAVLINK_GenericVehicle);

protected:

    uint8_t sysid_this_mav() const override;
    uint32_t custom_mode() const override { return 0; }
    MAV_TYPE frame_type() const override { return MAV_TYPE_GENERIC; }

    GCS_MAVLINK_GenericVehicle *new_gcs_mavlink_backend(GCS_MAVLINK_Parameters &params,
                                               AP_HAL::UARTDriver &uart) override {
        return new GCS_MAVLINK_GenericVehicle(params, uart);
    }
};

#endif  // HAL_GCS_ENABLED
