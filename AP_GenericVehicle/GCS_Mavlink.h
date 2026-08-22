#pragma once

#include <GCS_MAVLink/GCS_config.h>

#if HAL_GCS_ENABLED

#ifndef MAV_SYSTEM_ID
#define MAV_SYSTEM_ID 4
#endif

#include <GCS_MAVLink/GCS.h>

class GCS_MAVLINK_GenericVehicle : public GCS_MAVLINK
{

public:

    using GCS_MAVLINK::GCS_MAVLINK;

    uint8_t sysid_my_gcs() const override { return 0; }

protected:

    uint32_t telem_delay() const override { return 0; }

    void send_nav_controller_output() const override {}
    void send_pid_tuning() override {}

private:

    MAV_MODE base_mode() const override { return (MAV_MODE)MAV_MODE_FLAG_CUSTOM_MODE_ENABLED; }
    MAV_STATE vehicle_system_status() const override { return MAV_STATE_ACTIVE; }

};

#endif  // HAL_GCS_ENABLED
