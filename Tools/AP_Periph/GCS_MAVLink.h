/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <GCS_MAVLink/GCS.h>
#if HAL_GCS_ENABLED

/*
 *  GCS backend used for many examples and tools
 */
class GCS_MAVLINK_Periph : public GCS_MAVLINK
{
public:

    using GCS_MAVLINK::GCS_MAVLINK;

private:

    uint32_t telem_delay() const override { return 0; }
    void handleMessage(const mavlink_message_t &msg) override { handle_common_message(msg); }
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override { return true; }
    MAV_RESULT handle_preflight_reboot(const mavlink_command_long_t &packet, const mavlink_message_t &msg) override;
    uint8_t sysid_my_gcs() const override;

protected:

    // Periph information:
    MAV_MODE base_mode() const override { return (MAV_MODE)MAV_MODE_FLAG_CUSTOM_MODE_ENABLED; }
    MAV_STATE vehicle_system_status() const override { return MAV_STATE_CALIBRATING; }

    bool set_home_to_current_location(bool _lock) override { return false; }
    bool set_home(const Location& loc, bool _lock) override { return false; }

    void send_nav_controller_output() const override {};
    void send_pid_tuning() override {};
};

/*
 * a GCS singleton used for many example sketches and tools
 */

extern const AP_HAL::HAL& hal;

class GCS_Periph : public GCS
{
public:

    using GCS::GCS;

protected:

    uint8_t sysid_this_mav() const override;

    GCS_MAVLINK_Periph *new_gcs_mavlink_backend(GCS_MAVLINK_Parameters &params,
                                               AP_HAL::UARTDriver &uart) override {
        return new GCS_MAVLINK_Periph(params, uart);
    }

private:
    // the following define expands to a pair of methods to retrieve a
    // pointer to an object of the correct subclass for the link at
    // offset ofs.  These are of the form:
    // GCS_MAVLINK_XXXX *chan(const uint8_t ofs) override;
    // const GCS_MAVLINK_XXXX *chan(const uint8_t ofs) override const;
    GCS_MAVLINK_CHAN_METHOD_DEFINITIONS(GCS_MAVLINK_Periph);

    MAV_TYPE frame_type() const override { return MAV_TYPE_GENERIC; }
    uint32_t custom_mode() const override { return 3; } // magic number
};
#endif // HAL_GCS_ENABLED
