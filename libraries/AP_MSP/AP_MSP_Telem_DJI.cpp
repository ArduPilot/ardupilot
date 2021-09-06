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

#include <AP_Common/AP_FWVersion.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#include "AP_MSP.h"
#include "AP_MSP_Telem_DJI.h"

#if HAL_MSP_ENABLED
extern const AP_HAL::HAL& hal;

using namespace MSP;

bool AP_MSP_Telem_DJI::init_uart()
{
    if (_msp_port.uart != nullptr)  {
        _msp_port.uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        _msp_port.uart->begin(AP_SERIALMANAGER_MSP_BAUD, AP_SERIALMANAGER_MSP_BUFSIZE_RX, AP_SERIALMANAGER_MSP_BUFSIZE_TX);
        return true;
    }
    return false;
}

bool AP_MSP_Telem_DJI::is_scheduler_enabled() const
{
    const AP_MSP *msp = AP::msp();
    if (msp == nullptr) {
        return false;
    }
    return msp->check_option(AP_MSP::MspOption::OPTION_TELEMETRY_MODE);
}

void AP_MSP_Telem_DJI::hide_osd_items(void)
{
    const AP_MSP *msp = AP::msp();
    if (msp == nullptr) {
        return;
    }
    // apply base class defaults
    AP_MSP_Telem_Backend::hide_osd_items();

    // apply DJI OSD specific rules
    const AP_Notify& notify = AP::notify();
    // default is hide the DJI flightmode widget
    BIT_SET(osd_hidden_items_bitmask, OSD_FLYMODE);

    if (msp->_msp_status.flashing_on) {
        // flash flightmode on failsafe
        if (notify.flags.failsafe_battery || notify.flags.failsafe_gcs || notify.flags.failsafe_radio || notify.flags.ekf_bad) {
            BIT_CLEAR(osd_hidden_items_bitmask, OSD_FLYMODE);
        }
    }
}

uint32_t AP_MSP_Telem_DJI::get_osd_flight_mode_bitmask(void)
{
    uint32_t mode_mask = 0;
    const AP_Notify& notify = AP::notify();

    // set arming status
    if (notify.flags.armed) {
        BIT_SET(mode_mask, DJI_FLAG_ARM);
    }

    // check failsafe
    if (notify.flags.failsafe_battery || notify.flags.failsafe_gcs || notify.flags.failsafe_radio || notify.flags.ekf_bad ) {
        BIT_SET(mode_mask, DJI_FLAG_FS);
    }
    return mode_mask;
}

MSPCommandResult AP_MSP_Telem_DJI::msp_process_out_api_version(sbuf_t *dst)
{
    struct {
        uint8_t proto;
        uint8_t major;
        uint8_t minor;
    } api_version;

    api_version.proto = MSP_PROTOCOL_VERSION;
    api_version.major = API_VERSION_MAJOR;
    api_version.minor = API_VERSION_MINOR;

    sbuf_write_data(dst, &api_version, sizeof(api_version));
    return MSP_RESULT_ACK;
}

MSPCommandResult AP_MSP_Telem_DJI::msp_process_out_fc_version(sbuf_t *dst)
{
    struct {
        uint8_t major;
        uint8_t minor;
        uint8_t patch;
    } fc_version;

    fc_version.major = FC_VERSION_MAJOR;
    fc_version.minor = FC_VERSION_MINOR;
    fc_version.patch = FC_VERSION_PATCH_LEVEL;

    sbuf_write_data(dst, &fc_version, sizeof(fc_version));
    return MSP_RESULT_ACK;
}

MSPCommandResult AP_MSP_Telem_DJI::msp_process_out_fc_variant(sbuf_t *dst)
{
    sbuf_write_data(dst, "BTFL", FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);
    return MSP_RESULT_ACK;
}

MSPCommandResult AP_MSP_Telem_DJI::msp_process_out_esc_sensor_data(sbuf_t *dst)
{
#if HAL_WITH_ESC_TELEM
    const auto msp = AP::msp();
    if (msp && msp->check_option(AP_MSP::MspOption::OPTION_TELEMETRY_DJI_WORKAROUNDS)) {
        AP_ESC_Telem& telem = AP::esc_telem();
        int16_t highest_temperature = 0;
        telem.get_highest_motor_temperature(highest_temperature);

        struct PACKED {
            uint8_t temp;
            uint16_t rpm;
        } esc_sensor_data {};

        esc_sensor_data.temp = uint8_t(highest_temperature * 0.01f);            // deg, report max temperature
        esc_sensor_data.rpm = uint16_t(telem.get_average_motor_rpm() * 0.1f);   // rpm, report average RPM across all motors
        sbuf_write_data(dst, &esc_sensor_data, sizeof(esc_sensor_data));
    } else {
        return AP_MSP_Telem_Backend::msp_process_out_esc_sensor_data(dst);
    }
#endif
    return MSP_RESULT_ACK;
}

#endif //HAL_MSP_ENABLED
