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

#include <AP_AHRS/AP_AHRS.h>
#include <AP_RSSI/AP_RSSI.h>

#include "AP_MSP.h"
#include "AP_MSP_Telem_Generic.h"
#include "AP_MSP_Telem_DJI.h"

#include <ctype.h>
#include <stdio.h>

#if HAL_MSP_ENABLED

const uint16_t  OSD_FLIGHT_MODE_FOCUS_TIME = 2000;
extern const AP_HAL::HAL& hal;

using namespace MSP;

const AP_Param::GroupInfo AP_MSP::var_info[] = {

    // @Param: _OSD_NCELLS
    // @DisplayName: Cell count override
    // @Description: Used for average cell voltage calculation
    // @Values: 0:Auto,1:1,2:2,3:3,4:4,5:5,6:6,7:7,8:8,9:9,10:10,11:11,12:12,13:13,14:14
    // @User: Standard
    AP_GROUPINFO("_OSD_NCELLS", 1, AP_MSP, _cellcount, 0),

    // @Param: _OPTIONS
    // @DisplayName: MSP OSD Options
    // @Description: A bitmask to set some MSP specific options
    // @Bitmask: 0:EnableTelemetryMode, 1: DJIWorkarounds
    // @User: Standard
    AP_GROUPINFO("_OPTIONS", 2, AP_MSP, _options, OPTION_TELEMETRY_DJI_WORKAROUNDS),

    AP_GROUPEND
};

AP_MSP *AP_MSP::_singleton;

AP_MSP::AP_MSP()
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

bool AP_MSP::init_backend(uint8_t backend_idx, AP_HAL::UARTDriver *uart, AP_SerialManager::SerialProtocol protocol)
{
    if (protocol == AP_SerialManager::SerialProtocol_MSP) {
        _backends[backend_idx] = new AP_MSP_Telem_Generic(uart);
    } else if (protocol == AP_SerialManager::SerialProtocol_DJI_FPV) {
        _backends[backend_idx] = new AP_MSP_Telem_DJI(uart);
    } else {
        return false;
    }
    if (_backends[backend_idx] != nullptr) {
        _backends[backend_idx]->init();
        return true;
    }
    return false;
}

/*
 * init - perform required initialisation
 */
void AP_MSP::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();
    AP_HAL::UARTDriver *uart = nullptr;

    // DJI FPV backends
    for (uint8_t protocol_instance=0; protocol_instance<MSP_MAX_INSTANCES; protocol_instance++) {
        uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_DJI_FPV, protocol_instance);
        if (uart != nullptr) {
            if (!init_backend(_msp_status.backend_count, uart, AP_SerialManager::SerialProtocol_DJI_FPV)) {
                break;
            }
            // initialize osd settings from OSD backend
            if (!_msp_status.osd_initialized) {
                init_osd();
            }
            _msp_status.backend_count++;
        }
    }
    // generic MSP backends
    for (uint8_t protocol_instance=0; protocol_instance<MSP_MAX_INSTANCES-_msp_status.backend_count; protocol_instance++) {
        uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_MSP, protocol_instance);
        if (uart != nullptr) {
            if (!init_backend(_msp_status.backend_count, uart, AP_SerialManager::SerialProtocol_MSP)) {
                break;
            }
            _msp_status.backend_count++;
        }
    }

    if (_msp_status.backend_count > 0) {
        // we've found at least 1 msp backend, start protocol handler
        if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_MSP::loop, void),
                                          "MSP",
                                          1024, AP_HAL::Scheduler::PRIORITY_IO, 1)) {
            return;
        }
    }
}

void AP_MSP::init_osd()
{
#if OSD_ENABLED
    AP_OSD* osd = AP::osd();

    if (osd == nullptr) {
        return;
    }

    _osd_item_settings[OSD_RSSI_VALUE] = &osd->screen[0].rssi;
    _osd_item_settings[OSD_MAIN_BATT_VOLTAGE] = &osd->screen[0].bat_volt;
    _osd_item_settings[OSD_CROSSHAIRS] = &osd->screen[0].crosshair;
    _osd_item_settings[OSD_ARTIFICIAL_HORIZON] = &osd->screen[0].horizon;
    _osd_item_settings[OSD_HORIZON_SIDEBARS] = &osd->screen[0].sidebars;
    _osd_item_settings[OSD_CRAFT_NAME] = &osd->screen[0].message;
    _osd_item_settings[OSD_FLYMODE] = &osd->screen[0].fltmode;
    _osd_item_settings[OSD_CURRENT_DRAW] = &osd->screen[0].current;
    _osd_item_settings[OSD_MAH_DRAWN] = &osd->screen[0].batused;
    _osd_item_settings[OSD_GPS_SPEED] = &osd->screen[0].gspeed;
    _osd_item_settings[OSD_GPS_SATS] = &osd->screen[0].sats;
    _osd_item_settings[OSD_ALTITUDE] = &osd->screen[0].altitude;
    _osd_item_settings[OSD_POWER] = &osd->screen[0].power;
    _osd_item_settings[OSD_AVG_CELL_VOLTAGE] = &osd->screen[0].cell_volt;
    _osd_item_settings[OSD_GPS_LON] = &osd->screen[0].gps_longitude;
    _osd_item_settings[OSD_GPS_LAT] = &osd->screen[0].gps_latitude;
    _osd_item_settings[OSD_PITCH_ANGLE] = &osd->screen[0].pitch_angle;
    _osd_item_settings[OSD_ROLL_ANGLE] = &osd->screen[0].roll_angle;
    _osd_item_settings[OSD_MAIN_BATT_USAGE] = &osd->screen[0].batt_bar;
    _osd_item_settings[OSD_DISARMED] = &osd->screen[0].arming;
    _osd_item_settings[OSD_HOME_DIR] = &osd->screen[0].home_dir;
    _osd_item_settings[OSD_HOME_DIST] = &osd->screen[0].home_dist;
    _osd_item_settings[OSD_NUMERICAL_HEADING] = &osd->screen[0].heading;
    _osd_item_settings[OSD_NUMERICAL_VARIO] = &osd->screen[0].vspeed;
#if HAL_WITH_ESC_TELEM
    _osd_item_settings[OSD_ESC_TMP] = &osd->screen[0].esc_temp;
#endif
    _osd_item_settings[OSD_RTC_DATETIME] = &osd->screen[0].clk;
#endif  // OSD_ENABLED
    _msp_status.osd_initialized = true;
}

void AP_MSP::loop(void)
{
    for (uint8_t i=0; i<_msp_status.backend_count; i++) {
        // one time uart init
        if (_backends[i] != nullptr)  {
            _backends[i]->init_uart();
        }
    }

    while (true) {
        hal.scheduler->delay(10); // 115200 baud, 18 MSP packets @4Hz, 100Hz should be OK

        const uint32_t now = AP_HAL::millis();
        // toggle flashing every 0.7 seconds
        if (((now / 700) & 0x01) != _msp_status.flashing_on) {
            _msp_status.flashing_on = !_msp_status.flashing_on;
        }

        // detect flight mode changes and steal focus from text messages
        if (AP::notify().flags.flight_mode != _msp_status.last_flight_mode) {
            _msp_status.flight_mode_focus = true;
            _msp_status.last_flight_mode = AP::notify().flags.flight_mode;
            _msp_status.last_flight_mode_change_ms = AP_HAL::millis();
        } else if (now - _msp_status.last_flight_mode_change_ms > OSD_FLIGHT_MODE_FOCUS_TIME) {
            _msp_status.flight_mode_focus = false;
        }

        for (uint8_t i=0; i< _msp_status.backend_count; i++) {
            if (_backends[i] != nullptr) {
                // dynamically hide/unhide
                _backends[i]->hide_osd_items();
                // process incoming MSP frames (and reply if needed)
                _backends[i]->process_incoming_data();
                // push outgoing telemetry frames
                _backends[i]->process_outgoing_data();
            }
        }
    }
}

bool AP_MSP::check_option(msp_option_e option)
{
    return (_options & option) != 0;
}

namespace AP
{
AP_MSP *msp()
{
    return AP_MSP::get_singleton();
}
};

#endif //HAL_MSP_ENABLED
