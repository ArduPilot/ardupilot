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
#include "AP_MSP_Telem_DisplayPort.h"

#include <AP_Notify/AP_Notify.h>

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
    // @Bitmask: 0:EnableTelemetryMode, 1: DisableDJIWorkarounds, 2:EnableBTFLFonts
    // @User: Standard
    AP_GROUPINFO("_OPTIONS", 2, AP_MSP, _options, 0),

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
#if HAL_WITH_MSP_DISPLAYPORT
    } else if (protocol == AP_SerialManager::SerialProtocol_MSP_DisplayPort) {
        _backends[backend_idx] = new AP_MSP_Telem_DisplayPort(uart);
#endif
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
    uint8_t backends_using_msp_thread = 0;
    static const AP_SerialManager::SerialProtocol msp_protocols[] {
        AP_SerialManager::SerialProtocol_DJI_FPV,
        AP_SerialManager::SerialProtocol_MSP,
#if HAL_WITH_MSP_DISPLAYPORT
        AP_SerialManager::SerialProtocol_MSP_DisplayPort,
#endif
    };

    for (const auto msp_protocol: msp_protocols) {
        for (uint8_t protocol_instance=0; protocol_instance<MSP_MAX_INSTANCES-_msp_status.backend_count; protocol_instance++) {
            uart = serial_manager.find_serial(msp_protocol, protocol_instance);
            if (uart != nullptr) {
                if (!init_backend(_msp_status.backend_count, uart, msp_protocol)) {
                    break;
                }
                if (_backends[_msp_status.backend_count]->use_msp_thread()) {
                    backends_using_msp_thread++;
                }
                _msp_status.backend_count++;
            }
        }
    }

    if (backends_using_msp_thread > 0) {
        // we've found at least 1 msp backend, start protocol handler
        if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_MSP::loop, void),
                                          "MSP",
                                          1024, AP_HAL::Scheduler::PRIORITY_IO, 1)) {
            return;
        }
    }
}

void AP_MSP::update_osd_item_settings()
{
#if OSD_ENABLED
    AP_OSD* osd = AP::osd();

    if (osd == nullptr) {
        return;
    }
    _osd_item_settings[OSD_RSSI_VALUE] = &osd->screen[_msp_status.current_screen].rssi;             // OSDn_RSSI
    _osd_item_settings[OSD_MAIN_BATT_VOLTAGE] = &osd->screen[_msp_status.current_screen].bat_volt;  // OSDn_BAT_VOLT
    _osd_item_settings[OSD_CROSSHAIRS] = &osd->screen[_msp_status.current_screen].crosshair;        // OSDn_CRSSHAIR
    _osd_item_settings[OSD_ARTIFICIAL_HORIZON] = &osd->screen[_msp_status.current_screen].horizon;  // OSDn_HORIZON
    _osd_item_settings[OSD_HORIZON_SIDEBARS] = &osd->screen[_msp_status.current_screen].sidebars;   // OSDn_SIDEBARS
    _osd_item_settings[OSD_CRAFT_NAME] = &osd->screen[_msp_status.current_screen].message;          // OSDn_MESSAGE
    _osd_item_settings[OSD_FLYMODE] = &osd->screen[_msp_status.current_screen].fltmode;             // OSDn_FLTMODE
    _osd_item_settings[OSD_CURRENT_DRAW] = &osd->screen[_msp_status.current_screen].current;        // OSDn_CURRENT
    _osd_item_settings[OSD_MAH_DRAWN] = &osd->screen[_msp_status.current_screen].batused;           // OSDn_BATUSED
    _osd_item_settings[OSD_GPS_SPEED] = &osd->screen[_msp_status.current_screen].gspeed;            // OSDn_GSPEED
    _osd_item_settings[OSD_GPS_SATS] = &osd->screen[_msp_status.current_screen].sats;               // OSDn_SATS
    _osd_item_settings[OSD_ALTITUDE] = &osd->screen[_msp_status.current_screen].altitude;           // OSDn_ALTITUDE
    _osd_item_settings[OSD_POWER] = &osd->screen[_msp_status.current_screen].power;                 // OSDn_POWER
    _osd_item_settings[OSD_AVG_CELL_VOLTAGE] = &osd->screen[_msp_status.current_screen].cell_volt;  // OSDn_CELLVOLT
    _osd_item_settings[OSD_GPS_LON] = &osd->screen[_msp_status.current_screen].gps_longitude;       // OSDn_GPSLONG
    _osd_item_settings[OSD_GPS_LAT] = &osd->screen[_msp_status.current_screen].gps_latitude;        // OSDn_GPSLAT
    _osd_item_settings[OSD_PITCH_ANGLE] = &osd->screen[_msp_status.current_screen].pitch_angle;     // OSDn_PITCH
    _osd_item_settings[OSD_ROLL_ANGLE] = &osd->screen[_msp_status.current_screen].roll_angle;       // OSDn_ROLL
    _osd_item_settings[OSD_MAIN_BATT_USAGE] = &osd->screen[_msp_status.current_screen].batt_bar;    // OSDn_BATBAR
    _osd_item_settings[OSD_DISARMED] = &osd->screen[_msp_status.current_screen].arming;             // OSDn_ARMING
    _osd_item_settings[OSD_HOME_DIR] = &osd->screen[_msp_status.current_screen].home_dir;           // OSDn_HOMEDIR
    _osd_item_settings[OSD_HOME_DIST] = &osd->screen[_msp_status.current_screen].home_dist;         // OSDn_HOMEDIST
    _osd_item_settings[OSD_NUMERICAL_HEADING] = &osd->screen[_msp_status.current_screen].heading;   // OSDn_HEADING
    _osd_item_settings[OSD_NUMERICAL_VARIO] = &osd->screen[_msp_status.current_screen].vspeed;      // OSDn_VSPEED
#if HAL_WITH_ESC_TELEM
    _osd_item_settings[OSD_ESC_TMP] = &osd->screen[_msp_status.current_screen].esc_temp;            // OSDn_ESCTEMP
#endif
    _osd_item_settings[OSD_RTC_DATETIME] = &osd->screen[_msp_status.current_screen].clk;            // OSDn_CLK
#endif  // OSD_ENABLED
    _msp_status.osd_initialized = true;
}

void AP_MSP::loop(void)
{
    for (uint8_t i=0; i<_msp_status.backend_count; i++) {
        // one time uart init
        // note: we do not access a uart for a backend handled by another thread
        if (_backends[i] != nullptr && _backends[i]->use_msp_thread())  {
            _backends[i]->init_uart();
        }
    }

    while (true) {
        hal.scheduler->delay(10); // 115200 baud, 18 MSP packets @4Hz, 100Hz should be OK

        const uint32_t now = AP_HAL::millis();
        // toggle flashing every 0.7 seconds and every 2 seconds
        if ((uint32_t(now * 0.00143) & 0x01) != _msp_status.flashing_on) {
            _msp_status.flashing_on = !_msp_status.flashing_on;
        }
        if ((uint32_t(now * 0.0005) & 0x01) != _msp_status.slow_flashing_on) {
            _msp_status.slow_flashing_on = !_msp_status.slow_flashing_on;
        }

        // detect flight mode changes and steal focus from text messages
        if (AP::notify().flags.flight_mode != _msp_status.last_flight_mode) {
            _msp_status.flight_mode_focus = true;
            _msp_status.last_flight_mode = AP::notify().flags.flight_mode;
            _msp_status.last_flight_mode_change_ms = AP_HAL::millis();
        } else if (now - _msp_status.last_flight_mode_change_ms > OSD_FLIGHT_MODE_FOCUS_TIME) {
            _msp_status.flight_mode_focus = false;
        }

#if OSD_ENABLED
        // check if we had a screen change
        AP_OSD* osd = AP::osd();

        if (osd != nullptr) {
            const uint8_t screen = osd->is_readonly_screen() ? osd->get_current_screen() : _msp_status.current_screen;
            if (_msp_status.current_screen != screen || !_msp_status.osd_initialized) {
                _msp_status.current_screen = screen;
                update_osd_item_settings();
            }
        }
#endif  // OSD_ENABLED

        for (uint8_t i=0; i< _msp_status.backend_count; i++) {
            // note: we do not access a uart for a backend handled by another thread
            if (_backends[i] != nullptr && _backends[i]->use_msp_thread()) {
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

AP_MSP_Telem_Backend* AP_MSP::find_protocol(const AP_SerialManager::SerialProtocol protocol) const {
    for (uint8_t i=0; i< _msp_status.backend_count; i++) {
        if (_backends[i] != nullptr && _backends[i]->get_serial_protocol() == protocol) {
            return _backends[i];
        }
    }
    return nullptr;
}

bool AP_MSP::is_option_enabled(Option option) const
{
    return (_options & (uint8_t)option) != 0;
}

namespace AP
{
AP_MSP *msp()
{
    return AP_MSP::get_singleton();
}
};

#endif //HAL_MSP_ENABLED
