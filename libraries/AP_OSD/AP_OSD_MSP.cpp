/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
/*
  OSD backend for MSP
 */
#include "AP_OSD_MSP.h"


static const struct AP_Param::defaults_table_struct defaults_table[] = {
    /*
    //OSD_RSSI_VALUE
    { "OSD_RSSI_EN",       1.0 },
    { "OSD_RSSI_X",        1.0 },
    { "OSD_RSSI_Y",        1.0 },

    //OSD_MAIN_BATT_VOLTAGE
    { "OSD_BAT_VOLT_EN",       1.0 },
    { "OSD_BAT_VOLT_X",        1.0 },
    { "OSD_BAT_VOLT_Y",        1.0 },

    //OSD_CRAFT_NAME (text flightmode + status text messages + wind)
    { "OSD_MESSAGE_EN",       1.0 },
    { "OSD_MESSAGE_X",        1.0 },
    { "OSD_MESSAGE_Y",        1.0 },

    //OSD_FLYMODE (displays failsafe status and optionally rtl engaged)
    { "OSD_FLTMODE_EN",       1.0 },
    { "OSD_FLTMODE_X",        1.0 },
    { "OSD_FLTMODE_Y",        1.0 },

    //OSD_CURRENT_DRAW
    { "OSD_CURRENT_EN",       1.0 },
    { "OSD_CURRENT_X",        1.0 },
    { "OSD_CURRENT_Y",        1.0 },

    //OSD_MAH_DRAWN
    { "OSD_BATUSED_EN",       1.0 },
    { "OSD_BATUSED_X",        1.0 },
    { "OSD_BATUSED_Y",        1.0 },

    //OSD_GPS_SPEED
    { "OSD_GSPEED_EN",       1.0 },
    { "OSD_GSPEED_X",        1.0 },
    { "OSD_GSPEED_Y",        1.0 },

    //OSD_GPS_SATS
    { "OSD_SATS_EN",       1.0 },
    { "OSD_SATS_X",        1.0 },
    { "OSD_SATS_Y",        1.0 },

    //OSD_ALTITUDE
    { "OSD_ALTITUDE_EN",       1.0 },
    { "OSD_ALTITUDE_X",        1.0 },
    { "OSD_ALTITUDE_Y",        1.0 },

    //OSD_GPS_LON
    { "OSD_GPSLONG_EN",       1.0 },
    { "OSD_GPSLONG_X",        1.0 },
    { "OSD_GPSLONG_Y",        1.0 },

    //OSD_GPS_LAT
    { "OSD_GPSLAT_EN",       1.0 },
    { "OSD_GPSLAT_X",        1.0 },
    { "OSD_GPSLAT_Y",        1.0 },

    //OSD_PITCH_ANGLE
    { "OSD_PITCH_EN",       1.0 },
    { "OSD_PITCH_X",        1.0 },
    { "OSD_PITCH_Y",        1.0 },

    //OSD_ROLL_ANGLE
    { "OSD_ROLL_EN",       1.0 },
    { "OSD_ROLL_X",        1.0 },
    { "OSD_ROLL_Y",        1.0 },

    //OSD_MAIN_BATT_USAGE
    { "OSD_BATTBAR_EN",       1.0 },
    { "OSD_BATTBAR_X",        1.0 },
    { "OSD_BATTBAR_Y",        1.0 },

    //OSD_NUMERICAL_VARIO
    { "OSD_VSPEED_EN",       1.0 },
    { "OSD_VSPEED_X",        1.0 },
    { "OSD_VSPEED_Y",        1.0 },

#if HAVE_AP_BLHELI_SUPPORT
    //OSD_ESC_TMP
    { "OSD_BLHTEMP_EN",       1.0 },
    { "OSD_BLHTEMP_X",        1.0 },
    { "OSD_BLHTEMP_Y",        1.0 },
#endif

    //OSD_RTC_DATETIME
    { "OSD_CLK_EN",       1.0 },
    { "OSD_CLK_X",        1.0 },
    { "OSD_CLK_Y",        1.0 },

    // --------------------------
    // MSP OSD only
    // --------------------------

    // OSD items disabled by default (partially supported)
    //OSD_CROSSHAIRS
    { "OSD_CRSSHAIR_EN",       0 },

    //OSD_ARTIFICIAL_HORIZON
    { "OSD_HORIZON_EN",        0 },

    //OSD_HORIZON_SIDEBARS
    { "OSD_SIDEBARS_EN",       0 },

    //OSD_NUMERICAL_HEADING
    { "OSD_HEADING_EN",       0.0 },

    // Supported OSD items

    //OSD_POWER
    { "OSD_POWER_EN",       1.0 },
    { "OSD_POWER_X",        1.0 },
    { "OSD_POWER_Y",        1.0 },

    //OSD_AVG_CELL_VOLTAGE
    { "OSD_CELLVOLT_EN",       1.0 },
    { "OSD_CELLVOLT_X",        1.0 },
    { "OSD_CELLVOLT_Y",        1.0 },

    //OSD_DISARMED
    { "OSD_ARMING_EN",       1.0 },
    { "OSD_ARMING_X",        1.0 },
    { "OSD_ARMING_Y",        1.0 },

    //OSD_HOME_DIR
    { "OSD_HOMEDIR_EN",       1.0 },
    { "OSD_HOMEDIR_X",        1.0 },
    { "OSD_HOMEDIR_Y",        1.0 },

    //OSD_HOME_DIST
    { "OSD_HOMEDIST_EN",       1.0 },
    { "OSD_HOMEDIST_X",        1.0 },
    { "OSD_HOMEDIST_Y",        1.0 },
    */
};


extern const AP_HAL::HAL &hal;

// initialise backend
bool AP_OSD_MSP::init(void)
{
    return true;
}

// override built in positions with defaults for MSP OSD
void AP_OSD_MSP::setup_defaults(void)
{
    AP_Param::set_defaults_from_table(defaults_table, ARRAY_SIZE(defaults_table));
}

AP_OSD_Backend *AP_OSD_MSP::probe(AP_OSD &osd)
{
    AP_OSD_MSP *backend = new AP_OSD_MSP(osd);
    if (!backend) {
        return nullptr;
    }
    if (!backend->init()) {
        delete backend;
        return nullptr;
    }
    return backend;
}
