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
 * AP_OSD partially based on betaflight and inav osd.c implemention.
 * clarity.mcm font is taken from inav configurator.
 * Many thanks to their authors.
 */
/*
  parameter settings for one screen
 */
#include "AP_OSD_config.h"

#if OSD_ENABLED

#include "AP_OSD.h"
#include "AP_OSD_Backend.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Util.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Math/AP_Math.h>
#include <AP_RSSI/AP_RSSI.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Stats/AP_Stats.h>
#include <AP_Common/Location.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_MSP/msp.h>
#include <AP_OLC/AP_OLC.h>
#include <AP_VideoTX/AP_VideoTX.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_RPM/AP_RPM.h>
#if APM_BUILD_TYPE(APM_BUILD_Rover)
#include <AP_WindVane/AP_WindVane.h>
#endif
#include <AP_Filesystem/AP_Filesystem.h>

#include <ctype.h>
#include <GCS_MAVLink/GCS.h>
#include <AC_Fence/AC_Fence.h>

const AP_Param::GroupInfo AP_OSD_Screen::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable screen
    // @Description: Enable this screen
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_OSD_Screen, enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: CHAN_MIN
    // @DisplayName: Transmitter switch screen minimum pwm
    // @Description: This sets the PWM lower limit for this screen
    // @Range: 900 2100
    // @User: Standard
    AP_GROUPINFO("CHAN_MIN", 2, AP_OSD_Screen, channel_min, 900),

    // @Param: CHAN_MAX
    // @DisplayName: Transmitter switch screen maximum pwm
    // @Description: This sets the PWM upper limit for this screen
    // @Range: 900 2100
    // @User: Standard
    AP_GROUPINFO("CHAN_MAX", 3, AP_OSD_Screen, channel_max, 2100),

    // @Param: ALTITUDE_EN
    // @DisplayName: ALTITUDE_EN
    // @Description: Enables display of altitude AGL
    // @Values: 0:Disabled,1:Enabled

    // @Param: ALTITUDE_X
    // @DisplayName: ALTITUDE_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: ALTITUDE_Y
    // @DisplayName: ALTITUDE_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(altitude, "ALTITUDE", 4, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: BAT_VOLT_EN
    // @DisplayName: BATVOLT_EN
    // @Description: Displays main battery voltage
    // @Values: 0:Disabled,1:Enabled

    // @Param: BAT_VOLT_X
    // @DisplayName: BATVOLT_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: BAT_VOLT_Y
    // @DisplayName: BATVOLT_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(bat_volt, "BAT_VOLT", 5, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: RSSI_EN
    // @DisplayName: RSSI_EN
    // @Description: Displays RC signal strength
    // @Values: 0:Disabled,1:Enabled

    // @Param: RSSI_X
    // @DisplayName: RSSI_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: RSSI_Y
    // @DisplayName: RSSI_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(rssi, "RSSI", 6, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: CURRENT_EN
    // @DisplayName: CURRENT_EN
    // @Description: Displays main battery current
    // @Values: 0:Disabled,1:Enabled

    // @Param: CURRENT_X
    // @DisplayName: CURRENT_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: CURRENT_Y
    // @DisplayName: CURRENT_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(current, "CURRENT", 7, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: BATUSED_EN
    // @DisplayName: BATUSED_EN
    // @Description: Displays primary battery mAh consumed
    // @Values: 0:Disabled,1:Enabled

    // @Param: BATUSED_X
    // @DisplayName: BATUSED_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: BATUSED_Y
    // @DisplayName: BATUSED_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(batused, "BATUSED", 8, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: SATS_EN
    // @DisplayName: SATS_EN
    // @Description: Displays number of acquired satellites
    // @Values: 0:Disabled,1:Enabled

    // @Param: SATS_X
    // @DisplayName: SATS_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: SATS_Y
    // @DisplayName: SATS_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(sats, "SATS", 9, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: FLTMODE_EN
    // @DisplayName: FLTMODE_EN
    // @Description: Displays flight mode
    // @Values: 0:Disabled,1:Enabled

    // @Param: FLTMODE_X
    // @DisplayName: FLTMODE_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: FLTMODE_Y
    // @DisplayName: FLTMODE_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(fltmode, "FLTMODE", 10, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: MESSAGE_EN
    // @DisplayName: MESSAGE_EN
    // @Description: Displays Mavlink messages
    // @Values: 0:Disabled,1:Enabled

    // @Param: MESSAGE_X
    // @DisplayName: MESSAGE_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: MESSAGE_Y
    // @DisplayName: MESSAGE_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(message, "MESSAGE", 11, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: GSPEED_EN
    // @DisplayName: GSPEED_EN
    // @Description: Displays GPS ground speed
    // @Values: 0:Disabled,1:Enabled

    // @Param: GSPEED_X
    // @DisplayName: GSPEED_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: GSPEED_Y
    // @DisplayName: GSPEED_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(gspeed, "GSPEED", 12, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: HORIZON_EN
    // @DisplayName: HORIZON_EN
    // @Description: Displays artificial horizon
    // @Values: 0:Disabled,1:Enabled

    // @Param: HORIZON_X
    // @DisplayName: HORIZON_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: HORIZON_Y
    // @DisplayName: HORIZON_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(horizon, "HORIZON", 13, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: HOME_EN
    // @DisplayName: HOME_EN
    // @Description: Displays distance and relative direction to HOME
    // @Values: 0:Disabled,1:Enabled

    // @Param: HOME_X
    // @DisplayName: HOME_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: HOME_Y
    // @DisplayName: HOME_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(home, "HOME", 14, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: HEADING_EN
    // @DisplayName: HEADING_EN
    // @Description: Displays heading
    // @Values: 0:Disabled,1:Enabled

    // @Param: HEADING_X
    // @DisplayName: HEADING_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: HEADING_Y
    // @DisplayName: HEADING_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(heading, "HEADING", 15, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: THROTTLE_EN
    // @DisplayName: THROTTLE_EN
    // @Description: Displays actual throttle percentage being sent to motor(s)
    // @Values: 0:Disabled,1:Enabled

    // @Param: THROTTLE_X
    // @DisplayName: THROTTLE_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: THROTTLE_Y
    // @DisplayName: THROTTLE_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(throttle, "THROTTLE", 16, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: COMPASS_EN
    // @DisplayName: COMPASS_EN
    // @Description: Enables display of compass rose
    // @Values: 0:Disabled,1:Enabled

    // @Param: COMPASS_X
    // @DisplayName: COMPASS_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: COMPASS_Y
    // @DisplayName: COMPASS_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(compass, "COMPASS", 17, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: WIND_EN
    // @DisplayName: WIND_EN
    // @Description: Displays wind speed and relative direction, on Rover this is the apparent wind speed and direction from the windvane, if fitted
    // @Values: 0:Disabled,1:Enabled

    // @Param: WIND_X
    // @DisplayName: WIND_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: WIND_Y
    // @DisplayName: WIND_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(wind, "WIND", 18, AP_OSD_Screen, AP_OSD_Setting),


    // @Param: ASPEED_EN
    // @DisplayName: ASPEED_EN
    // @Description: Displays airspeed value being used by TECS (fused value)
    // @Values: 0:Disabled,1:Enabled

    // @Param: ASPEED_X
    // @DisplayName: ASPEED_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: ASPEED_Y
    // @DisplayName: ASPEED_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(aspeed, "ASPEED", 19, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: VSPEED_EN
    // @DisplayName: VSPEED_EN
    // @Description: Displays climb rate
    // @Values: 0:Disabled,1:Enabled

    // @Param: VSPEED_X
    // @DisplayName: VSPEED_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: VSPEED_Y
    // @DisplayName: VSPEED_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(vspeed, "VSPEED", 20, AP_OSD_Screen, AP_OSD_Setting),

#if HAL_WITH_ESC_TELEM
    // @Param: ESCTEMP_EN
    // @DisplayName: ESCTEMP_EN
    // @Description: Displays first esc's temp
    // @Values: 0:Disabled,1:Enabled

    // @Param: ESCTEMP_X
    // @DisplayName: ESCTEMP_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: ESCTEMP_Y
    // @DisplayName: ESCTEMP_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(esc_temp, "ESCTEMP", 21, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: ESCRPM_EN
    // @DisplayName: ESCRPM_EN
    // @Description: Displays first esc's rpm
    // @Values: 0:Disabled,1:Enabled

    // @Param: ESCRPM_X
    // @DisplayName: ESCRPM_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: ESCRPM_Y
    // @DisplayName: ESCRPM_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(esc_rpm, "ESCRPM", 22, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: ESCAMPS_EN
    // @DisplayName: ESCAMPS_EN
    // @Description: Displays first esc's current
    // @Values: 0:Disabled,1:Enabled

    // @Param: ESCAMPS_X
    // @DisplayName: ESCAMPS_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: ESCAMPS_Y
    // @DisplayName: ESCAMPS_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(esc_amps, "ESCAMPS", 23, AP_OSD_Screen, AP_OSD_Setting),
#endif
    // @Param: GPSLAT_EN
    // @DisplayName: GPSLAT_EN
    // @Description: Displays GPS latitude
    // @Values: 0:Disabled,1:Enabled

    // @Param: GPSLAT_X
    // @DisplayName: GPSLAT_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: GPSLAT_Y
    // @DisplayName: GPSLAT_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(gps_latitude, "GPSLAT", 24, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: GPSLONG_EN
    // @DisplayName: GPSLONG_EN
    // @Description: Displays GPS longitude
    // @Values: 0:Disabled,1:Enabled

    // @Param: GPSLONG_X
    // @DisplayName: GPSLONG_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: GPSLONG_Y
    // @DisplayName: GPSLONG_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(gps_longitude, "GPSLONG", 25, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: ROLL_EN
    // @DisplayName: ROLL_EN
    // @Description: Displays degrees of roll from level
    // @Values: 0:Disabled,1:Enabled

    // @Param: ROLL_X
    // @DisplayName: ROLL_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: ROLL_Y
    // @DisplayName: ROLL_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(roll_angle, "ROLL", 26, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: PITCH_EN
    // @DisplayName: PITCH_EN
    // @Description: Displays degrees of pitch from level
    // @Values: 0:Disabled,1:Enabled

    // @Param: PITCH_X
    // @DisplayName: PITCH_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: PITCH_Y
    // @DisplayName: PITCH_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(pitch_angle, "PITCH", 27, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: TEMP_EN
    // @DisplayName: TEMP_EN
    // @Description: Displays temperature reported by primary barometer
    // @Values: 0:Disabled,1:Enabled

    // @Param: TEMP_X
    // @DisplayName: TEMP_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: TEMP_Y
    // @DisplayName: TEMP_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(temp, "TEMP", 28, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: HDOP_EN
    // @DisplayName: HDOP_EN
    // @Description: Displays Horizontal Dilution Of Position
    // @Values: 0:Disabled,1:Enabled

    // @Param: HDOP_X
    // @DisplayName: HDOP_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: HDOP_Y
    // @DisplayName: HDOP_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(hdop, "HDOP", 29, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: WAYPOINT_EN
    // @DisplayName: WAYPOINT_EN
    // @Description: Displays bearing and distance to next waypoint
    // @Values: 0:Disabled,1:Enabled

    // @Param: WAYPOINT_X
    // @DisplayName: WAYPOINT_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: WAYPOINT_Y
    // @DisplayName: WAYPOINT_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(waypoint, "WAYPOINT", 30, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: XTRACK_EN
    // @DisplayName: XTRACK_EN
    // @Description: Displays crosstrack error
    // @Values: 0:Disabled,1:Enabled

    // @Param: XTRACK_X
    // @DisplayName: XTRACK_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: XTRACK_Y
    // @DisplayName: XTRACK_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(xtrack_error, "XTRACK", 31, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: DIST_EN
    // @DisplayName: DIST_EN
    // @Description: Displays total distance flown
    // @Values: 0:Disabled,1:Enabled

    // @Param: DIST_X
    // @DisplayName: DIST_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: DIST_Y
    // @DisplayName: DIST_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(dist, "DIST", 32, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: STATS_EN
    // @DisplayName: STATS_EN
    // @Description: Displays flight stats
    // @Values: 0:Disabled,1:Enabled

    // @Param: STATS_X
    // @DisplayName: STATS_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: STATS_Y
    // @DisplayName: STATS_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(stat, "STATS", 33, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: FLTIME_EN
    // @DisplayName: FLTIME_EN
    // @Description: Displays total flight time
    // @Values: 0:Disabled,1:Enabled

    // @Param: FLTIME_X
    // @DisplayName: FLTIME_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: FLTIME_Y
    // @DisplayName: FLTIME_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(flightime, "FLTIME", 34, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: CLIMBEFF_EN
    // @DisplayName: CLIMBEFF_EN
    // @Description: Displays climb efficiency (climb rate/current)
    // @Values: 0:Disabled,1:Enabled

    // @Param: CLIMBEFF_X
    // @DisplayName: CLIMBEFF_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: CLIMBEFF_Y
    // @DisplayName: CLIMBEFF_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(climbeff, "CLIMBEFF", 35, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: EFF_EN
    // @DisplayName: EFF_EN
    // @Description: Displays flight efficiency (mAh/km or /mi)
    // @Values: 0:Disabled,1:Enabled

    // @Param: EFF_X
    // @DisplayName: EFF_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: EFF_Y
    // @DisplayName: EFF_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(eff, "EFF", 36, AP_OSD_Screen, AP_OSD_Setting),

#if BARO_MAX_INSTANCES > 1
    // @Param: BTEMP_EN
    // @DisplayName: BTEMP_EN
    // @Description: Displays temperature reported by secondary barometer
    // @Values: 0:Disabled,1:Enabled

    // @Param: BTEMP_X
    // @DisplayName: BTEMP_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: BTEMP_Y
    // @DisplayName: BTEMP_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(btemp, "BTEMP", 37, AP_OSD_Screen, AP_OSD_Setting),
#endif

    // @Param: ATEMP_EN
    // @DisplayName: ATEMP_EN
    // @Description: Displays temperature reported by primary airspeed sensor
    // @Values: 0:Disabled,1:Enabled

    // @Param: ATEMP_X
    // @DisplayName: ATEMP_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: ATEMP_Y
    // @DisplayName: ATEMP_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(atemp, "ATEMP", 38, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: BAT2_VLT_EN
    // @DisplayName: BAT2VLT_EN
    // @Description: Displays battery2 voltage
    // @Values: 0:Disabled,1:Enabled

    // @Param: BAT2_VLT_X
    // @DisplayName: BAT2VLT_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: BAT2_VLT_Y
    // @DisplayName: BAT2VLT_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(bat2_vlt, "BAT2_VLT", 39, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: BAT2USED_EN
    // @DisplayName: BAT2USED_EN
    // @Description: Displays secondary battery mAh consumed
    // @Values: 0:Disabled,1:Enabled

    // @Param: BAT2USED_X
    // @DisplayName: BAT2USED_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: BAT2USED_Y
    // @DisplayName: BAT2USED_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(bat2used, "BAT2USED", 40, AP_OSD_Screen, AP_OSD_Setting),


    // @Param: ASPD2_EN
    // @DisplayName: ASPD2_EN
    // @Description: Displays airspeed reported directly from secondary airspeed sensor
    // @Values: 0:Disabled,1:Enabled

    // @Param: ASPD2_X
    // @DisplayName: ASPD2_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: ASPD2_Y
    // @DisplayName: ASPD2_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(aspd2, "ASPD2", 41, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: ASPD1_EN
    // @DisplayName: ASPD1_EN
    // @Description: Displays airspeed reported directly from primary airspeed sensor
    // @Values: 0:Disabled,1:Enabled

    // @Param: ASPD1_X
    // @DisplayName: ASPD1_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: ASPD1_Y
    // @DisplayName: ASPD1_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(aspd1, "ASPD1", 42, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: CLK_EN
    // @DisplayName: CLK_EN
    // @Description: Displays a clock panel based on AP_RTC local time
    // @Values: 0:Disabled,1:Enabled

    // @Param: CLK_X
    // @DisplayName: CLK_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: CLK_Y
    // @DisplayName: CLK_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(clk, "CLK", 43, AP_OSD_Screen, AP_OSD_Setting),

#if HAL_OSD_SIDEBAR_ENABLE || HAL_MSP_ENABLED
    // @Param: SIDEBARS_EN
    // @DisplayName: SIDEBARS_EN
    // @Description: Displays artificial horizon side bars
    // @Values: 0:Disabled,1:Enabled

    // @Param: SIDEBARS_X
    // @DisplayName: SIDEBARS_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: SIDEBARS_Y
    // @DisplayName: SIDEBARS_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(sidebars, "SIDEBARS", 44, AP_OSD_Screen, AP_OSD_Setting),
#endif

#if HAL_MSP_ENABLED
    // @Param: CRSSHAIR_EN
    // @DisplayName: CRSSHAIR_EN
    // @Description: Displays artificial horizon crosshair (MSP OSD only)
    // @Values: 0:Disabled,1:Enabled

    // @Param: CRSSHAIR_X
    // @DisplayName: CRSSHAIR_X
    // @Description: Horizontal position on screen (MSP OSD only)
    // @Range: 0 29

    // @Param: CRSSHAIR_Y
    // @DisplayName: CRSSHAIR_Y
    // @Description: Vertical position on screen (MSP OSD only)
    // @Range: 0 15
    AP_SUBGROUPINFO(crosshair, "CRSSHAIR", 45, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: HOMEDIST_EN
    // @DisplayName: HOMEDIST_EN
    // @Description: Displays distance from HOME (MSP OSD only)
    // @Values: 0:Disabled,1:Enabled

    // @Param: HOMEDIST_X
    // @DisplayName: HOMEDIST_X
    // @Description: Horizontal position on screen (MSP OSD only)
    // @Range: 0 29

    // @Param: HOMEDIST_Y
    // @DisplayName: HOMEDIST_Y
    // @Description: Vertical position on screen (MSP OSD only)
    // @Range: 0 15
    AP_SUBGROUPINFO(home_dist, "HOMEDIST", 46, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: HOMEDIR_EN
    // @DisplayName: HOMEDIR_EN
    // @Description: Displays relative direction to HOME (MSP OSD only)
    // @Values: 0:Disabled,1:Enabled

    // @Param: HOMEDIR_X
    // @DisplayName: HOMEDIR_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: HOMEDIR_Y
    // @DisplayName: HOMEDIR_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(home_dir, "HOMEDIR", 47, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: POWER_EN
    // @DisplayName: POWER_EN
    // @Description: Displays power (MSP OSD only)
    // @Values: 0:Disabled,1:Enabled

    // @Param: POWER_X
    // @DisplayName: POWER_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: POWER_Y
    // @DisplayName: POWER_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(power, "POWER", 48, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: CELLVOLT_EN
    // @DisplayName: CELL_VOLT_EN
    // @Description: Displays average cell voltage (MSP OSD only)
    // @Values: 0:Disabled,1:Enabled

    // @Param: CELLVOLT_X
    // @DisplayName: CELL_VOLT_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: CELLVOLT_Y
    // @DisplayName: CELL_VOLT_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(cell_volt, "CELLVOLT", 49, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: BATTBAR_EN
    // @DisplayName: BATT_BAR_EN
    // @Description: Displays battery usage bar (MSP OSD only)
    // @Values: 0:Disabled,1:Enabled

    // @Param: BATTBAR_X
    // @DisplayName: BATT_BAR_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: BATTBAR_Y
    // @DisplayName: BATT_BAR_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(batt_bar, "BATTBAR", 50, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: ARMING_EN
    // @DisplayName: ARMING_EN
    // @Description: Displays arming status (MSP OSD only)
    // @Values: 0:Disabled,1:Enabled

    // @Param: ARMING_X
    // @DisplayName: ARMING_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: ARMING_Y
    // @DisplayName: ARMING_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(arming, "ARMING", 51, AP_OSD_Screen, AP_OSD_Setting),
#endif //HAL_MSP_ENABLED

#if HAL_PLUSCODE_ENABLE
    // @Param: PLUSCODE_EN
    // @DisplayName: PLUSCODE_EN
    // @Description: Displays pluscode (OLC) element
    // @Values: 0:Disabled,1:Enabled

    // @Param: PLUSCODE_X
    // @DisplayName: PLUSCODE_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: PLUSCODE_Y
    // @DisplayName: PLUSCODE_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(pluscode, "PLUSCODE", 52, AP_OSD_Screen, AP_OSD_Setting),
#endif

#if AP_OSD_CALLSIGN_FROM_SD_ENABLED
    // @Param: CALLSIGN_EN
    // @DisplayName: CALLSIGN_EN
    // @Description: Displays callsign from callsign.txt on microSD card
    // @Values: 0:Disabled,1:Enabled

    // @Param: CALLSIGN_X
    // @DisplayName: CALLSIGN_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: CALLSIGN_Y
    // @DisplayName: CALLSIGN_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(callsign, "CALLSIGN", 53, AP_OSD_Screen, AP_OSD_Setting),
#endif

    // @Param: CURRENT2_EN
    // @DisplayName: CURRENT2_EN
    // @Description: Displays 2nd battery current
    // @Values: 0:Disabled,1:Enabled

    // @Param: CURRENT2_X
    // @DisplayName: CURRENT2_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: CURRENT2_Y
    // @DisplayName: CURRENT2_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(current2, "CURRENT2", 54, AP_OSD_Screen, AP_OSD_Setting),

#if AP_VIDEOTX_ENABLED
    // @Param: VTX_PWR_EN
    // @DisplayName: VTX_PWR_EN
    // @Description: Displays VTX Power
    // @Values: 0:Disabled,1:Enabled

    // @Param: VTX_PWR_X
    // @DisplayName: VTX_PWR_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: VTX_PWR_Y
    // @DisplayName: VTX_PWR_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(vtx_power, "VTX_PWR", 55, AP_OSD_Screen, AP_OSD_Setting),
#endif  // AP_VIDEOTX_ENABLED

#if AP_TERRAIN_AVAILABLE
    // @Param: TER_HGT_EN
    // @DisplayName: TER_HGT_EN
    // @Description: Displays Height above terrain
    // @Values: 0:Disabled,1:Enabled

    // @Param: TER_HGT_X
    // @DisplayName: TER_HGT_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: TER_HGT_Y
    // @DisplayName: TER_HGT_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(hgt_abvterr, "TER_HGT", 56, AP_OSD_Screen, AP_OSD_Setting),
#endif

    // @Param: AVGCELLV_EN
    // @DisplayName: AVGCELLV_EN
    // @Description: Displays average cell voltage. WARNING: this can be inaccurate if the cell count is not detected or set properly. If the  the battery is far from fully charged the detected cell count might not be accurate if auto cell count detection is used (OSD_CELL_COUNT=0).
    // @Values: 0:Disabled,1:Enabled

    // @Param: AVGCELLV_X
    // @DisplayName: AVGCELLV_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: AVGCELLV_Y
    // @DisplayName: AVGCELLV_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(avgcellvolt, "AVGCELLV", 57, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: RESTVOLT_EN
    // @DisplayName: RESTVOLT_EN
    // @Description: Displays main battery resting voltage
    // @Values: 0:Disabled,1:Enabled

    // @Param: RESTVOLT_X
    // @DisplayName: RESTVOLT_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: RESTVOLT_Y
    // @DisplayName: RESTVOLT_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(restvolt, "RESTVOLT", 58, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: FENCE_EN
    // @DisplayName: FENCE_EN
    // @Description: Displays indication of fence enable and breach
    // @Values: 0:Disabled,1:Enabled

    // @Param: FENCE_X
    // @DisplayName: FENCE_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: FENCE_Y
    // @DisplayName: FENCE_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(fence, "FENCE", 59, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: RNGF_EN
    // @DisplayName: RNGF_EN
    // @Description: Displays a rangefinder's distance in cm
    // @Values: 0:Disabled,1:Enabled

    // @Param: RNGF_X
    // @DisplayName: RNGF_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: RNGF_Y
    // @DisplayName: RNGF_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(rngf, "RNGF", 60, AP_OSD_Screen, AP_OSD_Setting),

    // @Param: ACRVOLT_EN
    // @DisplayName: ACRVOLT_EN
    // @Description: Displays resting voltage for the average cell. WARNING: this can be inaccurate if the cell count is not detected or set properly. If the  the battery is far from fully charged the detected cell count might not be accurate if auto cell count detection is used (OSD_CELL_COUNT=0).
    // @Values: 0:Disabled,1:Enabled

    // @Param: ACRVOLT_X
    // @DisplayName: ACRVOLT_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: ACRVOLT_Y
    // @DisplayName: ACRVOLT_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(avgcellrestvolt, "ACRVOLT", 61, AP_OSD_Screen, AP_OSD_Setting),

#if AP_RPM_ENABLED
	// @Param: RPM_EN
	// @DisplayName: RPM_EN
	// @Description: Displays main rotor revs/min
	// @Values: 0:Disabled,1:Enabled

	// @Param: RPM_X
	// @DisplayName: RPM_X
	// @Description: Horizontal position on screen
	// @Range: 0 29

	// @Param: RPM_Y
	// @DisplayName: RPM_Y
	// @Description: Vertical position on screen
	// @Range: 0 15
	AP_SUBGROUPINFO(rrpm, "RPM", 62, AP_OSD_Screen, AP_OSD_Setting),
#endif

    AP_GROUPEND
};

const AP_Param::GroupInfo AP_OSD_Screen::var_info2[] = {
    // duplicate of OSDn_ENABLE to ensure params are hidden when not enabled
    AP_GROUPINFO_FLAGS("ENABLE", 2, AP_OSD_Screen, enabled, 0, AP_PARAM_FLAG_ENABLE | AP_PARAM_FLAG_HIDDEN),

    // @Param: LINK_Q_EN
    // @DisplayName: LINK_Q_EN
    // @Description: Displays Receiver link quality
    // @Values: 0:Disabled,1:Enabled

    // @Param: LINK_Q_X
    // @DisplayName: LINK_Q_X
    // @Description: Horizontal position on screen
    // @Range: 0 29

    // @Param: LINK_Q_Y
    // @DisplayName: LINK_Q_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(link_quality, "LINK_Q", 1, AP_OSD_Screen, AP_OSD_Setting),

#if HAL_WITH_MSP_DISPLAYPORT
    // @Param: TXT_RES
    // @DisplayName: Sets the overlay text resolution (MSP DisplayPort only)
    // @Description: Sets the overlay text resolution for this screen to either LD 30x16 or HD 50x18 (MSP DisplayPort only)
    // @Values: 0:30x16,1:50x18
    // @User: Standard
    AP_GROUPINFO("TXT_RES", 3, AP_OSD_Screen, txt_resolution, 0),

    // @Param: FONT
    // @DisplayName: Sets the font index for this screen (MSP DisplayPort only)
    // @Description: Sets the font index for this screen (MSP DisplayPort only)
    // @Range: 0 15
    // @User: Standard
    AP_GROUPINFO("FONT", 4, AP_OSD_Screen, font_index, 0),
#endif
    AP_GROUPEND
};


uint8_t AP_OSD_AbstractScreen::symbols_lookup_table[AP_OSD_NUM_SYMBOLS];

// Symbol indexes to acces _symbols[index][set]
#define SYM_M 0
#define SYM_KM 1
#define SYM_FT 2
#define SYM_MI 3
#define SYM_ALT_M 4
#define SYM_ALT_FT 5
#define SYM_BATT_FULL 6
#define SYM_RSSI 7

#define SYM_VOLT 8
#define SYM_AMP 9
#define SYM_MAH 10
#define SYM_MS 11
#define SYM_FS 12
#define SYM_KMH 13
#define SYM_MPH 14
#define SYM_DEGR 15
#define SYM_PCNT 16
#define SYM_RPM 17
#define SYM_ASPD 18
#define SYM_GSPD 19
#define SYM_WSPD 20
#define SYM_VSPD 21
#define SYM_WPNO 22
#define SYM_WPDIR 23
#define SYM_WPDST 24
#define SYM_FTMIN 25
#define SYM_FTSEC 26

#define SYM_SAT_L 27
#define SYM_SAT_R 28
#define SYM_HDOP_L 29
#define SYM_HDOP_R 30

#define SYM_HOME 31
#define SYM_WIND 32

#define SYM_ARROW_START 33
#define SYM_ARROW_COUNT 34
#define SYM_AH_H_START 35
#define SYM_AH_H_COUNT 36

#define SYM_AH_V_START 37
#define SYM_AH_V_COUNT 38

#define SYM_AH_CENTER_LINE_LEFT 39
#define SYM_AH_CENTER_LINE_RIGHT 40
#define SYM_AH_CENTER 41

#define SYM_HEADING_N 42
#define SYM_HEADING_S 43
#define SYM_HEADING_E 44
#define SYM_HEADING_W 45
#define SYM_HEADING_DIVIDED_LINE 46
#define SYM_HEADING_LINE 47

#define SYM_UP_UP 48
#define SYM_UP 49
#define SYM_DOWN 50
#define SYM_DOWN_DOWN 51

#define SYM_DEGREES_C 52
#define SYM_DEGREES_F 53
#define SYM_GPS_LAT 54
#define SYM_GPS_LONG 55
#define SYM_ARMED 56
#define SYM_DISARMED 57
#define SYM_ROLL0 58
#define SYM_ROLLR 59
#define SYM_ROLLL 60
#define SYM_PTCH0 61
#define SYM_PTCHUP 62
#define SYM_PTCHDWN 63
#define SYM_XERR 64
#define SYM_KN 65
#define SYM_NM 66
#define SYM_DIST 67
#define SYM_FLY 68
#define SYM_EFF 69
#define SYM_AH 70
#define SYM_MW 71
#define SYM_CLK 72
#define SYM_KILO 73
#define SYM_TERALT 74
#define SYM_FENCE_ENABLED 75
#define SYM_FENCE_DISABLED 76
#define SYM_RNGFD 77
#define SYM_LQ 78

#define SYM_SIDEBAR_R_ARROW 79
#define SYM_SIDEBAR_L_ARROW 80
#define SYM_SIDEBAR_A 81
#define SYM_SIDEBAR_B 82
#define SYM_SIDEBAR_C 83
#define SYM_SIDEBAR_D 84
#define SYM_SIDEBAR_E 85
#define SYM_SIDEBAR_F 86
#define SYM_SIDEBAR_G 87
#define SYM_SIDEBAR_H 88
#define SYM_SIDEBAR_I 89
#define SYM_SIDEBAR_J 90

#define SYMBOL(n) AP_OSD_AbstractScreen::symbols_lookup_table[n]

// constructor
AP_OSD_Screen::AP_OSD_Screen()
{
    AP_Param::setup_object_defaults(this, var_info);
    AP_Param::setup_object_defaults(this, var_info2);
}

void AP_OSD_AbstractScreen::set_backend(AP_OSD_Backend *_backend)
{
    backend = _backend;
    osd = _backend->get_osd();
};

bool AP_OSD_AbstractScreen::check_option(uint32_t option)
{
    return osd?(osd->options & option) != 0 : false;
}

/*
  get the right units icon given a unit
 */
char AP_OSD_AbstractScreen::u_icon(enum unit_type unit)
{
    static const uint8_t icons_metric[UNIT_TYPE_LAST] {
        SYM_ALT_M,    //ALTITUDE
        SYM_KMH,      //SPEED
        SYM_MS,       //VSPEED
        SYM_M,        //DISTANCE
        SYM_KM,       //DISTANCE_LONG
        SYM_DEGREES_C //TEMPERATURE
    };
    static const uint8_t icons_imperial[UNIT_TYPE_LAST] {
        SYM_ALT_FT,   //ALTITUDE
        SYM_MPH,      //SPEED
        SYM_FS,       //VSPEED
        SYM_FT,       //DISTANCE
        SYM_MI,       //DISTANCE_LONG
        SYM_DEGREES_F //TEMPERATURE
    };
    static const uint8_t icons_SI[UNIT_TYPE_LAST] {
        SYM_ALT_M,    //ALTITUDE
        SYM_MS,       //SPEED
        SYM_MS,       //VSPEED
        SYM_M,        //DISTANCE
        SYM_KM,       //DISTANCE_LONG
        SYM_DEGREES_C //TEMPERATURE
    };
    static const uint8_t icons_aviation[UNIT_TYPE_LAST] {
        SYM_ALT_FT,   //ALTITUDE Ft
        SYM_KN,       //SPEED Knots
        SYM_FTMIN,    //VSPEED
        SYM_FT,       //DISTANCE
        SYM_NM,       //DISTANCE_LONG Nm
        SYM_DEGREES_C //TEMPERATURE
    };
    static const uint8_t* icons[AP_OSD::UNITS_LAST] = {
        icons_metric,
        icons_imperial,
        icons_SI,
        icons_aviation,
    };
    return (char)SYMBOL(icons[constrain_int16(osd->units, 0, AP_OSD::UNITS_LAST-1)][unit]);
}

/*
  scale a value for the user selected units
 */
float AP_OSD_AbstractScreen::u_scale(enum unit_type unit, float value)
{
    static const float scale_metric[UNIT_TYPE_LAST] = {
        1.0,       //ALTITUDE m
        3.6,       //SPEED km/hr
        1.0,       //VSPEED m/s
        1.0,       //DISTANCE m
        1.0/1000,  //DISTANCE_LONG km
        1.0,       //TEMPERATURE C
    };
    static const float scale_imperial[UNIT_TYPE_LAST] = {
        3.28084,     //ALTITUDE ft
        2.23694,     //SPEED mph
        3.28084,     //VSPEED ft/s
        3.28084,     //DISTANCE ft
        1.0/1609.34, //DISTANCE_LONG miles
        1.8,         //TEMPERATURE F
    };
    static const float offset_imperial[UNIT_TYPE_LAST] = {
        0.0,          //ALTITUDE
        0.0,          //SPEED
        0.0,          //VSPEED
        0.0,          //DISTANCE
        0.0,          //DISTANCE_LONG
        32.0,         //TEMPERATURE F
    };
    static const float scale_SI[UNIT_TYPE_LAST] = {
        1.0,       //ALTITUDE m
        1.0,       //SPEED m/s
        1.0,       //VSPEED m/s
        1.0,       //DISTANCE m
        1.0/1000,  //DISTANCE_LONG km
        1.0,       //TEMPERATURE C
    };
    static const float scale_aviation[UNIT_TYPE_LAST] = {
        3.28084,   //ALTITUDE Ft
        1.94384,   //SPEED Knots
        196.85,    //VSPEED ft/min
        3.28084,   //DISTANCE ft
        0.000539957,  //DISTANCE_LONG Nm
        1.0,       //TEMPERATURE C
    };
    static const float *scale[AP_OSD::UNITS_LAST] = {
        scale_metric,
        scale_imperial,
        scale_SI,
        scale_aviation
    };
    static const float *offsets[AP_OSD::UNITS_LAST] = {
        nullptr,
        offset_imperial,
        nullptr,
        nullptr
    };
    uint8_t units = constrain_int16(osd->units, 0, AP_OSD::UNITS_LAST-1);
    return value * scale[units][unit] + (offsets[units]?offsets[units][unit]:0);
}

char AP_OSD_Screen::get_arrow_font_index(int32_t angle_cd)
{
    uint32_t interval = 36000 / SYMBOL(SYM_ARROW_COUNT);
    angle_cd = wrap_360_cd(angle_cd);
    // if using BF font table must translate arrows
    if (check_option(AP_OSD::OPTION_BF_ARROWS)) {
        angle_cd = angle_cd > 18000? 54000 - angle_cd : 18000- angle_cd;
    } 
    return SYMBOL(SYM_ARROW_START) + ((angle_cd + interval / 2) / interval) % SYMBOL(SYM_ARROW_COUNT);
}

void AP_OSD_Screen::draw_altitude(uint8_t x, uint8_t y)
{
    float alt;
    AP_AHRS &ahrs = AP::ahrs();
    WITH_SEMAPHORE(ahrs.get_semaphore());
    ahrs.get_relative_position_D_home(alt);
    alt = -alt;
    backend->write(x, y, false, "%4d%c", (int)u_scale(ALTITUDE, alt), u_icon(ALTITUDE));
}

void AP_OSD_Screen::draw_bat_volt(uint8_t instance, VoltageType type, uint8_t x, uint8_t y)
{
    AP_BattMonitor &battery = AP::battery();
    float v = battery.voltage(instance);
    float blinkvolt = osd->warn_batvolt;
    uint8_t pct;
    bool show_remaining_pct = battery.capacity_remaining_pct(pct);
    uint8_t p = (100 - pct) / 16.6;
    switch (type) {
    case VoltageType::VOLTAGE: {
        break;
    }
    case VoltageType::RESTING_VOLTAGE: {
        v = battery.voltage_resting_estimate(instance);
        blinkvolt = osd->warn_restvolt;
        break;
    }
    case VoltageType::RESTING_CELL: { 
        blinkvolt = osd->warn_avgcellrestvolt;
        v = battery.voltage_resting_estimate(instance);
         FALLTHROUGH;
    }
    case VoltageType::AVG_CELL: {         
       if (type == VoltageType::AVG_CELL) { //for fallthrough of RESTING_CELL
            blinkvolt = osd->warn_avgcellvolt;
       }
       // calculate cell count - WARNING this can be inaccurate if the LIPO/LIION  battery is far from 
       // fully charged when attached and is used in this panel
       osd->max_battery_voltage.set(MAX(osd->max_battery_voltage,v));
       if (osd->cell_count > 0) {
           v = v / osd->cell_count;
       } else if (osd->cell_count < 0) { // user must decide on autodetect cell count or manually entered to display this panel since default is -1
           backend->write(x,y, false, "%c---%c", SYMBOL(SYM_BATT_FULL) + p, SYMBOL(SYM_VOLT));
           return;
       } else {  // use autodetected cell count
            v = v /  (uint8_t)(osd->max_battery_voltage * 0.2381 + 1);
       }
       break;
    }
    }    
    if (!show_remaining_pct) {
        // Do not show battery percentage
        backend->write(x,y, v < blinkvolt, "%2.1f%c", (double)v, SYMBOL(SYM_VOLT));
        return;
    }
    backend->write(x,y, v < blinkvolt, "%c%2.1f%c", SYMBOL(SYM_BATT_FULL) + p, (double)v, SYMBOL(SYM_VOLT));
}

void AP_OSD_Screen::draw_bat_volt(uint8_t x, uint8_t y)
{
    draw_bat_volt(0,VoltageType::VOLTAGE,x,y);
}

void AP_OSD_Screen::draw_avgcellvolt(uint8_t x, uint8_t y)
{
    draw_bat_volt(0,VoltageType::AVG_CELL,x,y);
}

void AP_OSD_Screen::draw_avgcellrestvolt(uint8_t x, uint8_t y)
{
    draw_bat_volt(0,VoltageType::RESTING_CELL,x, y);
}

void AP_OSD_Screen::draw_restvolt(uint8_t x, uint8_t y)
{
    draw_bat_volt(0,VoltageType::RESTING_VOLTAGE,x,y);
}

void AP_OSD_Screen::draw_rssi(uint8_t x, uint8_t y)
{
    AP_RSSI *ap_rssi = AP_RSSI::get_singleton();
    if (ap_rssi) {
        const uint8_t rssiv = ap_rssi->read_receiver_rssi() * 100;
        backend->write(x, y, rssiv < osd->warn_rssi, "%c%2d", SYMBOL(SYM_RSSI), rssiv);
    }
}

void AP_OSD_Screen::draw_link_quality(uint8_t x, uint8_t y)
{
    AP_RSSI *ap_rssi = AP_RSSI::get_singleton();
    if (ap_rssi) {
        const int16_t lqv = ap_rssi->read_receiver_link_quality();
        if (lqv < 0){
            backend->write(x, y, false, "%c--", SYMBOL(SYM_LQ));
        } else {
            backend->write(x, y, false, "%c%2d", SYMBOL(SYM_LQ), lqv);
        }
    }
}

void AP_OSD_Screen::draw_current(uint8_t instance, uint8_t x, uint8_t y)
{
    float amps;
    if (!AP::battery().current_amps(amps, instance)) {
        osd->_stats.avg_current_a = 0;
    }
    //filter current and display with autoranging for low values
    osd->_stats.avg_current_a= osd->_stats.avg_current_a + (amps - osd->_stats.avg_current_a) * 0.33;
    if (osd->_stats.avg_current_a < 10.0) {
        backend->write(x, y, false, "%2.2f%c", osd->_stats.avg_current_a, SYMBOL(SYM_AMP));
    }
    else {
        backend->write(x, y, false, "%2.1f%c", osd->_stats.avg_current_a, SYMBOL(SYM_AMP));
    }
}

void AP_OSD_Screen::draw_current(uint8_t x, uint8_t y)
{
    draw_current(0, x, y);
}

void AP_OSD_Screen::draw_fltmode(uint8_t x, uint8_t y)
{
    AP_Notify * notify = AP_Notify::get_singleton();
    char arm;
    if (AP_Notify::flags.armed) {
        arm = SYMBOL(SYM_ARMED);
    } else {
        arm = SYMBOL(SYM_DISARMED);
    }
    if (notify) {
        backend->write(x, y, false, "%s%c", notify->get_flight_mode_str(), arm);
    }
}

void AP_OSD_Screen::draw_sats(uint8_t x, uint8_t y)
{
    AP_GPS & gps = AP::gps();
    uint8_t nsat = gps.num_sats();
    bool flash = (nsat < osd->warn_nsat) || (gps.status() < AP_GPS::GPS_OK_FIX_3D);
    backend->write(x, y, flash, "%c%c%2u", SYMBOL(SYM_SAT_L), SYMBOL(SYM_SAT_R), nsat);
}

void AP_OSD_Screen::draw_batused(uint8_t instance, uint8_t x, uint8_t y)
{
    float mah;
    if (!AP::battery().consumed_mah(mah, instance)) {
        mah = 0;
    }
    if (mah <= 9999) {
        backend->write(x,y, false, "%4d%c", (int)mah, SYMBOL(SYM_MAH));
    } else {
        const float ah = mah * 1e-3f;
        backend->write(x,y, false, "%2.2f%c", (double)ah, SYMBOL(SYM_AH));
    }
}

void AP_OSD_Screen::draw_batused(uint8_t x, uint8_t y)
{
    draw_batused(0, x, y);
}

//Autoscroll message is the same as in minimosd-extra.
//Thanks to night-ghost for the approach.
void AP_OSD_Screen::draw_message(uint8_t x, uint8_t y)
{
    AP_Notify * notify = AP_Notify::get_singleton();
    if (notify) {
        int32_t visible_time = AP_HAL::millis() - notify->get_text_updated_millis();
        if (visible_time < osd->msgtime_s *1000) {
            char buffer[NOTIFY_TEXT_BUFFER_SIZE];
            strncpy(buffer, notify->get_text(), sizeof(buffer));
            int16_t len = strnlen(buffer, sizeof(buffer));

            for (int16_t i=0; i<len; i++) {
                //converted to uppercase,
                //because we do not have small letter chars inside used font
                buffer[i] = toupper(buffer[i]);
                //normalize whitespace
                if (isspace(buffer[i])) {
                    buffer[i] = ' ';
                }
            }

            int16_t start_position = 0;
            //scroll if required
            //scroll pattern: wait, scroll to the left, wait, scroll to the right
            if (len > message_visible_width) {
                int16_t chars_to_scroll = len - message_visible_width;
                int16_t total_cycles = 2*message_scroll_delay + 2*chars_to_scroll;
                int16_t current_cycle = (visible_time / message_scroll_time_ms) % total_cycles;

                //calculate scroll start_position
                if (current_cycle < total_cycles/2) {
                    //move to the left
                    start_position = current_cycle - message_scroll_delay;
                } else {
                    //move to the right
                    start_position = total_cycles - current_cycle;
                }
                start_position = constrain_int16(start_position, 0, chars_to_scroll);
                int16_t end_position = start_position + message_visible_width;

                //ensure array boundaries
                start_position = MIN(start_position, int(sizeof(buffer)-1));
                end_position = MIN(end_position, int(sizeof(buffer)-1));

                //trim invisible part
                buffer[end_position] = 0;
            }

            backend->write(x, y, buffer + start_position);
        }
    }
}

// draw a arrow at the given angle, and print the given magnitude
void AP_OSD_Screen::draw_speed(uint8_t x, uint8_t y, float angle_rad, float magnitude)
{
    int32_t angle_cd = angle_rad * DEGX100;
    char arrow = get_arrow_font_index(angle_cd);
    if (u_scale(SPEED, magnitude) < 9.95) {
        backend->write(x, y, false, "%c %1.1f%c", arrow, u_scale(SPEED, magnitude), u_icon(SPEED));
    } else {
        backend->write(x, y, false, "%c%3d%c", arrow, (int)roundf(u_scale(SPEED, magnitude)), u_icon(SPEED));
    }
}

void AP_OSD_Screen::draw_gspeed(uint8_t x, uint8_t y)
{
    AP_AHRS &ahrs = AP::ahrs();
    WITH_SEMAPHORE(ahrs.get_semaphore());
    Vector2f v = ahrs.groundspeed_vector();
    backend->write(x, y, false, "%c", SYMBOL(SYM_GSPD));
    float angle = 0;
    const float length = v.length();
    if (length > 1.0f) {
        angle = atan2f(v.y, v.x) - ahrs.yaw;
    }
    draw_speed(x + 1, y, angle, length);
}

//Thanks to betaflight/inav for simple and clean artificial horizon visual design
void AP_OSD_Screen::draw_horizon(uint8_t x, uint8_t y)
{
    AP_AHRS &ahrs = AP::ahrs();
    WITH_SEMAPHORE(ahrs.get_semaphore());
    float roll;
    float pitch;
    AP::vehicle()->get_osd_roll_pitch_rad(roll,pitch);
    pitch *= -1;

    //inverted roll AH
    if (check_option(AP_OSD::OPTION_INVERTED_AH_ROLL)) {
        roll = -roll;
    }

    pitch = constrain_float(pitch, -ah_max_pitch, ah_max_pitch);
    float ky = sinf(roll);
    float kx = cosf(roll);

    float ratio = backend->get_aspect_ratio_correction();

    if (fabsf(ky) < fabsf(kx)) {
        for (int dx = -4; dx <= 4; dx++) {
            float fy = (ratio * dx) * (ky/kx) + pitch * ah_pitch_rad_to_char + 0.5f;
            int dy = floorf(fy);
            char c = (fy - dy) * SYMBOL(SYM_AH_H_COUNT);
            //chars in font in reversed order
            c = SYMBOL(SYM_AH_H_START) + ((SYMBOL(SYM_AH_H_COUNT) - 1) - c);
            if (dy >= -4 && dy <= 4) {
                backend->write(x + dx, y - dy, false, "%c", c);
            }
        }
    } else {
        for (int dy=-4; dy<=4; dy++) {
            float fx = ((dy / ratio) - pitch * ah_pitch_rad_to_char) * (kx/ky) + 0.5f;
            int dx = floorf(fx);
            char c = (fx - dx) * SYMBOL(SYM_AH_V_COUNT);
            c = SYMBOL(SYM_AH_V_START) + c;
            if (dx >= -4 && dx <=4) {
                backend->write(x + dx, y - dy, false, "%c", c);
            }
        }
    }

    if (!check_option(AP_OSD::OPTION_DISABLE_CROSSHAIR)) {
        backend->write(x-1,y, false, "%c%c%c", SYMBOL(SYM_AH_CENTER_LINE_LEFT), SYMBOL(SYM_AH_CENTER), SYMBOL(SYM_AH_CENTER_LINE_RIGHT));
    }

}

void AP_OSD_Screen::draw_distance(uint8_t x, uint8_t y, float distance)
{
    char unit_icon = u_icon(DISTANCE);
    float distance_scaled = u_scale(DISTANCE, distance);
    const char *fmt = "%4.0f%c";
    if (distance_scaled > 9999.0f || (osd->units == AP_OSD::UNITS_IMPERIAL && distance_scaled > 5280.0f && (osd->options & AP_OSD::OPTION_IMPERIAL_MILES))) {
        distance_scaled = u_scale(DISTANCE_LONG, distance);
        unit_icon= u_icon(DISTANCE_LONG);
        //try to pack as many useful info as possible
        if (distance_scaled<9.0f) {
            fmt = "%1.3f%c";
        } else if (distance_scaled < 99.0f) {
            fmt = "%2.2f%c";
        } else if (distance_scaled < 999.0f) {
            fmt = "%3.1f%c";
        } else {
            fmt = "%4.0f%c";
        }
    } else if (distance_scaled < 10.0f) {
        fmt = "% 3.1f%c";
    }
    backend->write(x, y, false, fmt, (double)distance_scaled, unit_icon);
}

void AP_OSD_Screen::draw_home(uint8_t x, uint8_t y)
{
    AP_AHRS &ahrs = AP::ahrs();
    WITH_SEMAPHORE(ahrs.get_semaphore());
    Location loc;
    if (ahrs.get_location(loc) && ahrs.home_is_set()) {
        const Location &home_loc = ahrs.get_home();
        float distance = home_loc.get_distance(loc);
        int32_t angle_cd = loc.get_bearing_to(home_loc) - ahrs.yaw_sensor;
        if (distance < 2.0f) {
            //avoid fast rotating arrow at small distances
            angle_cd = 0;
        }
        char arrow = get_arrow_font_index(angle_cd);
        backend->write(x, y, false, "%c%c", SYMBOL(SYM_HOME), arrow);
        draw_distance(x+2, y, distance);
    } else {
        backend->write(x, y, true, "%c", SYMBOL(SYM_HOME));
    }
}

void AP_OSD_Screen::draw_heading(uint8_t x, uint8_t y)
{
    AP_AHRS &ahrs = AP::ahrs();
    uint16_t yaw = ahrs.yaw_sensor / 100;
    backend->write(x, y, false, "%3d%c", yaw, SYMBOL(SYM_DEGR));
}

#if AP_RPM_ENABLED
void AP_OSD_Screen::draw_rrpm(uint8_t x, uint8_t y)
{
    float _rrpm;
    const AP_RPM *rpm = AP_RPM::get_singleton();
    if (rpm != nullptr) {
            if (!rpm->get_rpm(0, _rrpm)) {
                // No valid RPM data
                _rrpm = -1;
            }
        } else {
            // No RPM because pointer is null
            _rrpm = -1;
        }
    int r_rpm = static_cast<int>(_rrpm);
    backend->write(x, y, false, "%4d%c", (int)r_rpm, SYMBOL(SYM_RPM));
}
#endif

void AP_OSD_Screen::draw_throttle(uint8_t x, uint8_t y)
{
    backend->write(x, y, false, "%3d%c", gcs().get_hud_throttle(), SYMBOL(SYM_PCNT));
}

#if HAL_OSD_SIDEBAR_ENABLE

void AP_OSD_Screen::draw_sidebars(uint8_t x, uint8_t y)
{
    const int8_t total_sectors = 18;
    static const uint8_t sidebar_sectors[total_sectors] = {
        SYM_SIDEBAR_A,
        SYM_SIDEBAR_B,
        SYM_SIDEBAR_C,
        SYM_SIDEBAR_D,
        SYM_SIDEBAR_E,
        SYM_SIDEBAR_F,
        SYM_SIDEBAR_G,
        SYM_SIDEBAR_E,
        SYM_SIDEBAR_F,
        SYM_SIDEBAR_G,
        SYM_SIDEBAR_E,
        SYM_SIDEBAR_F,
        SYM_SIDEBAR_G,
        SYM_SIDEBAR_E,
        SYM_SIDEBAR_F,
        SYM_SIDEBAR_H,
        SYM_SIDEBAR_I,
        SYM_SIDEBAR_J,
    };

    // Get altitude and airspeed, scaled to appropriate units
    float aspd = 0.0f;
    float alt = 0.0f;
    AP_AHRS &ahrs = AP::ahrs();
    WITH_SEMAPHORE(ahrs.get_semaphore());
    bool have_speed_estimate = ahrs.airspeed_estimate(aspd);
    if (!have_speed_estimate) { aspd = 0.0f; }
    ahrs.get_relative_position_D_home(alt);
    float scaled_aspd = u_scale(SPEED, aspd);
    float scaled_alt = u_scale(ALTITUDE, -alt);
    static const int aspd_interval = 10; //units between large tick marks
    int alt_interval = (osd->units == AP_OSD::UNITS_AVIATION || osd->units == AP_OSD::UNITS_IMPERIAL) ? 20 : 10;

    // render airspeed ladder
    int aspd_symbol_index = fmodf(scaled_aspd, aspd_interval) / aspd_interval * total_sectors;
    for (int i = 0; i < 7; i++){
        if (i == 3) {
            // the middle section of the ladder with the currrent airspeed
            backend->write(x, y+i, false, "%3d%c%c", (int) scaled_aspd, u_icon(SPEED), SYMBOL(SYM_SIDEBAR_R_ARROW));
        } else {
            backend->write(x+4, y+i, false,  "%c", SYMBOL(sidebar_sectors[aspd_symbol_index]));
        }
        aspd_symbol_index = (aspd_symbol_index + 12) % 18;
    }

    // render the altitude ladder
    // similar formula to above, but accounts for negative altitudes
    int alt_symbol_index = fmodf(fmodf(scaled_alt, alt_interval) + alt_interval, alt_interval) / alt_interval * total_sectors;
    for (int i = 0; i < 7; i++){
        if (i == 3) {
            // the middle section of the ladder with the currrent altitude
            backend->write(x+16, y+i, false, "%c%d%c", SYMBOL(SYM_SIDEBAR_L_ARROW), (int) scaled_alt, u_icon(ALTITUDE));
        } else {
            backend->write(x+16, y+i, false,  "%c", SYMBOL(sidebar_sectors[alt_symbol_index]));
        }
        alt_symbol_index = (alt_symbol_index + 12) % 18;
    }
}

#endif // HAL_OSD_SIDEBAR_ENABLE

//Thanks to betaflight/inav for simple and clean compass visual design
void AP_OSD_Screen::draw_compass(uint8_t x, uint8_t y)
{
    const int8_t total_sectors = 16;
    static const uint8_t compass_circle[total_sectors] = {
        SYM_HEADING_N,
        SYM_HEADING_LINE,
        SYM_HEADING_DIVIDED_LINE,
        SYM_HEADING_LINE,
        SYM_HEADING_E,
        SYM_HEADING_LINE,
        SYM_HEADING_DIVIDED_LINE,
        SYM_HEADING_LINE,
        SYM_HEADING_S,
        SYM_HEADING_LINE,
        SYM_HEADING_DIVIDED_LINE,
        SYM_HEADING_LINE,
        SYM_HEADING_W,
        SYM_HEADING_LINE,
        SYM_HEADING_DIVIDED_LINE,
        SYM_HEADING_LINE,
    };
    AP_AHRS &ahrs = AP::ahrs();
    int32_t yaw = ahrs.yaw_sensor;
    int32_t interval = 36000 / total_sectors;
    int8_t center_sector = ((yaw + interval / 2) / interval) % total_sectors;
    for (int8_t i = -4; i <= 4; i++) {
        int8_t sector = center_sector + i;
        sector = (sector + total_sectors) % total_sectors;
        backend->write(x + i, y, false,  "%c", SYMBOL(compass_circle[sector]));
    }
}

void AP_OSD_Screen::draw_wind(uint8_t x, uint8_t y)
{
#if !APM_BUILD_TYPE(APM_BUILD_Rover)
    AP_AHRS &ahrs = AP::ahrs();
    WITH_SEMAPHORE(ahrs.get_semaphore());
    Vector3f v = ahrs.wind_estimate();
    float angle = 0;
    const float length = v.length();
    if (length > 1.0f) {
        if (check_option(AP_OSD::OPTION_INVERTED_WIND)) {
            angle = M_PI;
        }
        angle = angle + atan2f(v.y, v.x) - ahrs.yaw;
    } 
    draw_speed(x + 1, y, angle, length);

#else
    const AP_WindVane* windvane = AP_WindVane::get_singleton();
    if (windvane != nullptr) {
        draw_speed(x + 1, y, windvane->get_apparent_wind_direction_rad() + M_PI, windvane->get_apparent_wind_speed());
    }
#endif

    backend->write(x, y, false, "%c", SYMBOL(SYM_WSPD));
}

void AP_OSD_Screen::draw_aspeed(uint8_t x, uint8_t y)
{
    float aspd = 0.0f;
    AP_AHRS &ahrs = AP::ahrs();
    WITH_SEMAPHORE(ahrs.get_semaphore());
    bool have_estimate = ahrs.airspeed_estimate(aspd);
    if (have_estimate) {
        backend->write(x, y, false, "%c%4d%c", SYMBOL(SYM_ASPD), (int)u_scale(SPEED, aspd), u_icon(SPEED));
    } else {
        backend->write(x, y, false, "%c ---%c", SYMBOL(SYM_ASPD), u_icon(SPEED));
    }
}

void AP_OSD_Screen::draw_vspeed(uint8_t x, uint8_t y)
{
    Vector3f v;
    float vspd;
    float vs_scaled;
    AP_AHRS &ahrs = AP::ahrs();
    WITH_SEMAPHORE(ahrs.get_semaphore());
    if (ahrs.get_velocity_NED(v)) {
        vspd = -v.z;
    } else {
        auto &baro = AP::baro();
        WITH_SEMAPHORE(baro.get_semaphore());
        vspd = baro.get_climb_rate();
    }
    char sym;
    if (vspd > 3.0f) {
        sym = SYMBOL(SYM_UP_UP);
    } else if (vspd >=0.0f) {
        sym = SYMBOL(SYM_UP);
    } else if (vspd >= -3.0f) {
        sym = SYMBOL(SYM_DOWN);
    } else {
        sym = SYMBOL(SYM_DOWN_DOWN);
    }
    vs_scaled = u_scale(VSPEED, fabsf(vspd));
    if ((osd->units != AP_OSD::UNITS_AVIATION) && (vs_scaled < 9.95f)) {
        backend->write(x, y, false, "%c%.1f%c", sym, (float)vs_scaled, u_icon(VSPEED));
    } else {
        const char *fmt = osd->units == AP_OSD::UNITS_AVIATION ? "%c%4d%c" : "%c%2d%c";
        backend->write(x, y, false, fmt, sym, (int)roundf(vs_scaled), u_icon(VSPEED));
    }
}

#if HAL_WITH_ESC_TELEM
void AP_OSD_Screen::draw_esc_temp(uint8_t x, uint8_t y)
{
    int16_t etemp;
    // first parameter is index into array of ESC's.  Hardwire to zero (first) for now.
    if (!AP::esc_telem().get_temperature(0, etemp)) {
        return;
    }

    backend->write(x, y, false, "%3d%c", (int)u_scale(TEMPERATURE, etemp / 100), u_icon(TEMPERATURE));
}

void AP_OSD_Screen::draw_esc_rpm(uint8_t x, uint8_t y)
{
    float rpm;
    // first parameter is index into array of ESC's.  Hardwire to zero (first) for now.
    if (!AP::esc_telem().get_rpm(0, rpm)) {
        return;
    }
    float krpm = rpm * 0.001f;
    const char *format = krpm < 9.995 ? "%.2f%c%c" : (krpm < 99.95 ? "%.1f%c%c" : "%.0f%c%c");
    backend->write(x, y, false, format, krpm, SYMBOL(SYM_KILO), SYMBOL(SYM_RPM));
}

void AP_OSD_Screen::draw_esc_amps(uint8_t x, uint8_t y)
{
    float amps;
    // first parameter is index into array of ESC's.  Hardwire to zero (first) for now.
    if (!AP::esc_telem().get_current(0, amps)) {
        return;
    }
    backend->write(x, y, false, "%4.1f%c", amps, SYMBOL(SYM_AMP));
}
#endif

void AP_OSD_Screen::draw_gps_latitude(uint8_t x, uint8_t y)
{
    AP_GPS & gps = AP::gps();
    const Location &loc = gps.location();   // loc.lat and loc.lng
    int32_t dec_portion, frac_portion;
    int32_t abs_lat = labs(loc.lat);

    dec_portion = loc.lat / 10000000L;
    frac_portion = abs_lat - labs(dec_portion)*10000000UL;

    backend->write(x, y, false, "%c%4ld.%07ld", SYMBOL(SYM_GPS_LAT), (long)dec_portion,(long)frac_portion);
}

void AP_OSD_Screen::draw_gps_longitude(uint8_t x, uint8_t y)
{
    AP_GPS & gps = AP::gps();
    const Location &loc = gps.location();   // loc.lat and loc.lng
    int32_t dec_portion, frac_portion;
    int32_t abs_lon = labs(loc.lng);

    dec_portion = loc.lng / 10000000L;
    frac_portion = abs_lon - labs(dec_portion)*10000000UL;

    backend->write(x, y, false, "%c%4ld.%07ld", SYMBOL(SYM_GPS_LONG), (long)dec_portion,(long)frac_portion);
}

void AP_OSD_Screen::draw_roll_angle(uint8_t x, uint8_t y)
{
    AP_AHRS &ahrs = AP::ahrs();
    uint16_t roll = abs(ahrs.roll_sensor) / 100;
    char r;
    if (ahrs.roll_sensor > 50) {
        r = SYMBOL(SYM_ROLLR);
    } else if (ahrs.roll_sensor < -50) {
        r = SYMBOL(SYM_ROLLL);
    } else {
        r = SYMBOL(SYM_ROLL0);
    }
    backend->write(x, y, false, "%c%3d%c", r, roll, SYMBOL(SYM_DEGR));
}

void AP_OSD_Screen::draw_pitch_angle(uint8_t x, uint8_t y)
{
    AP_AHRS &ahrs = AP::ahrs();
    uint16_t pitch = abs(ahrs.pitch_sensor) / 100;
    char p;
    if (ahrs.pitch_sensor > 50) {
        p = SYMBOL(SYM_PTCHUP);
    } else if (ahrs.pitch_sensor < -50) {
        p = SYMBOL(SYM_PTCHDWN);
    } else {
        p = SYMBOL(SYM_PTCH0);
    }
    backend->write(x, y, false, "%c%3d%c", p, pitch, SYMBOL(SYM_DEGR));
}

void AP_OSD_Screen::draw_temp(uint8_t x, uint8_t y)
{
    AP_Baro &barometer = AP::baro();
    float tmp = barometer.get_temperature();
    backend->write(x, y, false, "%3d%c", (int)u_scale(TEMPERATURE, tmp), u_icon(TEMPERATURE));
}


void AP_OSD_Screen::draw_hdop(uint8_t x, uint8_t y)
{
    AP_GPS & gps = AP::gps();
    float hdp = gps.get_hdop() * 0.01f;
    backend->write(x, y, false, "%c%c%3.2f", SYMBOL(SYM_HDOP_L), SYMBOL(SYM_HDOP_R), (double)hdp);
}

void AP_OSD_Screen::draw_waypoint(uint8_t x, uint8_t y)
{
    AP_AHRS &ahrs = AP::ahrs();
    int32_t angle_cd = osd->nav_info.wp_bearing - ahrs.yaw_sensor;
    if (osd->nav_info.wp_distance < 2.0f) {
        //avoid fast rotating arrow at small distances
        angle_cd = 0;
    }
    char arrow = get_arrow_font_index(angle_cd);
    backend->write(x,y, false, "%c%2u%c",SYMBOL(SYM_WPNO), osd->nav_info.wp_number, arrow);
    draw_distance(x+4, y, osd->nav_info.wp_distance);
}

void AP_OSD_Screen::draw_xtrack_error(uint8_t x, uint8_t y)
{
    backend->write(x, y, false, "%c", SYMBOL(SYM_XERR));
    draw_distance(x+1, y, osd->nav_info.wp_xtrack_error);
}

void AP_OSD_Screen::draw_stat(uint8_t x, uint8_t y)
{
    backend->write(x+2, y, false, "%c%c%c", 0x4d,0x41,0x58);
    backend->write(x, y+1, false, "%c",SYMBOL(SYM_GSPD));
    backend->write(x+1, y+1, false, "%4d%c", (int)u_scale(SPEED, osd->_stats.max_speed_mps), u_icon(SPEED));
    backend->write(x, y+2, false, "%5.1f%c", (double)osd->_stats.max_current_a, SYMBOL(SYM_AMP));
    backend->write(x, y+3, false, "%5d%c", (int)u_scale(ALTITUDE, osd->_stats.max_alt_m), u_icon(ALTITUDE));
    backend->write(x, y+4, false, "%c", SYMBOL(SYM_HOME));
    draw_distance(x+1, y+4, osd->_stats.max_dist_m);
    backend->write(x, y+5, false, "%c", SYMBOL(SYM_DIST));
    draw_distance(x+1, y+5, osd->_stats.last_distance_m);
}

void AP_OSD_Screen::draw_dist(uint8_t x, uint8_t y)
{
    backend->write(x, y, false, "%c", SYMBOL(SYM_DIST));
    draw_distance(x+1, y, osd->_stats.last_distance_m);
}

void  AP_OSD_Screen::draw_flightime(uint8_t x, uint8_t y)
{
    AP_Stats *stats = AP::stats();
    if (stats) {
        uint32_t t = stats->get_flight_time_s();
        backend->write(x, y, false, "%c%3u:%02u", SYMBOL(SYM_FLY), unsigned(t/60), unsigned(t%60));
    }
}

void AP_OSD_Screen::draw_eff(uint8_t x, uint8_t y)
{
    AP_BattMonitor &battery = AP::battery();
    Vector2f v;
    {
        AP_AHRS &ahrs = AP::ahrs();
        WITH_SEMAPHORE(ahrs.get_semaphore());
        v = ahrs.groundspeed_vector();
    }
    float speed = u_scale(SPEED,v.length());
    float current_amps;
    if ((speed > 2.0) && battery.current_amps(current_amps)) {
        backend->write(x, y, false, "%c%3d%c", SYMBOL(SYM_EFF),int(1000.0f*current_amps/speed),SYMBOL(SYM_MAH));
    } else {
        backend->write(x, y, false, "%c---%c", SYMBOL(SYM_EFF),SYMBOL(SYM_MAH));
    }
}

void AP_OSD_Screen::draw_climbeff(uint8_t x, uint8_t y)
{
    char unit_icon = u_icon(DISTANCE);
    Vector3f v;
    float vspd;
    do {
        {
            auto &ahrs = AP::ahrs();
            WITH_SEMAPHORE(ahrs.get_semaphore());
            if (ahrs.get_velocity_NED(v)) {
                vspd = -v.z;
                break;
            }
        }
        auto &baro = AP::baro();
        WITH_SEMAPHORE(baro.get_semaphore());
        vspd = baro.get_climb_rate();
    } while (false);
    if (vspd < 0.0) {
        vspd = 0.0;
    }
    AP_BattMonitor &battery = AP::battery();
    float amps;
    if (battery.current_amps(amps) && is_positive(amps)) {
        backend->write(x, y, false,"%c%c%3.1f%c",SYMBOL(SYM_PTCHUP),SYMBOL(SYM_EFF),(double)(3.6f * u_scale(VSPEED,vspd)/amps),unit_icon);
    } else {
        backend->write(x, y, false,"%c%c---%c",SYMBOL(SYM_PTCHUP),SYMBOL(SYM_EFF),unit_icon);
    }
}

#if BARO_MAX_INSTANCES > 1
void AP_OSD_Screen::draw_btemp(uint8_t x, uint8_t y)
{
    AP_Baro &barometer = AP::baro();
    float btmp = barometer.get_temperature(1);
    backend->write(x, y, false, "%3d%c", (int)u_scale(TEMPERATURE, btmp), u_icon(TEMPERATURE));
}
#endif

void AP_OSD_Screen::draw_atemp(uint8_t x, uint8_t y)
{
#if AP_AIRSPEED_ENABLED
    AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
    if (!airspeed) {
        return;
    }
    float temperature = 0;
    airspeed->get_temperature(temperature);
    if (airspeed->healthy()) {
        backend->write(x, y, false, "%3d%c", (int)u_scale(TEMPERATURE, temperature), u_icon(TEMPERATURE));
    } else {
        backend->write(x, y, false, "--%c", u_icon(TEMPERATURE));
    }
#endif
}

void AP_OSD_Screen::draw_bat2_vlt(uint8_t x, uint8_t y)
{
    draw_bat_volt(1,VoltageType::VOLTAGE,x,y);
}

void AP_OSD_Screen::draw_bat2used(uint8_t x, uint8_t y)
{
    draw_batused(1, x, y);
}

void AP_OSD_Screen::draw_aspd1(uint8_t x, uint8_t y)
{
#if AP_AIRSPEED_ENABLED
    AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
    if (!airspeed) {
        return;
    }
    float asp1 = airspeed->get_airspeed();
    if (airspeed != nullptr && airspeed->healthy()) {
        backend->write(x, y, false, "%c%4d%c", SYMBOL(SYM_ASPD), (int)u_scale(SPEED, asp1), u_icon(SPEED));
    } else {
        backend->write(x, y, false, "%c ---%c", SYMBOL(SYM_ASPD), u_icon(SPEED));
    }
#endif
}

void AP_OSD_Screen::draw_aspd2(uint8_t x, uint8_t y)
{
#if AP_AIRSPEED_ENABLED
    AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
    if (!airspeed) {
        return;
    }
    float asp2 = airspeed->get_airspeed(1);
    if (airspeed != nullptr && airspeed->healthy(1)) {
        backend->write(x, y, false, "%c%4d%c", SYMBOL(SYM_ASPD), (int)u_scale(SPEED, asp2), u_icon(SPEED));
    } else {
        backend->write(x, y, false, "%c ---%c", SYMBOL(SYM_ASPD), u_icon(SPEED));
    }
#endif
}

void AP_OSD_Screen::draw_clk(uint8_t x, uint8_t y)
{
    AP_RTC &rtc = AP::rtc();
    uint8_t hour, min, sec;
    uint16_t ms;
    if (!rtc.get_local_time(hour, min, sec, ms)) {
    backend->write(x, y, false, "%c--:--", SYMBOL(SYM_CLK));
    } else {
    backend->write(x, y, false, "%c%02u:%02u", SYMBOL(SYM_CLK), hour, min);
    }
}

#if HAL_PLUSCODE_ENABLE
void AP_OSD_Screen::draw_pluscode(uint8_t x, uint8_t y)
{
    AP_GPS & gps = AP::gps();
    const Location &loc = gps.location();
    char buff[16];
    if (gps.status() == AP_GPS::NO_GPS || gps.status() == AP_GPS::NO_FIX){
        backend->write(x, y, false, "--------+--");
    } else {
        AP_OLC::olc_encode(loc.lat, loc.lng, 10, buff, sizeof(buff));
        backend->write(x, y, false, "%s", buff);
    }
}
#endif

/*
  support callsign display from a file called callsign.txt
 */
void AP_OSD_Screen::draw_callsign(uint8_t x, uint8_t y)
{
#if AP_OSD_CALLSIGN_FROM_SD_ENABLED
    if (!callsign_data.load_attempted) {
        callsign_data.load_attempted = true;
        FileData *fd = AP::FS().load_file("callsign.txt");
        if (fd != nullptr) {
            uint32_t len = fd->length;
            // trim off whitespace
            while (len > 0 && isspace(fd->data[len-1])) {
                len--;
            }
            callsign_data.str = strndup((const char *)fd->data, len);
            delete fd;
        }
    }
    if (callsign_data.str != nullptr) {
        backend->write(x, y, false, "%s", callsign_data.str);
    }
#endif
}

void AP_OSD_Screen::draw_current2(uint8_t x, uint8_t y)
{
    draw_current(1, x, y);
}

#if AP_VIDEOTX_ENABLED
void AP_OSD_Screen::draw_vtx_power(uint8_t x, uint8_t y)
{
    AP_VideoTX *vtx = AP_VideoTX::get_singleton();
    if (!vtx) {
        return;
    }
    uint16_t powr = 0;
    // If currently in pit mode, just render 0mW to the screen
    if(!vtx->has_option(AP_VideoTX::VideoOptions::VTX_PITMODE)){
        powr = vtx->get_power_mw();
    }
    backend->write(x, y, !vtx->is_configuration_finished(), "%4hu%c", powr, SYMBOL(SYM_MW));
}
#endif  // AP_VIDEOTX_ENABLED

#if AP_TERRAIN_AVAILABLE
void AP_OSD_Screen::draw_hgt_abvterr(uint8_t x, uint8_t y)
{
    AP_Terrain *terrain = AP::terrain();

    float terrain_altitude;
    if (terrain != nullptr && terrain->height_above_terrain(terrain_altitude,true)) {
        bool blink = (osd->warn_terr != -1)? (terrain_altitude < osd->warn_terr) : false; //blink if warn_terr is not disabled and alt above terrain is below warning value
        backend->write(x, y, blink, "%4d%c%c", (int)u_scale(ALTITUDE, terrain_altitude), u_icon(ALTITUDE), SYMBOL(SYM_TERALT));
     } else {
        backend->write(x, y, false, " ---%c%c", u_icon(ALTITUDE),SYMBOL(SYM_TERALT));
     }
}
#endif

#if AP_FENCE_ENABLED
void AP_OSD_Screen::draw_fence(uint8_t x, uint8_t y)
{
    AC_Fence *fenceptr = AP::fence();
    if (fenceptr == nullptr) {
       return;
    }
    if (fenceptr->enabled() && fenceptr->present()) {
        backend->write(x, y, fenceptr->get_breaches(), "%c", SYMBOL(SYM_FENCE_ENABLED));
    } else {
        backend->write(x, y, false, "%c", SYMBOL(SYM_FENCE_DISABLED));
    }
}
#endif

void AP_OSD_Screen::draw_rngf(uint8_t x, uint8_t y)
{
    RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (rangefinder == nullptr) {
       return;
    }
    if (rangefinder->status_orient(ROTATION_PITCH_270) < RangeFinder::Status::Good) {
        backend->write(x, y, false, "%c---%c", SYMBOL(SYM_RNGFD), u_icon(DISTANCE));
    } else {
        const float distance = rangefinder->distance_orient(ROTATION_PITCH_270);
        backend->write(x, y, false, "%c%4.1f%c", SYMBOL(SYM_RNGFD), u_scale(DISTANCE, distance), u_icon(DISTANCE));
    }
}

#define DRAW_SETTING(n) if (n.enabled) draw_ ## n(n.xpos, n.ypos)

#if HAL_WITH_OSD_BITMAP || HAL_WITH_MSP_DISPLAYPORT
void AP_OSD_Screen::draw(void)
{
    if (!enabled || !backend) {
        return;
    }
    //Note: draw order should be optimized.
    //Big and less important items should be drawn first,
    //so they will not overwrite more important ones.
#if HAL_OSD_SIDEBAR_ENABLE
    DRAW_SETTING(sidebars);
#endif

    DRAW_SETTING(message);
    DRAW_SETTING(horizon);
    DRAW_SETTING(compass);
    DRAW_SETTING(altitude);

#if AP_TERRAIN_AVAILABLE
    DRAW_SETTING(hgt_abvterr);
#endif

    DRAW_SETTING(rngf);
    DRAW_SETTING(waypoint);
    DRAW_SETTING(xtrack_error);
    DRAW_SETTING(bat_volt);
    DRAW_SETTING(bat2_vlt);
    DRAW_SETTING(avgcellvolt);
    DRAW_SETTING(avgcellrestvolt);
    DRAW_SETTING(restvolt);
    DRAW_SETTING(rssi);
    DRAW_SETTING(link_quality);
    DRAW_SETTING(current);
    DRAW_SETTING(batused);
    DRAW_SETTING(bat2used);
    DRAW_SETTING(sats);
    DRAW_SETTING(fltmode);
    DRAW_SETTING(gspeed);
    DRAW_SETTING(aspeed);
    DRAW_SETTING(aspd1);
    DRAW_SETTING(aspd2);
    DRAW_SETTING(vspeed);
    DRAW_SETTING(throttle);
    DRAW_SETTING(heading);
    DRAW_SETTING(wind);
    DRAW_SETTING(home);
#if AP_RPM_ENABLED
    DRAW_SETTING(rrpm);
#endif
#if AP_FENCE_ENABLED
    DRAW_SETTING(fence);
#endif
    DRAW_SETTING(roll_angle);
    DRAW_SETTING(pitch_angle);
    DRAW_SETTING(temp);
#if BARO_MAX_INSTANCES > 1
    DRAW_SETTING(btemp);
#endif
    DRAW_SETTING(atemp);
    DRAW_SETTING(hdop);
    DRAW_SETTING(flightime);
    DRAW_SETTING(clk);
#if AP_VIDEOTX_ENABLED
    DRAW_SETTING(vtx_power);
#endif

#if HAL_WITH_ESC_TELEM
    DRAW_SETTING(esc_temp);
    DRAW_SETTING(esc_rpm);
    DRAW_SETTING(esc_amps);
#endif

    DRAW_SETTING(gps_latitude);
    DRAW_SETTING(gps_longitude);
#if HAL_PLUSCODE_ENABLE
    DRAW_SETTING(pluscode);
#endif
    DRAW_SETTING(dist);
    DRAW_SETTING(stat);
    DRAW_SETTING(climbeff);
    DRAW_SETTING(eff);
    DRAW_SETTING(callsign);
    DRAW_SETTING(current2);
}
#endif
#endif // OSD_ENABLED
