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

#include <AP_RPM/AP_RPM_Params.h>

// table of user settable parameters
const AP_Param::GroupInfo AP_RPM_Params::var_info[] = {
    // @Param: TYPE
    // @DisplayName: RPM type
    // @Description: What type of RPM sensor is connected
    // @Values: 0:None,1:Not Used,2:GPIO,3:EFI,4:Harmonic Notch,5:ESC Telemetry Motors Bitmask,6:Generator
    // @User: Standard
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_RPM_Params, type, 0, AP_PARAM_FLAG_ENABLE),
    // Note, 1 was previously for type = PWM. This has been removed from docs to make setup less confusing for users.
    // However, 1 is reserved as it does still fallthrough to type = GPIO.

    // @Param: SCALING
    // @DisplayName: RPM scaling
    // @Description: Scaling factor between sensor reading and RPM.
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("SCALING", 2, AP_RPM_Params, scaling, 1.0f),

    // @Param: MAX
    // @DisplayName: Maximum RPM
    // @Description: Maximum RPM to report. Only used on type = GPIO.
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MAX", 3, AP_RPM_Params, maximum, 100000),

    // @Param: MIN
    // @DisplayName: Minimum RPM
    // @Description: Minimum RPM to report. Only used on type = GPIO.
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MIN", 4, AP_RPM_Params, minimum, 10),

    // @Param: MIN_QUAL
    // @DisplayName: Minimum Quality
    // @Description: Minimum data quality to be used
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("MIN_QUAL", 5, AP_RPM_Params, quality_min, 0.5),

    // @Param: PIN
    // @DisplayName: Input pin number
    // @Description: Which digital GPIO pin to use. Only used on type = GPIO. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    AP_GROUPINFO("PIN", 6, AP_RPM_Params, pin, -1),

    // @Param: ESC_MASK
    // @DisplayName: Bitmask of ESC telemetry channels to average
    // @Description: Mask of channels which support ESC rpm telemetry. RPM telemetry of the selected channels will be averaged
    // @Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16
    // @User: Advanced
    AP_GROUPINFO("ESC_MASK", 7, AP_RPM_Params, esc_mask, 0),

#if AP_RPM_ESC_TELEM_OUTBOUND_ENABLED
    // @Param: ESC_INDEX
    // @DisplayName: ESC Telemetry Index to write RPM to
    // @Description: ESC Telemetry Index to write RPM to. Use 0 to disable.
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ESC_INDEX", 8, AP_RPM_Params, esc_telem_outbound_index, 0),
#endif

    AP_GROUPEND
};

AP_RPM_Params::AP_RPM_Params(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}
