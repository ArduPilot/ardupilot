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
  parameter object for one setting in AP_OSD
 */

#include "AP_OSD.h"
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <ctype.h>

#if OSD_PARAM_ENABLED

const AP_Param::GroupInfo AP_OSD_ParamSetting::var_info[] = {
    // @Param: _EN
    // @DisplayName: Enable
    // @Description: Enable setting
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("_EN", 1, AP_OSD_ParamSetting, enabled, 0),

    // @Param: _X
    // @DisplayName: X position
    // @Description: Horizontal position on screen
    // @Range: 0 29
    // @User: Standard
    AP_GROUPINFO("_X", 2, AP_OSD_ParamSetting, xpos, 0),

    // @Param: _Y
    // @DisplayName: Y position
    // @Description: Vertical position on screen
    // @Range: 0 15
    // @User: Standard
    AP_GROUPINFO("_Y", 3, AP_OSD_ParamSetting, ypos, 0),

    // Parameter access keys. These default to -1 too allow user overrides
    // to work properly

    // @Param: _KEY
    // @DisplayName: Parameter key
    // @Description: Key of the parameter to be displayed and modified
    // @User: Standard
    AP_GROUPINFO("_KEY", 4, AP_OSD_ParamSetting, _param_key, -1),

    // @Param: _IDX
    // @DisplayName: Parameter index
    // @Description: Index of the parameter to be displayed and modified
    // @User: Standard
    AP_GROUPINFO("_IDX", 5, AP_OSD_ParamSetting, _param_idx, -1),

    // @Param: _GRP
    // @DisplayName: Parameter group
    // @Description: Group of the parameter to be displayed and modified
    // @User: Standard
    AP_GROUPINFO("_GRP", 6, AP_OSD_ParamSetting, _param_group, -1),

    // @Param: _MIN
    // @DisplayName: Parameter minimum
    // @Description: Minimum value of the parameter to be displayed and modified
    // @User: Standard
    AP_GROUPINFO("_MIN", 7, AP_OSD_ParamSetting, _param_min, 0.0f),

    // @Param: _MAX
    // @DisplayName: Parameter maximum
    // @Description: Maximum of the parameter to be displayed and modified
    // @User: Standard
    AP_GROUPINFO("_MAX", 8, AP_OSD_ParamSetting, _param_max, 1.0f),

    // @Param: _INCR
    // @DisplayName: Parameter increment
    // @Description: Increment of the parameter to be displayed and modified
    // @User: Standard
    AP_GROUPINFO("_INCR", 9, AP_OSD_ParamSetting, _param_incr, 0.001f),

    // @Param: _TYPE
    // @DisplayName: Parameter type
    // @Description: Type of the parameter to be displayed and modified
    // @User: Standard
    AP_GROUPINFO("_TYPE", 10, AP_OSD_ParamSetting, _type, 0),

    AP_GROUPEND
};

#define PARAM_COMPOSITE_INDEX(key, idx, group) (uint32_t((uint32_t(key) << 23) | (uint32_t(idx) << 18) | uint32_t(group)))

#define OSD_PARAM_DEBUG 0
#if OSD_PARAM_DEBUG
#define debug(fmt, args ...) do { hal.console->printf("OSD: " fmt, args); } while (0)
#else
#define debug(fmt, args ...)
#endif

// at the cost of a little flash, we can create much better ranges and values for certain important settings
// common labels - all strings must be upper case
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane) || APM_BUILD_COPTER_OR_HELI

static const char* SERIAL_PROTOCOL_VALUES[] = {
    "", "MAV", "MAV2", "FSKY_D", "FSKY_S", "GPS", "", "ALEX", "STORM", "RNG", 
    "FSKY_TX", "LID360", "", "BEACN", "VOLZ", "SBUS", "ESC_TLM", "DEV_TLM", "OPTFLW", "RBTSRV",
    "NMEA", "WNDVNE", "SLCAN", "RCIN", "MGSQRT", "LTM", "RUNCAM", "HOT_TLM", "SCRIPT", "CRSF",
    "GEN", "WNCH", "MSP", "DJI"
};

static const char* SERVO_FUNCTIONS[] = {
    "NONE", "RCPASS", "FLAP", "FLAP_AUTO", "AIL", "", "MNT_PAN", "MNT_TLT", "MNT_RLL", "MNT_OPEN", 
    "CAM_TRG", "", "MNT2_PAN", "MNT2_TLT", "MNT2_RLL", "MNT2_OPEN", "DIF_SPL_L1", "DIF_SPL_R1", "", "ELE",
    "", "RUD", "SPR_PMP", "SPR_SPIN", "FLPRON_L", "FLPRON_R", "GRND_STEER", "PARACHT", "GRIP", "GEAR",
    "ENG_RUN_EN", "HELI_RSC", "HELI_TAIL_RSC", "MOT_1", "MOT_2", "MOT_3", "MOT_4", "MOT_5", "MOT_6", "MOT_7",
    "MOT_8", "MOT_TLT", "", "", "", "", "", "", "", "",
    "", "RCIN_1", "RCIN_2", "RCIN_3", "RCIN_4", "RCIN_5", "RCIN_6", "RCIN_7", "RCIN_8", "RCIN_9",
    "RCIN_10", "RCIN_11", "RCIN_12", "RCIN_13", "RCIN_14", "RCIN_15", "RCIN_16", "IGN", "", "START",
    "THR", "TRCK_YAW", "TRCK_PIT", "THR_L", "THR_R", "TLTMOT_L", "TLTMOT_R", "ELEVN_L", "ELEVN_R", "VTAIL_L",
    "VTAIL_R", "BOOST_THR", "MOT_9", "MOT_10", "MOT_11", "MOT_12", "DIF_SPL_L2", "DIF_SPL_R2", "", "MAIN_SAIL",
    "CAM_ISO", "CAM_APTR", "CAM_FOC", "CAM_SH_SPD", "SCRPT_1", "SCRPT_2", "SCRPT_3", "SCRPT_4", "SCRPT_5", "SCRPT_6",
    "SCRPT_7", "SCRPT_8", "SCRPT_9", "SCRPT_10", "SCRPT_11", "SCRPT_12", "SCRPT_13", "SCRPT_14", "SCRPT_15", "SCRPT_16",
    "", "", "", "", "", "", "", "", "", "",
    "NEOPX_1", "NEOPX_2", "NEOPX_3", "NEOPX_4", "RAT_RLL", "RAT_PIT","RAT_THRST", "RAT_YAW", "WSAIL_EL", "PRLED_1",
    "PRLED_2", "PRLED_3", "PRLED_CLK", "WNCH_CL"
};

#endif

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)

static const char* AUX_OPTIONS[] = {
    "NONE", "", "", "", "RTL", "", "", "", "", "CAM_TRG",
    "", "", "", "", "", "", "AUTO", "", "", "",
    "", "", "", "", "MIS_RST", "", "", "", "RLY", "LAND_GR",
    "LOST_SND", "M_ESTOP", "", "", "", "RLY3", "RLY4", "", "OA_ADSB", "",
    "", "ARM/DS", "", "INVERT", "", "", "RC_OVRD", "", "", "",
    "", "MANUAL", "", "", "", "GUIDE", "LOIT", "", "CLR_WP", "",
    "", "", "COMP_LRN", "", "REV_THR", "GPS_DIS", "RLY5", "RLY6", "", "",
    "", "", "CIRCLE", "", "", "", "", "TAKEOFF", "RCAM_CTL", "RCAM_OSD",
    "", "DSARM", "QASS3POS", "", "AIR", "GEN", "TER_AUTO", "CROW_SEL", "SOAR", "", 
    "", "", "", "", "", "", "", "", "", "",
    "KILLIMU1", "KILLIMU2", "CAM_TOG", "", "", "GPSYAW_DIS"
};

static const char* FLTMODES[] = {
    "MAN", "CIRC", "STAB", "TRAIN", "ACRO", "FBWA", "FBWB", "CRUISE", "ATUNE", "", "AUTO",
    "RTL", "LOIT", "TKOF", "ADSB", "GUID", "", "QSTAB", "QHOV", "QLOIT", "QLAND",
    "QRTL", "QTUNE", "QACRO", "THRML", "L2QLND"
};

static const char* FS_ACT[] = {
    "NONE", "RTL", "LAND", "TERM", "QLAND", "PARA"
};

static const char* FS_SHRT_ACTNS[] = {
    "CRC_NOCHNGE", "CIRC", "FBWA", "DSABLE"
};

static const char* FS_LNG_ACTNS[] = {
    "CNTNUE", "RTL", "GLIDE", "PARACHT"
};

// plane parameters
const AP_OSD_ParamSetting::ParamMetadata AP_OSD_ParamSetting::_param_metadata[OSD_PARAM_NUM_TYPES] = {
    { -1, AP_SerialManager::SerialProtocol_NumProtocols - 1,    1, ARRAY_SIZE(SERIAL_PROTOCOL_VALUES), SERIAL_PROTOCOL_VALUES },  // OSD_PARAM_SERIAL_PROTOCOL
    { 0, SRV_Channel::k_nr_aux_servo_functions - 1,             1, ARRAY_SIZE(SERVO_FUNCTIONS), SERVO_FUNCTIONS },                // OSD_PARAM_SERVO_FUNCTION
    { 0, 105, 1, ARRAY_SIZE(AUX_OPTIONS), AUX_OPTIONS },                        // OSD_PARAM_AUX_FUNCTION
    { 0, 25, 1,  ARRAY_SIZE(FLTMODES), FLTMODES },                              // OSD_PARAM_FLIGHT_MODE
    { 0, 5, 1,   ARRAY_SIZE(FS_ACT), FS_ACT },                                  // OSD_PARAM_FAILSAFE_ACTION
    { 0, 3, 1,   ARRAY_SIZE(FS_SHRT_ACTNS), FS_SHRT_ACTNS },                    // OSD_PARAM_FAILSAFE_ACTION_1
    { 0, 3, 1,   ARRAY_SIZE(FS_LNG_ACTNS), FS_LNG_ACTNS },                      // OSD_PARAM_FAILSAFE_ACTION_2
};

#elif APM_BUILD_COPTER_OR_HELI

static const char* AUX_OPTIONS[] = {
    "NONE", "", "FLIP", "SIMP", "RTL", "SAV_TRM", "", "SAV_WP", "", "CAM_TRG",
    "RNG", "FENCE", "", "SSIMP", "ACRO_TRN", "SPRAY", "AUTO", "AUTOTN", "LAND", "GRIP",
    "", "CHUTE_EN", "CHUTE_RL", "CHUTE_3P", "MIS_RST", "ATT_FF", "ATT_ACC", "RET_MNT", "RLY", "LAND_GR",
    "LOST_SND", "M_ESTOP", "M_ILOCK", "BRAKE", "RLY2", "RLY3", "RLY4", "THROW", "OA_ADSB", "PR_LOIT",
    "OA_PROX", "ARM/DS", "SMRT_RTL", "INVERT", "", "", "RC_OVRD", "USR1", "USR2", "USR3",
    "", "", "ACRO", "", "", "GUIDE", "LOIT", "FOLLOW", "CLR_WP", "",
    "ZZAG", "ZZ_SVWP", "COMP_LRN", "", "", "GPS_DIS", "RLY5", "RLY6", "STAB", "PHOLD",
    "AHOLD", "FHOLD", "CIRCLE", "DRIFT", "", "", "STANDBY", "", "RCAM_CTL", "RCAM_OSD",
    "VISO_CAL", "DISARM", "", "ZZ_Auto", "AIR", "", "", "", "", "",
    "", "", "", "", "", "", "", "", "", "",
    "KILLIMU1", "KILLIMU2", "CAM_MOD_TOG", "", "", "GPSYAW_DIS"
};

static const char* FLTMODES[] = {
    "STAB", "ACRO", "ALTHOLD", "AUTO", "GUIDED", "LOIT", "RTL", "CIRC", "", "LAND",
    "", "DRFT", "", "SPORT", "FLIP", "ATUN", "POSHLD", "BRAKE", "THROW", "AVD_ADSB",
    "GUID_NOGPS", "SMRTRTL", "FLOHOLD", "FOLLOW", "ZIGZAG", "SYSID", "HELI_ARO", "AUTORTL",
    "TRTLE"
};

static const char* FS_OPTIONS[] = {
    "NONE", "CONT_RCFS", "CONT_GCSFS", "CONT_RC/GCSFS", "CONT_GUID_RC", "", "", "", "CONT_LAND", "",
    "", "", "", "", "", "CONT_CTRL_GCS", "", "", "CONTNUE"
};

static const char* THR_FS_ACT[] = {
    "NONE", "RTL", "CONT", "LAND", "SRTL_RTL", "SRTL_LAND"
};

static const char* FS_ACT[] = {
    "NONE", "LAND", "RTL", "SRTL_RTL", "SRTL_LAND", "TERM"
};

// copter parameters
const AP_OSD_ParamSetting::ParamMetadata AP_OSD_ParamSetting::_param_metadata[OSD_PARAM_NUM_TYPES] = {
    { -1, AP_SerialManager::SerialProtocol_NumProtocols - 1,    1, ARRAY_SIZE(SERIAL_PROTOCOL_VALUES), SERIAL_PROTOCOL_VALUES },  // OSD_PARAM_SERIAL_PROTOCOL
    { 0, SRV_Channel::k_nr_aux_servo_functions - 1,             1, ARRAY_SIZE(SERVO_FUNCTIONS), SERVO_FUNCTIONS },                // OSD_PARAM_SERVO_FUNCTION
    { 0, 105, 1, ARRAY_SIZE(AUX_OPTIONS), AUX_OPTIONS },                        // OSD_PARAM_AUX_FUNCTION
    { 0, 28, 1,  ARRAY_SIZE(FLTMODES), FLTMODES },                              // OSD_PARAM_FLIGHT_MODE
    { 0, 3, 1,   ARRAY_SIZE(FS_OPTIONS), FS_OPTIONS },                          // OSD_PARAM_FAILSAFE_ACTION
    { 0, 5, 1,   ARRAY_SIZE(FS_ACT), FS_ACT },                                  // OSD_PARAM_FAILSAFE_ACTION_1
    { 0, 5, 1,   ARRAY_SIZE(THR_FS_ACT), THR_FS_ACT },                          // OSD_PARAM_FAILSAFE_ACTION_2
};

#else
const AP_OSD_ParamSetting::ParamMetadata AP_OSD_ParamSetting::_param_metadata[OSD_PARAM_NUM_TYPES] = {};
#endif

extern const AP_HAL::HAL& hal;

// constructor
AP_OSD_ParamSetting::AP_OSD_ParamSetting(uint8_t param_number, bool _enabled, uint8_t x, uint8_t y,  int16_t key, int8_t idx, int32_t group, int8_t type, float min, float max, float incr)
    : AP_OSD_Setting(_enabled, x, y), _param_number(param_number)
{
    _param_group.set(group);
    _param_idx.set(idx);
    _param_key.set(key);
    _param_min.set(min);
    _param_max.set(max);
    _param_incr.set(incr);
    _type.set(type);
}

// default constructor that just sets some sensible defaults that exist on all platforms
AP_OSD_ParamSetting::AP_OSD_ParamSetting(uint8_t param_number)
    : AP_OSD_Setting(false, 2, param_number + 1), _param_number(param_number)
{
    _param_min.set(0.0f);
    _param_max.set(1.0f);
    _param_incr.set(0.001f);
    _type.set(OSD_PARAM_NONE);
}

// construct a setting from a compact static initializer structure
AP_OSD_ParamSetting::AP_OSD_ParamSetting(const Initializer& initializer)
    : AP_OSD_ParamSetting(initializer.index)
{
    _param_group.set(initializer.token.group_element);
    _param_idx.set(initializer.token.idx);
    _param_key.set(initializer.token.key);
    _type.set(initializer.type);
    enabled.set(true);
}

// update the contained parameter
void AP_OSD_ParamSetting::update()
{
    // if the user has not made any changes then skip the update
    if (PARAM_TOKEN_INDEX(_current_token) == PARAM_COMPOSITE_INDEX(_param_key, _param_idx, _param_group) && _param_key >= 0) {
        return;
    }
    // if a parameter was configured then use that
    _current_token = AP_Param::ParamToken {};
    // surely there is a more efficient way than brute-force search
    for (_param = AP_Param::first(&_current_token, &_param_type);
        _param && (AP_Param::get_persistent_key(_current_token.key) != uint16_t(_param_key.get())
            || _current_token.idx != uint8_t(_param_idx.get())
            || _current_token.group_element != uint32_t(_param_group.get()));
        _param = AP_Param::next_scalar(&_current_token, &_param_type)) {
    }

    if (_param == nullptr) {
        enabled.set(false);
    } else {
        guess_ranges();
    }
}

// update parameter settings from the named parameter
bool AP_OSD_ParamSetting::set_by_name(const char* name, uint8_t config_type, float pmin, float pmax, float pincr)
{
    AP_Param::ParamToken token = AP_Param::ParamToken {};
    ap_var_type type;
    AP_Param* param = AP_Param::find_by_name(name, &type, &token);

    if (param == nullptr) {
        // leave unchanged
        return false;
    } else {
        _current_token = token;
        _param_type = type;
        _param = param;
        enabled.set_and_save_ifchanged(true);
    }

    _type.set_and_save_ifchanged(config_type);

    if (config_type == OSD_PARAM_NONE && !is_zero(pincr)) {
        // ranges
        _param_min.set_and_save_ifchanged(pmin);
        _param_max.set_and_save_ifchanged(pmax);
        _param_incr.set_and_save_ifchanged(pincr);
    } else {
        guess_ranges(true);
    }

    _param_key.set_and_save_ifchanged(AP_Param::get_persistent_key(_current_token.key));
    _param_idx.set_and_save_ifchanged(_current_token.idx);
    _param_group.set_and_save_ifchanged(_current_token.group_element);
    return true;
}

// guess the ranges and increment for the selected parameter
// only called when a change has been made
void AP_OSD_ParamSetting::guess_ranges(bool force)
{
    if (_param->is_read_only()) {
        return;
    }

    // check for statically configured setting metadata
    if (set_from_metadata()) {
        return;
    }

    // nothing statically configured so guess some appropriate values
    float min = -1, max = 127, incr = 1;

    if (_param != nullptr) {
        switch (_param_type) {
        case AP_PARAM_INT8:
            break;
        case AP_PARAM_INT16: {
            AP_Int16* p = (AP_Int16*)_param;
            min = -1;
            uint8_t digits = 0;
            for (int16_t int16p = p->get(); int16p > 0; int16p /= 10) {
                digits++;
            }
            incr = MAX(1, powf(10, digits - 2));
            max = powf(10, digits + 1);
            debug("Guessing range for value %d as %f -> %f, %f\n", p->get(), min, max, incr);
            break;
        }
        case AP_PARAM_INT32: {
            AP_Int32* p = (AP_Int32*)_param;
            min = -1;
            uint8_t digits = 0;
            for (int32_t int32p = p->get(); int32p > 0; int32p /= 10) {
                digits++;
            }
            incr = MAX(1, powf(10, digits - 2));
            max = powf(10, digits + 1);
            debug("Guessing range for value %d as %f -> %f, %f\n", int(p->get()), min, max, incr);
            break;
        }
        case AP_PARAM_FLOAT: {
            AP_Float* p = (AP_Float*)_param;

            uint8_t digits = 0;
            for (float floatp = p->get(); floatp > 1.0f; floatp /= 10) {
                digits++;
            }
            float floatp = p->get();
            if (digits < 1) {
                if (!is_zero(floatp)) {
                    incr = floatp * 0.01f; // move in 1% increments
                } else {
                    incr = 0.01f; // move in absolute 1% increments
                }
                max = 1.0;
                min = 0.0f;
            } else {
                if (!is_zero(floatp)) {
                    incr = floatp * 0.01f; // move in 1% increments
                } else {
                    incr = MAX(1, powf(10, digits - 2));
                }
                max = powf(10, digits + 1);
                min = 0.0f;
            }
            debug("Guessing range for value %f as %f -> %f, %f\n", p->get(), min, max, incr);
            break;
        }
        case AP_PARAM_VECTOR3F:
        case AP_PARAM_NONE:
        case AP_PARAM_GROUP:
            break;
        }

        if (force || !_param_min.configured()) {
            _param_min.set(min);
        }
        if (force || !_param_max.configured()) {
            _param_max.set(max);
        }
        if (force || !_param_incr.configured()) {
            _param_incr.set(incr);
        }
    }
}

// copy the name converting FOO_BAR_BAZ to FooBarBaz
void AP_OSD_ParamSetting::copy_name_camel_case(char* name, size_t len) const
{
    char buf[17];
    _param->copy_name_token(_current_token, buf, 17);
    buf[16] = 0;
    name[0] = buf[0];
    for (uint8_t i = 1, n = 1; i < len; i++, n++) {
        if (buf[i] == '_') {
            name[n] = buf[i+1];
            i++;
        } else {
            name[n] = tolower(buf[i]);
        }
    }
}

bool AP_OSD_ParamSetting::set_from_metadata()
{
    // check for statically configured setting metadata
    if (_type > 0 && _type < OSD_PARAM_NUM_TYPES && _param_metadata[_type - 1].values_max > 0) {
        _param_incr.set(_param_metadata[_type - 1].increment);
        _param_min.set(_param_metadata[_type - 1].min_value);
        _param_max.set(_param_metadata[_type - 1].max_value);
        return true;
    }
    return false;
}

// modify the selected parameter values
void AP_OSD_ParamSetting::save_as_new()
{
    _param_group.save();
    _param_key.save();
    _param_idx.save();
    // the user has configured the range and increment, but the parameter
    // is no longer valid so reset these to guessed values
    guess_ranges(true);
    if (_param_min.configured()) {
        _param_min.save();
    }
    if (_param_max.configured()) {
        _param_max.save();
    }
    if (_param_incr.configured()) {
        _param_incr.save();
    }
}

#endif // OSD_PARAM_ENABLED

