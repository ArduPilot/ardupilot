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
#include "AP_GPS_config.h"

#if AP_GPS_ENABLED

#include "AP_GPS.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Notify/AP_Notify.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_RTC/AP_RTC.h>
#include <climits>
#include <AP_SerialManager/AP_SerialManager.h>

#include "AP_GPS_NOVA.h"
#include "AP_GPS_Blended.h"
#include "AP_GPS_ERB.h"
#include "AP_GPS_GSOF.h"
#include "AP_GPS_NMEA.h"
#include "AP_GPS_SBF.h"
#include "AP_GPS_SBP.h"
#include "AP_GPS_SBP2.h"
#include "AP_GPS_SIRF.h"
#include "AP_GPS_UBLOX.h"
#include "AP_GPS_MAV.h"
#include "AP_GPS_MSP.h"
#include "AP_GPS_ExternalAHRS.h"
#include "GPS_Backend.h"
#if HAL_SIM_GPS_ENABLED
#include "AP_GPS_SITL.h"
#endif

#if HAL_ENABLE_DRONECAN_DRIVERS
#include <AP_CANManager/AP_CANManager.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include "AP_GPS_DroneCAN.h"
#endif

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include "AP_GPS_FixType.h"

#if AP_GPS_RTCM_DECODE_ENABLED
#include "RTCM3_Parser.h"
#endif

#if !AP_GPS_BLENDED_ENABLED
#if defined(GPS_BLENDED_INSTANCE)
#error GPS_BLENDED_INSTANCE should not be defined when AP_GPS_BLENDED_ENABLED is false
#endif
#endif

#define GPS_RTK_INJECT_TO_ALL 127
#ifndef GPS_MAX_RATE_MS
#define GPS_MAX_RATE_MS 200 // maximum value of rate_ms (i.e. slowest update rate) is 5hz or 200ms
#endif
#define GPS_BAUD_TIME_MS 1200
#define GPS_TIMEOUT_MS 4000u

extern const AP_HAL::HAL &hal;

// baudrates to try to detect GPSes with
const uint32_t AP_GPS::_baudrates[] = {9600U, 115200U, 4800U, 19200U, 38400U, 57600U, 230400U, 460800U};

// initialisation blobs to send to the GPS to try to get it into the
// right mode.
const char AP_GPS::_initialisation_blob[] =
#if AP_GPS_UBLOX_ENABLED
    UBLOX_SET_BINARY_230400
#endif
#if AP_GPS_SIRF_ENABLED
    SIRF_SET_BINARY
#endif
#if AP_GPS_NMEA_UNICORE_ENABLED
    NMEA_UNICORE_SETUP
#endif
    ""   // to compile we need *some_initialiser if all backends compiled out
    ;

#if HAL_GCS_ENABLED
// ensure that our GPS_Status enumeration is 1:1 with the mavlink
// numbers of the same fix type.  This allows us to do a simple cast
// from one to the other when sending GPS mavlink messages, rather
// than having some sort of mapping function from our internal
// enumeration into the mavlink enumeration.  Doing things this way
// has two advantages - in the future we could add that mapping
// function and change our enumeration, and the other is that it
// allows us to build the GPS library without having the mavlink
// headers built (for example, in AP_Periph we shouldn't need mavlink
// headers).
static_assert((uint32_t)AP_GPS::GPS_Status::NO_GPS == (uint32_t)GPS_FIX_TYPE_NO_GPS, "NO_GPS incorrect");
static_assert((uint32_t)AP_GPS::GPS_Status::NO_FIX == (uint32_t)GPS_FIX_TYPE_NO_FIX, "NO_FIX incorrect");
static_assert((uint32_t)AP_GPS::GPS_Status::GPS_OK_FIX_2D == (uint32_t)GPS_FIX_TYPE_2D_FIX, "FIX_2D incorrect");
static_assert((uint32_t)AP_GPS::GPS_Status::GPS_OK_FIX_3D == (uint32_t)GPS_FIX_TYPE_3D_FIX, "FIX_3D incorrect");
static_assert((uint32_t)AP_GPS::GPS_Status::GPS_OK_FIX_3D_DGPS == (uint32_t)GPS_FIX_TYPE_DGPS, "FIX_DGPS incorrect");
static_assert((uint32_t)AP_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FLOAT == (uint32_t)GPS_FIX_TYPE_RTK_FLOAT, "FIX_RTK_FLOAT incorrect");
static_assert((uint32_t)AP_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FIXED == (uint32_t)GPS_FIX_TYPE_RTK_FIXED, "FIX_RTK_FIXED incorrect");
#endif

// ensure that our own enum-class status is equivalent to the
// ArduPilot-scoped AP_GPS_FixType enumeration:
static_assert((uint32_t)AP_GPS::GPS_Status::NO_GPS == (uint8_t)AP_GPS_FixType::NO_GPS, "NO_GPS incorrect");
static_assert((uint32_t)AP_GPS::GPS_Status::NO_FIX == (uint8_t)AP_GPS_FixType::NONE, "NO_FIX incorrect");
static_assert((uint32_t)AP_GPS::GPS_Status::GPS_OK_FIX_2D == (uint8_t)AP_GPS_FixType::FIX_2D, "FIX_2D incorrect");
static_assert((uint32_t)AP_GPS::GPS_Status::GPS_OK_FIX_3D == (uint8_t)AP_GPS_FixType::FIX_3D, "FIX_3D incorrect");
static_assert((uint32_t)AP_GPS::GPS_Status::GPS_OK_FIX_3D_DGPS == (uint8_t)AP_GPS_FixType::DGPS, "FIX_DGPS incorrect");
static_assert((uint32_t)AP_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FLOAT == (uint8_t)AP_GPS_FixType::RTK_FLOAT, "FIX_RTK_FLOAT incorrect");
static_assert((uint32_t)AP_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FIXED == (uint8_t)AP_GPS_FixType::RTK_FIXED, "FIX_RTK_FIXED incorrect");

AP_GPS *AP_GPS::_singleton;

// table of user settable parameters
const AP_Param::GroupInfo AP_GPS::var_info[] = {

    // 0 was GPS_TYPE

    // 1 was GPS_TYPE2

    // @Param: _NAVFILTER
    // @DisplayName: Navigation filter setting
    // @Description: Navigation filter engine setting
    // @Values: 0:Portable,2:Stationary,3:Pedestrian,4:Automotive,5:Sea,6:Airborne1G,7:Airborne2G,8:Airborne4G
    // @User: Advanced
    AP_GROUPINFO("_NAVFILTER", 2, AP_GPS, _navfilter, GPS_ENGINE_AIRBORNE_4G),

#if GPS_MAX_RECEIVERS > 1
    // @Param: _AUTO_SWITCH
    // @DisplayName: Automatic Switchover Setting
    // @Description: Automatic switchover to GPS reporting best lock, 1:UseBest selects the GPS with highest status, if both are equal the GPS with highest satellite count is used 4:Use primary if 3D fix or better, will revert to 'UseBest' behaviour if 3D fix is lost on primary
    // @Values: 0:Use primary, 1:UseBest, 2:Blend, 4:Use primary if 3D fix or better
    // @User: Advanced
    AP_GROUPINFO("_AUTO_SWITCH", 3, AP_GPS, _auto_switch, (int8_t)GPSAutoSwitch::USE_BEST),
#endif

    // 4 was _MIN_GPS, removed Feb 2024

    // @Param: _SBAS_MODE
    // @DisplayName: SBAS Mode
    // @Description: This sets the SBAS (satellite based augmentation system) mode if available on this GPS. If set to 2 then the SBAS mode is not changed in the GPS. Otherwise the GPS will be reconfigured to enable/disable SBAS. Disabling SBAS may be worthwhile in some parts of the world where an SBAS signal is available but the baseline is too long to be useful.
    // @Values: 0:Disabled,1:Enabled,2:NoChange
    // @User: Advanced
    AP_GROUPINFO("_SBAS_MODE", 5, AP_GPS, _sbas_mode, 2),

    // @Param: _MIN_ELEV
    // @DisplayName: Minimum elevation
    // @Description: This sets the minimum elevation of satellites above the horizon for them to be used for navigation. Setting this to -100 leaves the minimum elevation set to the GPS modules default.
    // @Range: -100 90
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("_MIN_ELEV", 6, AP_GPS, _min_elevation, -100),

    // @Param: _INJECT_TO
    // @DisplayName: Destination for GPS_INJECT_DATA MAVLink packets
    // @Description: The GGS can send raw serial packets to inject data to multiple GPSes.
    // @Values: 0:send to first GPS,1:send to 2nd GPS,127:send to all
    // @User: Advanced
    AP_GROUPINFO("_INJECT_TO",   7, AP_GPS, _inject_to, GPS_RTK_INJECT_TO_ALL),

#if AP_GPS_SBP2_ENABLED || AP_GPS_SBP_ENABLED
    // @Param: _SBP_LOGMASK
    // @DisplayName: Swift Binary Protocol Logging Mask
    // @Description: Masked with the SBP msg_type field to determine whether SBR1/SBR2 data is logged
    // @Values: 0:None (0x0000),-1:All (0xFFFF),-256:External only (0xFF00)
    // @User: Advanced
    AP_GROUPINFO("_SBP_LOGMASK", 8, AP_GPS, _sbp_logmask, -256),
#endif //AP_GPS_SBP2_ENABLED || AP_GPS_SBP_ENABLED

    // @Param: _RAW_DATA
    // @DisplayName: Raw data logging
    // @Description: Handles logging raw data; on uBlox chips that support raw data this will log RXM messages into logger; on Septentrio this will log on the equipment's SD card and when set to 2, the autopilot will try to stop logging after disarming and restart after arming
    // @Values: 0:Ignore,1:Always log,2:Stop logging when disarmed (SBF only),5:Only log every five samples (uBlox only)
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("_RAW_DATA", 9, AP_GPS, _raw_data, 0),

    // 10 was GPS_GNSS_MODE

    // @Param: _SAVE_CFG
    // @DisplayName: Save GPS configuration
    // @Description: Determines whether the configuration for this GPS should be written to non-volatile memory on the GPS. Currently working for UBlox 6 series and above.
    // @Values: 0:Do not save config,1:Save config,2:Save only when needed
    // @User: Advanced
    AP_GROUPINFO("_SAVE_CFG", 11, AP_GPS, _save_config, 2),

    // 12 was GPS_GNSS_MODE2

    // @Param: _AUTO_CONFIG
    // @DisplayName: Automatic GPS configuration
    // @Description: Controls if the autopilot should automatically configure the GPS based on the parameters and default settings
    // @Values: 0:Disables automatic configuration,1:Enable automatic configuration for Serial GPSes only,2:Enable automatic configuration for DroneCAN as well
    // @User: Advanced
    AP_GROUPINFO("_AUTO_CONFIG", 13, AP_GPS, _auto_config, 1),

    // 14 was GPS_RATE_MS

    // 15 was GPS_RATE_MS2

    // 16 was GPS_POS1

    // 17 was GPS_POS2

    // 18 was GPS_DELAY_MS

    // 19 was GPS_DELAY_MS2

#if AP_GPS_BLENDED_ENABLED
    // @Param: _BLEND_MASK
    // @DisplayName: Multi GPS Blending Mask
    // @Description: Determines which of the accuracy measures Horizontal position, Vertical Position and Speed are used to calculate the weighting on each GPS receiver when soft switching has been selected by setting GPS_AUTO_SWITCH to 2(Blend)
    // @Bitmask: 0:Horiz Pos,1:Vert Pos,2:Speed
    // @User: Advanced
    AP_GROUPINFO("_BLEND_MASK", 20, AP_GPS, _blend_mask, 5),

    // @Param: _BLEND_TC
    // @DisplayName: Blending time constant
    // @Description: Controls the slowest time constant applied to the calculation of GPS position and height offsets used to adjust different GPS receivers for steady state position differences.
    // @Units: s
    // @Range: 5.0 30.0
    // @User: Advanced
    // Had key 21, no longer used
#endif

    // @Param: _DRV_OPTIONS
    // @DisplayName: driver options
    // @Description: Additional backend specific options
    // @Bitmask: 0:Use UART2 for moving baseline on ublox,1:Use base station for GPS yaw on SBF,2:Use baudrate 115200,3:Use dedicated CAN port b/w GPSes for moving baseline,4:Use ellipsoid height instead of AMSL, 5:Override GPS satellite health of L5 band from L1 health, 6:Enable RTCM full parse even for a single channel, 7:Disable automatic full RTCM parsing when RTCM seen on more than one channel
    // @User: Advanced
    AP_GROUPINFO("_DRV_OPTIONS", 22, AP_GPS, _driver_options, 0),

    // 23 was GPS_COM_PORT

    // 24 was GPS_COM_PORT2

    // 25 was GPS_MB1_*

    // 26 was GPS_MB2_*

#if GPS_MAX_RECEIVERS > 1
    // @Param: _PRIMARY
    // @DisplayName: Primary GPS
    // @Description: This GPS will be used when GPS_AUTO_SWITCH is 0 and used preferentially with GPS_AUTO_SWITCH = 4.
    // @Increment: 1
    // @Values: 0:FirstGPS, 1:SecondGPS
    // @User: Advanced
    AP_GROUPINFO("_PRIMARY", 27, AP_GPS, _primary, 0),
#endif

    // 28 was GPS_CAN_NODEID1

    // 29 was GPS_CAN_NODEID2

    // 30 was GPS1_CAN_OVRIDE

    // 31 was GPS2_CAN_OVRIDE

    // @Group: 1_
    // @Path: AP_GPS_Params.cpp
    AP_SUBGROUPINFO(params[0], "1_", 32, AP_GPS, AP_GPS::Params),

#if GPS_MAX_RECEIVERS > 1
    // @Group: 2_
    // @Path: AP_GPS_Params.cpp
    AP_SUBGROUPINFO(params[1], "2_", 33, AP_GPS, AP_GPS::Params),
#endif

    AP_GROUPEND
};

// constructor
AP_GPS::AP_GPS()
{
    static_assert((sizeof(_initialisation_blob) * (CHAR_BIT + 2)) < (4800 * GPS_BAUD_TIME_MS * 1e-3),
                    "GPS initilisation blob is too large to be completely sent before the baud rate changes");

    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_GPS must be singleton");
    }
    _singleton = this;
}

// return true if a specific type of GPS uses a UART
bool AP_GPS::needs_uart(GPS_Type type) const
{
    switch (type) {
    case GPS_TYPE_NONE:
    case GPS_TYPE_HIL:
    case GPS_TYPE_UAVCAN:
    case GPS_TYPE_UAVCAN_RTK_BASE:
    case GPS_TYPE_UAVCAN_RTK_ROVER:
    case GPS_TYPE_MAV:
    case GPS_TYPE_MSP:
    case GPS_TYPE_EXTERNAL_AHRS:
        return false;
    default:
        break;
    }
    return true;
}

/// Startup initialisation.
void AP_GPS::init()
{
    // set the default for the first GPS according to define:
    params[0].type.set_default(HAL_GPS1_TYPE_DEFAULT);

    convert_parameters();

    // Set new primary param based on old auto_switch use second option
    if ((_auto_switch.get() == 3) && !_primary.configured()) {
        _primary.set_and_save(1);
        _auto_switch.set_and_save(0);
    }

    // search for serial ports with gps protocol
    const auto &serial_manager = AP::serialmanager();
    uint8_t uart_idx = 0;
    for (uint8_t i=0; i<ARRAY_SIZE(params); i++) {
        if (needs_uart(params[i].type)) {
            _port[i] = serial_manager.find_serial(AP_SerialManager::SerialProtocol_GPS, uart_idx);
            uart_idx++;
        }
    }
    _last_instance_swap_ms = 0;

    // prep the state instance fields
    for (uint8_t i = 0; i < GPS_MAX_INSTANCES; i++) {
        state[i].instance = i;
    }

    // sanity check update rate
    for (uint8_t i=0; i<ARRAY_SIZE(params); i++) {
        auto &rate_ms = params[i].rate_ms;
        if (rate_ms <= 0 || rate_ms > GPS_MAX_RATE_MS) {
            rate_ms.set(GPS_MAX_RATE_MS);
        }
    }

    // create the blended instance if appropriate:
#if AP_GPS_BLENDED_ENABLED
    drivers[GPS_BLENDED_INSTANCE] = NEW_NOTHROW AP_GPS_Blended(*this, params[GPS_BLENDED_INSTANCE], state[GPS_BLENDED_INSTANCE], timing[GPS_BLENDED_INSTANCE]);
#endif
}

void AP_GPS::convert_parameters()
{
    // find GPS's top level key
    uint16_t k_param_gps_key;
    if (!AP_Param::find_top_level_key_by_pointer(this, k_param_gps_key)) {
        return;
    }

    // table parameters to convert without scaling
    static const AP_Param::ConversionInfo conversion_info[] {
        // PARAMETER_CONVERSION - Added: Mar-2024 for 4.6
        { k_param_gps_key, 0, AP_PARAM_INT8, "GPS1_TYPE" },
        { k_param_gps_key, 1, AP_PARAM_INT8, "GPS2_TYPE" },
        { k_param_gps_key, 10, AP_PARAM_INT8, "GPS1_GNSS_MODE" },
        { k_param_gps_key, 12, AP_PARAM_INT8, "GPS2_GNSS_MODE" },
        { k_param_gps_key, 14, AP_PARAM_INT16, "GPS1_RATE_MS" },
        { k_param_gps_key, 15, AP_PARAM_INT16, "GPS2_RATE_MS" },
        { k_param_gps_key, 16, AP_PARAM_VECTOR3F, "GPS1_POS" },
        { k_param_gps_key, 17, AP_PARAM_VECTOR3F, "GPS2_POS" },
        { k_param_gps_key, 18, AP_PARAM_INT16, "GPS1_DELAY_MS" },
        { k_param_gps_key, 19, AP_PARAM_INT16, "GPS2_DELAY_MS" },
#if AP_GPS_SBF_ENABLED
        { k_param_gps_key, 23, AP_PARAM_INT8, "GPS1_COM_PORT" },
        { k_param_gps_key, 24, AP_PARAM_INT8, "GPS2_COM_PORT" },
#endif

#if HAL_ENABLE_DRONECAN_DRIVERS
        { k_param_gps_key, 28, AP_PARAM_INT32, "GPS1_CAN_NODEID" },
        { k_param_gps_key, 29, AP_PARAM_INT32, "GPS2_CAN_NODEID" },
        { k_param_gps_key, 30, AP_PARAM_INT32, "GPS1_CAN_OVRIDE" },
        { k_param_gps_key, 31, AP_PARAM_INT32, "GPS2_CAN_OVRIDE" },
#endif
    };
    AP_Param::convert_old_parameters(conversion_info, ARRAY_SIZE(conversion_info));

#if GPS_MOVING_BASELINE
    // convert old MovingBaseline parameters
    // PARAMETER_CONVERSION - Added: Mar-2024 for 4.6
    for (uint8_t i=0; i<MIN(2, GPS_MAX_RECEIVERS); i++) {
        // the old _MB parameters were 25 and 26:
        const uint8_t old_index = 25 + i;
        AP_Param::convert_class(k_param_gps_key, &params[i].mb_params, params[i].mb_params.var_info, old_index, false);
    }
#endif  // GPS_MOVING_BASELINE
}

// return number of active GPS sensors. Note that if the first GPS
// is not present but the 2nd is then we return 2. Note that a blended
// GPS solution is treated as an additional sensor.
uint8_t AP_GPS::num_sensors(void) const
{
#if AP_GPS_BLENDED_ENABLED
    if (_output_is_blended) {
        return num_instances+1;
    }
#endif
    return num_instances;
}

bool AP_GPS::speed_accuracy(uint8_t instance, float &sacc) const
{
    if (state[instance].have_speed_accuracy) {
        sacc = state[instance].speed_accuracy;
        return true;
    }
    return false;
}

bool AP_GPS::horizontal_accuracy(uint8_t instance, float &hacc) const
{
    if (state[instance].have_horizontal_accuracy) {
        hacc = state[instance].horizontal_accuracy;
        return true;
    }
    return false;
}

bool AP_GPS::vertical_accuracy(uint8_t instance, float &vacc) const
{
    if (state[instance].have_vertical_accuracy) {
        vacc = state[instance].vertical_accuracy;
        return true;
    }
    return false;
}

AP_GPS::CovarianceType AP_GPS::position_covariance(const uint8_t instance, Matrix3f& cov) const
{
    AP_GPS::CovarianceType cov_type = AP_GPS::CovarianceType::UNKNOWN;
    cov.zero();
    float hacc, vacc;
    if (horizontal_accuracy(instance, hacc) && vertical_accuracy(instance, vacc))
    {
        cov_type = AP_GPS::CovarianceType::DIAGONAL_KNOWN;
        const auto hacc_variance = hacc * hacc;
        cov[0][0] = hacc_variance;
        cov[1][1] = hacc_variance;
        cov[2][2] = vacc * vacc;
    }
    // There may be a receiver that implements hdop+vdop but not accuracy
    // If so, there could be a condition here to attempt to calculate it

    return cov_type;
}


/**
   convert GPS week and milliseconds to unix epoch in milliseconds
 */
uint64_t AP_GPS::istate_time_to_epoch_ms(uint16_t gps_week, uint32_t gps_ms)
{
    uint64_t fix_time_ms = UNIX_OFFSET_MSEC + gps_week * AP_MSEC_PER_WEEK + gps_ms;
    return fix_time_ms;
}

/**
   calculate current time since the unix epoch in microseconds
 */
uint64_t AP_GPS::time_epoch_usec(uint8_t instance) const
{
    const GPS_State &istate = state[instance];
    if ((istate.last_gps_time_ms == 0 && istate.last_corrected_gps_time_us == 0) || istate.time_week == 0) {
        return 0;
    }
    uint64_t fix_time_ms;
    // add in the time since the last fix message
    if (istate.last_corrected_gps_time_us != 0) {
        fix_time_ms = istate_time_to_epoch_ms(istate.time_week, drivers[instance]->get_last_itow_ms());
        return (fix_time_ms*1000ULL) + (AP_HAL::micros64() - istate.last_corrected_gps_time_us);
    } else {
        fix_time_ms = istate_time_to_epoch_ms(istate.time_week, istate.time_week_ms);
        return (fix_time_ms + (AP_HAL::millis() - istate.last_gps_time_ms)) * 1000ULL;
    }
}

/**
   calculate last message time since the unix epoch in microseconds
 */
uint64_t AP_GPS::last_message_epoch_usec(uint8_t instance) const
{
    const GPS_State &istate = state[instance];
    if (istate.time_week == 0) {
        return 0;
    }
    return istate_time_to_epoch_ms(istate.time_week, drivers[instance]->get_last_itow_ms()) * 1000ULL;
}

/*
  send some more initialisation string bytes if there is room in the
  UART transmit buffer
 */
void AP_GPS::send_blob_start(uint8_t instance, const char *_blob, uint16_t size)
{
    initblob_state[instance].blob = _blob;
    initblob_state[instance].remaining = size;
}

/*
  initialise state for sending binary blob initialisation strings.  We
  may choose different blobs not just based on type but also based on
  parameters.
 */
void AP_GPS::send_blob_start(uint8_t instance)
{
    const auto type = params[instance].type;

#if AP_GPS_UBLOX_ENABLED
    if (type == GPS_TYPE_UBLOX && option_set(DriverOptions::UBX_Use115200)) {
        static const char blob[] = UBLOX_SET_BINARY_115200;
        send_blob_start(instance, blob, sizeof(blob));
        return;
    }
#endif // AP_GPS_UBLOX_ENABLED

#if GPS_MOVING_BASELINE && AP_GPS_UBLOX_ENABLED
    if ((type == GPS_TYPE_UBLOX_RTK_BASE ||
         type == GPS_TYPE_UBLOX_RTK_ROVER) &&
        !option_set(DriverOptions::UBX_MBUseUart2)) {
        // we use 460800 when doing moving baseline as we need
        // more bandwidth. We don't do this if using UART2, as
        // in that case the RTCMv3 data doesn't go over the
        // link to the flight controller
        static const char blob[] = UBLOX_SET_BINARY_460800;
        send_blob_start(instance, blob, sizeof(blob));
        return;
    }
#endif

    // the following devices don't have init blobs:
    const char *blob = nullptr;
    uint32_t blob_size = 0;
    switch (GPS_Type(type)) {
#if AP_GPS_SBF_ENABLED
    case GPS_TYPE_SBF:
    case GPS_TYPE_SBF_DUAL_ANTENNA:
#endif //AP_GPS_SBF_ENABLED
#if AP_GPS_GSOF_ENABLED
    case GPS_TYPE_GSOF:
#endif //AP_GPS_GSOF_ENABLED
#if AP_GPS_NOVA_ENABLED
    case GPS_TYPE_NOVA:
#endif //AP_GPS_NOVA_ENABLED
#if HAL_SIM_GPS_ENABLED
    case GPS_TYPE_SITL:
#endif  // HAL_SIM_GPS_ENABLED
        // none of these GPSs have initialisation blobs
        break;
    default:
        // send combined initialisation blob, on the assumption that the
        // GPS units will parse what they need and ignore the data they
        // don't understand:
        blob = _initialisation_blob;
        blob_size = sizeof(_initialisation_blob);
        break;
    }

    send_blob_start(instance, blob, blob_size);
}

/*
  send some more initialisation string bytes if there is room in the
  UART transmit buffer
 */
void AP_GPS::send_blob_update(uint8_t instance)
{
    // exit immediately if no uart for this instance
    if (_port[instance] == nullptr) {
        return;
    }

    if (initblob_state[instance].remaining == 0) {
        return;
    }

    // see if we can write some more of the initialisation blob
    const uint16_t n = MIN(_port[instance]->txspace(),
                           initblob_state[instance].remaining);
    const size_t written = _port[instance]->write((const uint8_t*)initblob_state[instance].blob, n);
    initblob_state[instance].blob += written;
    initblob_state[instance].remaining -= written;
}

/*
  run detection step for one GPS instance. If this finds a GPS then it
  will fill in drivers[instance] and change state[instance].status
  from NO_GPS to NO_FIX.
 */
void AP_GPS::detect_instance(uint8_t instance)
{
    const uint32_t now = AP_HAL::millis();

    state[instance].status = NO_GPS;
    state[instance].hdop = GPS_UNKNOWN_DOP;
    state[instance].vdop = GPS_UNKNOWN_DOP;

    AP_GPS_Backend *new_gps = _detect_instance(instance);
    if (new_gps == nullptr) {
        return;
    }

    state[instance].status = NO_FIX;
    drivers[instance] = new_gps;
    timing[instance].last_message_time_ms = now;
    timing[instance].delta_time_ms = GPS_TIMEOUT_MS;

    new_gps->broadcast_gps_type();
}

/*
  run detection step for one GPS instance. If this finds a GPS then it
  will return it - otherwise nullptr
 */
AP_GPS_Backend *AP_GPS::_detect_instance(uint8_t instance)
{
    struct detect_state *dstate = &detect_state[instance];

    const auto type = params[instance].type;

    switch (GPS_Type(type)) {
    // user has to explicitly set the MAV type, do not use AUTO
    // do not try to detect the MAV type, assume it's there
    case GPS_TYPE_MAV:
#if AP_GPS_MAV_ENABLED
        dstate->auto_detected_baud = false; // specified, not detected
        return NEW_NOTHROW AP_GPS_MAV(*this, params[instance], state[instance], nullptr);
#endif //AP_GPS_MAV_ENABLED

    // user has to explicitly set the UAVCAN type, do not use AUTO
    case GPS_TYPE_UAVCAN:
    case GPS_TYPE_UAVCAN_RTK_BASE:
    case GPS_TYPE_UAVCAN_RTK_ROVER:
#if AP_GPS_DRONECAN_ENABLED
        dstate->auto_detected_baud = false; // specified, not detected
        return AP_GPS_DroneCAN::probe(*this, state[instance]);
#endif
        return nullptr; // We don't do anything here if UAVCAN is not supported
#if HAL_MSP_GPS_ENABLED
    case GPS_TYPE_MSP:
        dstate->auto_detected_baud = false; // specified, not detected
        return NEW_NOTHROW AP_GPS_MSP(*this, params[instance], state[instance], nullptr);
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    case GPS_TYPE_EXTERNAL_AHRS:
        if (AP::externalAHRS().get_port(AP_ExternalAHRS::AvailableSensor::GPS) >= 0) {
            dstate->auto_detected_baud = false; // specified, not detected
            return NEW_NOTHROW AP_GPS_ExternalAHRS(*this, params[instance], state[instance], nullptr);
        }
        break;
#endif
#if AP_GPS_GSOF_ENABLED
    case GPS_TYPE_GSOF:
        dstate->auto_detected_baud = false; // specified, not detected
        return NEW_NOTHROW AP_GPS_GSOF(*this, params[instance], state[instance], _port[instance]);
#endif //AP_GPS_GSOF_ENABLED
    default:
        break;
    }

    if (_port[instance] == nullptr) {
        // UART not available
        return nullptr;
    }

    // all remaining drivers automatically cycle through baud rates to detect
    // the correct baud rate, and should have the selected baud broadcast
    dstate->auto_detected_baud = true;
    const uint32_t now = AP_HAL::millis();

    if (now - dstate->last_baud_change_ms > GPS_BAUD_TIME_MS) {
        // try the next baud rate
        // incrementing like this will skip the first element in array of bauds
        // this is okay, and relied upon
        if (dstate->probe_baud == 0) {
            dstate->probe_baud = _port[instance]->get_baud_rate();
        } else {
            dstate->current_baud++;
            if (dstate->current_baud == ARRAY_SIZE(_baudrates)) {
                dstate->current_baud = 0;
            }
            dstate->probe_baud = _baudrates[dstate->current_baud];
        }
        uint16_t rx_size=0, tx_size=0;
        if (type == GPS_TYPE_UBLOX_RTK_ROVER) {
            tx_size = 2048;
        }
        if (type == GPS_TYPE_UBLOX_RTK_BASE) {
            rx_size = 2048;
        }
        _port[instance]->begin(dstate->probe_baud, rx_size, tx_size);
        _port[instance]->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        dstate->last_baud_change_ms = now;

        if (_auto_config >= GPS_AUTO_CONFIG_ENABLE_SERIAL_ONLY) {
            send_blob_start(instance);
        }
    }

    if (_auto_config >= GPS_AUTO_CONFIG_ENABLE_SERIAL_ONLY) {
        send_blob_update(instance);
    }

    switch (GPS_Type(type)) {
#if AP_GPS_SBF_ENABLED
    // by default the sbf/trimble gps outputs no data on its port, until configured.
    case GPS_TYPE_SBF:
    case GPS_TYPE_SBF_DUAL_ANTENNA:
        return NEW_NOTHROW AP_GPS_SBF(*this, params[instance], state[instance], _port[instance]);
#endif //AP_GPS_SBF_ENABLED
#if AP_GPS_NOVA_ENABLED
    case GPS_TYPE_NOVA:
        return NEW_NOTHROW AP_GPS_NOVA(*this, params[instance], state[instance], _port[instance]);
#endif //AP_GPS_NOVA_ENABLED

#if HAL_SIM_GPS_ENABLED
    case GPS_TYPE_SITL:
        return NEW_NOTHROW AP_GPS_SITL(*this, params[instance], state[instance], _port[instance]);
#endif  // HAL_SIM_GPS_ENABLED

    default:
        break;
    }

    if (initblob_state[instance].remaining != 0) {
        // don't run detection engines if we haven't sent out the initblobs
        return nullptr;
    }

    uint16_t bytecount = MIN(8192U, _port[instance]->available());

    while (bytecount-- > 0) {
        const uint8_t data = _port[instance]->read();
        (void)data;  // if all backends are compiled out then "data" is unused

#if AP_GPS_UBLOX_ENABLED
        if ((type == GPS_TYPE_AUTO ||
             type == GPS_TYPE_UBLOX) &&
            ((!_auto_config && _baudrates[dstate->current_baud] >= 38400) ||
             (_baudrates[dstate->current_baud] >= 115200 && option_set(DriverOptions::UBX_Use115200)) ||
             _baudrates[dstate->current_baud] == 230400) &&
            AP_GPS_UBLOX::_detect(dstate->ublox_detect_state, data)) {
            return NEW_NOTHROW AP_GPS_UBLOX(*this, params[instance], state[instance], _port[instance], GPS_ROLE_NORMAL);
        }

        const uint32_t ublox_mb_required_baud = option_set(DriverOptions::UBX_MBUseUart2)?230400:460800;
        if ((type == GPS_TYPE_UBLOX_RTK_BASE ||
             type == GPS_TYPE_UBLOX_RTK_ROVER) &&
            _baudrates[dstate->current_baud] == ublox_mb_required_baud &&
            AP_GPS_UBLOX::_detect(dstate->ublox_detect_state, data)) {
            GPS_Role role;
            if (type == GPS_TYPE_UBLOX_RTK_BASE) {
                role = GPS_ROLE_MB_BASE;
            } else {
                role = GPS_ROLE_MB_ROVER;
            }
            return NEW_NOTHROW AP_GPS_UBLOX(*this, params[instance], state[instance], _port[instance], role);
        }
#endif  // AP_GPS_UBLOX_ENABLED
#if AP_GPS_SBP2_ENABLED
        if ((type == GPS_TYPE_AUTO || type == GPS_TYPE_SBP) &&
                 AP_GPS_SBP2::_detect(dstate->sbp2_detect_state, data)) {
            return NEW_NOTHROW AP_GPS_SBP2(*this, params[instance], state[instance], _port[instance]);
        }
#endif //AP_GPS_SBP2_ENABLED
#if AP_GPS_SBP_ENABLED
        if ((type == GPS_TYPE_AUTO || type == GPS_TYPE_SBP) &&
                 AP_GPS_SBP::_detect(dstate->sbp_detect_state, data)) {
            return NEW_NOTHROW AP_GPS_SBP(*this, params[instance], state[instance], _port[instance]);
        }
#endif //AP_GPS_SBP_ENABLED
#if AP_GPS_SIRF_ENABLED
        if ((type == GPS_TYPE_AUTO || type == GPS_TYPE_SIRF) &&
                 AP_GPS_SIRF::_detect(dstate->sirf_detect_state, data)) {
            return NEW_NOTHROW AP_GPS_SIRF(*this, params[instance], state[instance], _port[instance]);
        }
#endif
#if AP_GPS_ERB_ENABLED
        if ((type == GPS_TYPE_AUTO || type == GPS_TYPE_ERB) &&
                 AP_GPS_ERB::_detect(dstate->erb_detect_state, data)) {
            return NEW_NOTHROW AP_GPS_ERB(*this, params[instance], state[instance], _port[instance]);
        }
#endif // AP_GPS_ERB_ENABLED
#if AP_GPS_NMEA_ENABLED
        if ((type == GPS_TYPE_NMEA ||
                    type == GPS_TYPE_HEMI ||
#if AP_GPS_NMEA_UNICORE_ENABLED
                    type == GPS_TYPE_UNICORE_NMEA ||
                    type == GPS_TYPE_UNICORE_MOVINGBASE_NMEA ||
#endif
                    type == GPS_TYPE_ALLYSTAR) &&
                   AP_GPS_NMEA::_detect(dstate->nmea_detect_state, data)) {
            return NEW_NOTHROW AP_GPS_NMEA(*this, params[instance], state[instance], _port[instance]);
        }
#endif //AP_GPS_NMEA_ENABLED
    }

    return nullptr;
}

AP_GPS::GPS_Status AP_GPS::highest_supported_status(uint8_t instance) const
{
    if (instance < GPS_MAX_RECEIVERS && drivers[instance] != nullptr) {
        return drivers[instance]->highest_supported_status();
    }
    return AP_GPS::GPS_OK_FIX_3D;
}

#if HAL_LOGGING_ENABLED
bool AP_GPS::should_log() const
{
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger == nullptr) {
        return false;
    }
    if (_log_gps_bit == (uint32_t)-1) {
        return false;
    }
    if (!logger->should_log(_log_gps_bit)) {
        return false;
    }
    return true;
}
#endif


/*
  update one GPS instance. This should be called at 10Hz or greater
 */
void AP_GPS::update_instance(uint8_t instance)
{
    const auto type = params[instance].type;
    if (type == GPS_TYPE_HIL) {
        // in HIL, leave info alone
        return;
    }
    if (type == GPS_TYPE_NONE) {
        // not enabled
        state[instance].status = NO_GPS;
        state[instance].hdop = GPS_UNKNOWN_DOP;
        state[instance].vdop = GPS_UNKNOWN_DOP;
        return;
    }
    if (locked_ports & (1U<<instance)) {
        // the port is locked by another driver
        return;
    }

    if (drivers[instance] == nullptr) {
        // we don't yet know the GPS type of this one, or it has timed
        // out and needs to be re-initialised
        detect_instance(instance);
        return;
    }

    if (_auto_config >= GPS_AUTO_CONFIG_ENABLE_SERIAL_ONLY) {
        send_blob_update(instance);
    }

    // we have an active driver for this instance
    bool result = drivers[instance]->read();
    uint32_t tnow = AP_HAL::millis();

    // if we did not get a message, and the idle timer of 2 seconds
    // has expired, re-initialise the GPS. This will cause GPS
    // detection to run again
    bool data_should_be_logged = false;
    if (!result) {
        if (tnow - timing[instance].last_message_time_ms > GPS_TIMEOUT_MS) {
            memset((void *)&state[instance], 0, sizeof(state[instance]));
            state[instance].instance = instance;
            state[instance].hdop = GPS_UNKNOWN_DOP;
            state[instance].vdop = GPS_UNKNOWN_DOP;
            timing[instance].last_message_time_ms = tnow;
            timing[instance].delta_time_ms = GPS_TIMEOUT_MS;
            // do not try to detect again if type is MAV or UAVCAN
            if (type == GPS_TYPE_MAV ||
                type == GPS_TYPE_UAVCAN ||
                type == GPS_TYPE_UAVCAN_RTK_BASE ||
                type == GPS_TYPE_UAVCAN_RTK_ROVER) {
                state[instance].status = NO_FIX;
            } else {
                // free the driver before we run the next detection, so we
                // don't end up with two allocated at any time
                delete drivers[instance];
                drivers[instance] = nullptr;
                state[instance].status = NO_GPS;
            }
            // log this data as a "flag" that the GPS is no longer
            // valid (see PR#8144)
            data_should_be_logged = true;
        }
    } else {
        if (state[instance].corrected_timestamp_updated) {
            // set the timestamp for this messages based on
            // set_uart_timestamp() or per specific transport in backend
            // , if available
            tnow = state[instance].last_corrected_gps_time_us/1000U;
            state[instance].corrected_timestamp_updated = false;
        }

        // announce the GPS type once
        if (!state[instance].announced_detection) {
            state[instance].announced_detection = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GPS %d: detected %s", instance + 1, drivers[instance]->name());
        }

        // delta will only be correct after parsing two messages
        timing[instance].delta_time_ms = tnow - timing[instance].last_message_time_ms;
        timing[instance].last_message_time_ms = tnow;
        // if GPS disabled for flight testing then don't update fix timing value
        if (state[instance].status >= GPS_OK_FIX_2D && !_force_disable_gps) {
            timing[instance].last_fix_time_ms = tnow;
        }

        data_should_be_logged = true;
    }

#if GPS_MAX_RECEIVERS > 1
    if (drivers[instance] && type == GPS_TYPE_UBLOX_RTK_BASE) {
        // see if a moving baseline base has some RTCMv3 data
        // which we need to pass along to the rover
        const uint8_t *rtcm_data;
        uint16_t rtcm_len;
        if (drivers[instance]->get_RTCMV3(rtcm_data, rtcm_len)) {
            for (uint8_t i=0; i< GPS_MAX_RECEIVERS; i++) {
                if (i != instance && params[i].type == GPS_TYPE_UBLOX_RTK_ROVER) {
                    // pass the data to the rover
                    inject_data(i, rtcm_data, rtcm_len);
                    drivers[instance]->clear_RTCMV3();
                    break;
                }
            }
        }
    }
#endif

    if (data_should_be_logged) {
        // keep count of delayed frames and average frame delay for health reporting
        const uint16_t gps_max_delta_ms = 245; // 200 ms (5Hz) + 45 ms buffer
        GPS_timing &t = timing[instance];

        if (t.delta_time_ms > gps_max_delta_ms) {
            t.delayed_count++;
        } else {
            t.delayed_count = 0;
        }
        if (t.delta_time_ms < 2000) {
            if (t.average_delta_ms <= 0) {
                t.average_delta_ms = t.delta_time_ms;
            } else {
                t.average_delta_ms = 0.98f * t.average_delta_ms + 0.02f * t.delta_time_ms;
            }
        }
    }

#if HAL_LOGGING_ENABLED
    if (data_should_be_logged && should_log()) {
        Write_GPS(instance);
    }
#else
    (void)data_should_be_logged;
#endif

#if AP_RTC_ENABLED
    if (state[instance].status >= GPS_OK_FIX_3D) {
        const uint64_t now = time_epoch_usec(instance);
        if (now != 0) {
            AP::rtc().set_utc_usec(now, AP_RTC::SOURCE_GPS);
        }
    }
#endif
}


#if GPS_MOVING_BASELINE
bool AP_GPS::get_RelPosHeading(uint32_t &timestamp, float &relPosHeading, float &relPosLength, float &relPosD, float &accHeading)
{
    for (uint8_t i=0; i< GPS_MAX_RECEIVERS; i++) {
        if (drivers[i] &&
            state[i].relposheading_ts != 0 &&
            AP_HAL::millis() - state[i].relposheading_ts < 500) {
           relPosHeading = state[i].relPosHeading;
           relPosLength = state[i].relPosLength;
           relPosD = state[i].relPosD;
           accHeading = state[i].accHeading;
           timestamp = state[i].relposheading_ts;
           return true;
        }
    }
    return false;
}

bool AP_GPS::get_RTCMV3(const uint8_t *&bytes, uint16_t &len)
{
    for (uint8_t i=0; i< GPS_MAX_RECEIVERS; i++) {
        if (drivers[i] && params[i].type == GPS_TYPE_UBLOX_RTK_BASE) {
            return drivers[i]->get_RTCMV3(bytes, len);
        }
    }
    return false;
}

void AP_GPS::clear_RTCMV3()
{
    for (uint8_t i=0; i< GPS_MAX_RECEIVERS; i++) {
        if (drivers[i] && params[i].type == GPS_TYPE_UBLOX_RTK_BASE) {
            drivers[i]->clear_RTCMV3();
        }
    }
}

/*
    inject Moving Baseline Data messages.
*/
void AP_GPS::inject_MBL_data(uint8_t* data, uint16_t length)
{
    for (uint8_t i=0; i< GPS_MAX_RECEIVERS; i++) {
        if (params[i].type == GPS_TYPE_UBLOX_RTK_ROVER) {
            // pass the data to the rover
            inject_data(i, data, length);
            break;
        }
    }
}
#endif //#if GPS_MOVING_BASELINE

/*
  update all GPS instances
 */
void AP_GPS::update(void)
{
    WITH_SEMAPHORE(rsem);

    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        update_instance(i);
    }

    // calculate number of instances
    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        if (drivers[i] != nullptr) {
            num_instances = i+1;
        }
    }

#if GPS_MAX_RECEIVERS > 1
#if HAL_LOGGING_ENABLED
    const uint8_t old_primary = primary_instance;
#endif
    update_primary();
#if HAL_LOGGING_ENABLED
    if (primary_instance != old_primary) {
        AP::logger().Write_Event(LogEvent::GPS_PRIMARY_CHANGED);
    }
#endif  // HAL_LOGING_ENABLED
#endif  // GPS_MAX_RECEIVERS > 1

#ifndef HAL_BUILD_AP_PERIPH
    // update notify with gps status. We always base this on the primary_instance
    AP_Notify::flags.gps_status = state[primary_instance].status;
    AP_Notify::flags.gps_num_sats = state[primary_instance].num_sats;
#endif
}

/*
  update primary GPS instance
 */
#if GPS_MAX_RECEIVERS > 1
void AP_GPS::update_primary(void)
{
#if AP_GPS_BLENDED_ENABLED
    /*
      if blending is requested, attempt to calculate weighting for
      each GPS
      we do not do blending if using moving baseline yaw as the rover is
      significant lagged and gives no more information on position or
      velocity
    */
    const bool using_moving_base = is_rtk_base(0) || is_rtk_base(1);
    if ((GPSAutoSwitch)_auto_switch.get() == GPSAutoSwitch::BLEND && !using_moving_base) {
        _output_is_blended = ((AP_GPS_Blended*)drivers[GPS_BLENDED_INSTANCE])->calc_weights();
    } else {
        _output_is_blended = false;
        ((AP_GPS_Blended*)drivers[GPS_BLENDED_INSTANCE])->zero_health_counter();
    }

    if (_output_is_blended) {
        // Use the weighting to calculate blended GPS states
        ((AP_GPS_Blended*)drivers[GPS_BLENDED_INSTANCE])->calc_state();
        // set primary to the virtual instance
        primary_instance = GPS_BLENDED_INSTANCE;
        return;
    }
#endif //   AP_GPS_BLENDED_ENABLED

    // check the primary param is set to possible GPS
    int8_t primary_param = _primary.get();
    if ((primary_param < 0) || (primary_param>=GPS_MAX_RECEIVERS)) {
        primary_param = 0;
    }
    // if primary is not enabled try first instance
    if (get_type(primary_param) == GPS_TYPE_NONE) {
        primary_param = 0;
    }

    if ((GPSAutoSwitch)_auto_switch.get() == GPSAutoSwitch::NONE) {
        // No switching of GPSs, always use the primary instance
        primary_instance = primary_param;
        return;
    }

    uint32_t now = AP_HAL::millis();

    // special handling of RTK moving baseline pair. Always use the
    // base as the rover position is derived from the base, which
    // means the rover always has worse position and velocity than the
    // base. This overrides the normal logic which would select the
    // rover as it typically is in fix type 6 (RTK) whereas base is
    // usually fix type 3
    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        if (is_rtk_base(i) &&
            is_rtk_rover(i^1) &&
            ((state[i].status >= GPS_OK_FIX_3D) || (state[i].status >= state[i^1].status))) {
            if (primary_instance != i) {
                _last_instance_swap_ms = now;
                primary_instance = i;
            }
            // don't do any more switching logic. We don't want the
            // RTK status of the rover to cause a switch
            return;
        }
    }

#if AP_GPS_BLENDED_ENABLED
    // handling switching away from blended GPS
    if (primary_instance == GPS_BLENDED_INSTANCE) {
        primary_instance = 0;
        for (uint8_t i=1; i<GPS_MAX_RECEIVERS; i++) {
            // choose GPS with highest state or higher number of
            // satellites. Reject a GPS with an old update time, as it
            // may be the old timestamp that triggered the loss of
            // blending
            const uint32_t delay_threshold = 400;
            const bool higher_status = state[i].status > state[primary_instance].status;
            const bool old_data_primary = (now - state[primary_instance].last_gps_time_ms) > delay_threshold;
            const bool old_data = (now - state[i].last_gps_time_ms) > delay_threshold;
            const bool equal_status = state[i].status == state[primary_instance].status;
            const bool more_sats = state[i].num_sats > state[primary_instance].num_sats;
            if (old_data && !old_data_primary) {
                // don't switch to a GPS that has not updated in 400ms
                continue;
            }
            if (state[i].status < GPS_OK_FIX_3D) {
                // don't use a GPS without 3D fix
                continue;
            }
            // select the new GPS if the primary has old data, or new
            // GPS either has higher status, or has the same status
            // and more satellites
            if ((old_data_primary && !old_data) ||
                higher_status ||
                (equal_status && more_sats)) {
                primary_instance = i;
            }
        }
        _last_instance_swap_ms = now;
        return;
    }
#endif  // AP_GPS_BLENDED_ENABLED

    // Use primary if 3D fix or better
    if (((GPSAutoSwitch)_auto_switch.get() == GPSAutoSwitch::USE_PRIMARY_IF_3D_FIX) && (state[primary_param].status >= GPS_OK_FIX_3D)) {
        // Primary GPS has a least a 3D fix, switch to it if necessary
        if (primary_instance != primary_param) {
            primary_instance = primary_param;
            _last_instance_swap_ms = now;
        }
        return;
    }

    // handle switch between real GPSs
    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        if (i == primary_instance) {
            continue;
        }
        if (state[i].status > state[primary_instance].status) {
            // we have a higher status lock, or primary is set to the blended GPS, change GPS
            primary_instance = i;
            _last_instance_swap_ms = now;
            continue;
        }

        bool another_gps_has_1_or_more_sats = (state[i].num_sats >= state[primary_instance].num_sats + 1);

        if (state[i].status == state[primary_instance].status && another_gps_has_1_or_more_sats) {

            bool another_gps_has_2_or_more_sats = (state[i].num_sats >= state[primary_instance].num_sats + 2);

            if ((another_gps_has_1_or_more_sats && (now - _last_instance_swap_ms) >= 20000) ||
                (another_gps_has_2_or_more_sats && (now - _last_instance_swap_ms) >= 5000)) {
                // this GPS has more satellites than the
                // current primary, switch primary. Once we switch we will
                // then tend to stick to the new GPS as primary. We don't
                // want to switch too often as it will look like a
                // position shift to the controllers.
                primary_instance = i;
                _last_instance_swap_ms = now;
            }
        }
    }
}
#endif  // GPS_MAX_RECEIVERS > 1

#if HAL_GCS_ENABLED
void AP_GPS::handle_gps_inject(const mavlink_message_t &msg)
{
    mavlink_gps_inject_data_t packet;
    mavlink_msg_gps_inject_data_decode(&msg, &packet);

    if (packet.len > sizeof(packet.data)) {
        // invalid packet
        return;
    }

    handle_gps_rtcm_fragment(0, packet.data, packet.len);
}

/*
  pass along a mavlink message (for MAV type)
 */
void AP_GPS::handle_msg(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_GPS_RTCM_DATA:
        // pass data to de-fragmenter
        handle_gps_rtcm_data(chan, msg);
        break;
    case MAVLINK_MSG_ID_GPS_INJECT_DATA:
        handle_gps_inject(msg);
        break;
    default: {
        uint8_t i;
        for (i=0; i<num_instances; i++) {
            if ((drivers[i] != nullptr) && (params[i].type != GPS_TYPE_NONE)) {
                drivers[i]->handle_msg(msg);
            }
        }
        break;
    }
    }
}
#endif

#if HAL_MSP_GPS_ENABLED
void AP_GPS::handle_msp(const MSP::msp_gps_data_message_t &pkt)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != nullptr && params[i].type == GPS_TYPE_MSP) {
            drivers[i]->handle_msp(pkt);
        }
    }
}
#endif // HAL_MSP_GPS_ENABLED

#if HAL_EXTERNAL_AHRS_ENABLED

bool AP_GPS::get_first_external_instance(uint8_t& instance) const
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != nullptr && params[i].type == GPS_TYPE_EXTERNAL_AHRS) {
            instance = i;
            return true;
        }
    }
    return false;
}

void AP_GPS::handle_external(const AP_ExternalAHRS::gps_data_message_t &pkt, const uint8_t instance)
{
    if (get_type(instance) == GPS_TYPE_EXTERNAL_AHRS && drivers[instance] != nullptr) {
        drivers[instance]->handle_external(pkt);
    }
}
#endif // HAL_EXTERNAL_AHRS_ENABLED

/**
   Lock a GPS port, preventing the GPS driver from using it. This can
   be used to allow a user to control a GPS port via the
   SERIAL_CONTROL protocol
 */
void AP_GPS::lock_port(uint8_t instance, bool lock)
{

    if (instance >= GPS_MAX_RECEIVERS) {
        return;
    }
    if (lock) {
        locked_ports |= (1U<<instance);
    } else {
        locked_ports &= ~(1U<<instance);
    }
}

// Inject a packet of raw binary to a GPS
void AP_GPS::inject_data(const uint8_t *data, uint16_t len)
{
    //Support broadcasting to all GPSes.
    if (_inject_to == GPS_RTK_INJECT_TO_ALL) {
        for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
            if (is_rtk_rover(i)) {
                // we don't externally inject to moving baseline rover
                continue;
            }
            inject_data(i, data, len);
        }
    } else {
        inject_data(_inject_to, data, len);
    }
}

void AP_GPS::inject_data(uint8_t instance, const uint8_t *data, uint16_t len)
{
    if (instance < GPS_MAX_RECEIVERS && drivers[instance] != nullptr) {
        drivers[instance]->inject_data(data, len);
    }
}

/*
  get GPS yaw following mavlink GPS_RAW_INT and GPS2_RAW
  convention. We return 0 if the GPS is not configured to provide
  yaw. We return 65535 for a GPS configured to provide yaw that can't
  currently provide it. We return from 1 to 36000 for yaw otherwise
 */
uint16_t AP_GPS::gps_yaw_cdeg(uint8_t instance) const
{
    if (!have_gps_yaw_configured(instance)) {
        return 0;
    }
    float yaw_deg, accuracy_deg;
    uint32_t time_ms;
    if (!gps_yaw_deg(instance, yaw_deg, accuracy_deg, time_ms)) {
        return 65535;
    }
    int yaw_cd = wrap_360_cd(yaw_deg * 100);
    if (yaw_cd == 0) {
        return 36000;
    }
    return yaw_cd;
}

void AP_GPS::send_mavlink_gps_raw(mavlink_channel_t chan)
{
    if (params[0].type == GPS_TYPE_NONE) {  // If none, do not send telemetry.
        return;
    }

    const Location &loc = location(0);
    float hacc = 0.0f;
    float vacc = 0.0f;
    float sacc = 0.0f;
    float undulation = 0.0;
    int32_t height_elipsoid_mm = 0;
    if (get_undulation(0, undulation)) {
        height_elipsoid_mm = loc.alt*10 - undulation*1000;
    }
    horizontal_accuracy(0, hacc);
    vertical_accuracy(0, vacc);
    speed_accuracy(0, sacc);
    mavlink_msg_gps_raw_int_send(
        chan,
        last_fix_time_ms(0)*(uint64_t)1000,
        status(0),
        loc.lat,        // in 1E7 degrees
        loc.lng,        // in 1E7 degrees
        loc.alt * 10UL, // in mm
        get_hdop(0),
        get_vdop(0),
        ground_speed(0)*100,  // cm/s
        ground_course(0)*100, // 1/100 degrees,
        num_sats(0),
        height_elipsoid_mm,   // Ellipsoid height in mm
        hacc * 1000,          // one-sigma standard deviation in mm
        vacc * 1000,          // one-sigma standard deviation in mm
        sacc * 1000,          // one-sigma standard deviation in mm/s
        0,                    // TODO one-sigma heading accuracy standard deviation
        gps_yaw_cdeg(0));
}

#if GPS_MAX_RECEIVERS > 1
void AP_GPS::send_mavlink_gps2_raw(mavlink_channel_t chan)
{
    // always send the message if 2nd GPS is configured
    if (params[1].type == GPS_TYPE_NONE) {
        return;
    }

    const Location &loc = location(1);
    float hacc = 0.0f;
    float vacc = 0.0f;
    float sacc = 0.0f;
    float undulation = 0.0;
    float height_elipsoid_mm = 0;
    if (get_undulation(1, undulation)) {
        height_elipsoid_mm = loc.alt*10 - undulation*1000;
    }
    horizontal_accuracy(1, hacc);
    vertical_accuracy(1, vacc);
    speed_accuracy(1, sacc);
    mavlink_msg_gps2_raw_send(
        chan,
        last_fix_time_ms(1)*(uint64_t)1000,
        status(1),
        loc.lat,
        loc.lng,
        loc.alt * 10UL,
        get_hdop(1),
        get_vdop(1),
        ground_speed(1)*100,  // cm/s
        ground_course(1)*100, // 1/100 degrees,
        num_sats(1),
        state[1].rtk_num_sats,
        state[1].rtk_age_ms,
        gps_yaw_cdeg(1),
        height_elipsoid_mm,   // Ellipsoid height in mm
        hacc * 1000,          // one-sigma standard deviation in mm
        vacc * 1000,          // one-sigma standard deviation in mm
        sacc * 1000,          // one-sigma standard deviation in mm/s
        0);                    // TODO one-sigma heading accuracy standard deviation
}
#endif // GPS_MAX_RECEIVERS

#if HAL_GCS_ENABLED
void AP_GPS::send_mavlink_gps_rtk(mavlink_channel_t chan, uint8_t inst)
{
    if (inst >= GPS_MAX_RECEIVERS) {
        return;
    }
    if (drivers[inst] != nullptr && drivers[inst]->supports_mavlink_gps_rtk_message()) {
        drivers[inst]->send_mavlink_gps_rtk(chan);
    }
}
#endif

bool AP_GPS::first_unconfigured_gps(uint8_t &instance) const
{
    for (int i = 0; i < GPS_MAX_RECEIVERS; i++) {
        if (params[i].type != GPS_TYPE_NONE && (drivers[i] == nullptr || !drivers[i]->is_configured())) {
            instance = i;
            return true;
        }
    }
    return false;
}

void AP_GPS::broadcast_first_configuration_failure_reason(void) const
{
    uint8_t unconfigured;
    if (first_unconfigured_gps(unconfigured)) {
        if (drivers[unconfigured] == nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GPS %d: was not found", unconfigured + 1);
        } else {
            drivers[unconfigured]->broadcast_configuration_failure_reason();
        }
    }
}

// pre-arm check that all GPSs are close to each other.  farthest distance between GPSs (in meters) is returned
bool AP_GPS::all_consistent(float &distance) const
{
    // return true immediately if only one valid receiver
    if (num_instances <= 1 ||
        drivers[0] == nullptr || params[0].type == GPS_TYPE_NONE) {
        distance = 0;
        return true;
    }

    // calculate distance
    distance = state[0].location.get_distance_NED(state[1].location).length();
    // success if distance is within 50m
    return (distance < 50);
}

/*
   re-assemble fragmented RTCM data
 */
void AP_GPS::handle_gps_rtcm_fragment(uint8_t flags, const uint8_t *data, uint8_t len)
{
    if ((flags & 1) == 0) {
        // it is not fragmented, pass direct
        inject_data(data, len);
        return;
    }

    // see if we need to allocate re-assembly buffer
    if (rtcm_buffer == nullptr) {
        rtcm_buffer = (struct rtcm_buffer *)calloc(1, sizeof(*rtcm_buffer));
        if (rtcm_buffer == nullptr) {
            // nothing to do but discard the data
            return;
        }
    }

    const uint8_t fragment = (flags >> 1U) & 0x03;
    const uint8_t sequence = (flags >> 3U) & 0x1F;
    uint8_t* start_of_fragment_in_buffer = &rtcm_buffer->buffer[MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN * (uint16_t)fragment];
    bool should_clear_previous_fragments = false;

    if (rtcm_buffer->fragments_received) {
        const bool sequence_nr_changed = rtcm_buffer->sequence != sequence;
        const bool seen_this_fragment_index = rtcm_buffer->fragments_received & (1U << fragment);

        // check whether this is a duplicate fragment. If it is, we can
        // return early.
        if (!sequence_nr_changed && seen_this_fragment_index && !memcmp(start_of_fragment_in_buffer, data, len)) {
            return;
        }

        // not a duplicate
        should_clear_previous_fragments = sequence_nr_changed || seen_this_fragment_index;
    }

    if (should_clear_previous_fragments) {
        // we have one or more partial fragments already received
        // which conflict with the new fragment, discard previous fragments
        rtcm_buffer->fragment_count = 0;
        rtcm_stats.fragments_discarded += __builtin_popcount(rtcm_buffer->fragments_received);
        rtcm_buffer->fragments_received = 0;
    }

    // add this fragment
    rtcm_buffer->sequence = sequence;
    rtcm_buffer->fragments_received |= (1U << fragment);

    // copy the data
    memcpy(start_of_fragment_in_buffer, data, len);

    // when we get a fragment of less than max size then we know the
    // number of fragments. Note that this means if you want to send a
    // block of RTCM data of an exact multiple of the buffer size you
    // need to send a final packet of zero length
    if (len < MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN) {
        rtcm_buffer->fragment_count = fragment+1;
        rtcm_buffer->total_length = (MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN*fragment) + len;
    } else if (rtcm_buffer->fragments_received == 0x0F) {
        // special case of 4 full fragments
        rtcm_buffer->fragment_count = 4;
        rtcm_buffer->total_length = MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN*4;
    }


    // see if we have all fragments
    if (rtcm_buffer->fragment_count != 0 &&
        rtcm_buffer->fragments_received == (1U << rtcm_buffer->fragment_count) - 1) {
        // we have them all, inject
        rtcm_stats.fragments_used += __builtin_popcount(rtcm_buffer->fragments_received);
        inject_data(rtcm_buffer->buffer, rtcm_buffer->total_length);
        rtcm_buffer->fragment_count = 0;
        rtcm_buffer->fragments_received = 0;
    }
}

/*
   re-assemble GPS_RTCM_DATA message
 */
void AP_GPS::handle_gps_rtcm_data(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    mavlink_gps_rtcm_data_t packet;
    mavlink_msg_gps_rtcm_data_decode(&msg, &packet);

    if (packet.len > sizeof(packet.data)) {
        // invalid packet
        return;
    }

#if AP_GPS_RTCM_DECODE_ENABLED
    if (!option_set(DriverOptions::DisableRTCMDecode)) {
        const uint16_t mask = (1U << unsigned(chan));
        rtcm.seen_mav_channels |= mask;
        if (option_set(DriverOptions::AlwaysRTCMDecode) ||
            (rtcm.seen_mav_channels & ~mask) != 0) {
            /*
              we are seeing RTCM on multiple mavlink channels. We will run
              the data through a full per-channel RTCM decoder
            */
            if (parse_rtcm_injection(chan, packet)) {
                return;
            }
        }
    }
#endif

    handle_gps_rtcm_fragment(packet.flags, packet.data, packet.len);
}

#if AP_GPS_RTCM_DECODE_ENABLED
/*
  fully parse RTCM data coming in from a MAVLink channel, when we have
  a full message inject it to the GPS. This approach allows for 2 or
  more MAVLink channels to be used for the same RTCM data, allowing
  for redundent transports for maximum reliability at the cost of some
  extra CPU and a bit of re-assembly lag
 */
bool AP_GPS::parse_rtcm_injection(mavlink_channel_t chan, const mavlink_gps_rtcm_data_t &pkt)
{
    if (chan >= MAVLINK_COMM_NUM_BUFFERS) {
        return false;
    }
    if (rtcm.parsers[chan] == nullptr) {
        rtcm.parsers[chan] = NEW_NOTHROW RTCM3_Parser();
        if (rtcm.parsers[chan] == nullptr) {
            return false;
        }
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GPS: RTCM parsing for chan %u", unsigned(chan));
    }
    for (uint16_t i=0; i<pkt.len; i++) {
        if (rtcm.parsers[chan]->read(pkt.data[i])) {
            // we have a full message, inject it
            const uint8_t *buf = nullptr;
            uint16_t len = rtcm.parsers[chan]->get_len(buf);

            // see if we have already sent it. This prevents
            // duplicates from multiple sources
            const uint32_t crc = crc_crc32(0, buf, len);

#if HAL_LOGGING_ENABLED
            AP::logger().WriteStreaming("RTCM", "TimeUS,Chan,RTCMId,Len,CRC", "s#---", "F----", "QBHHI",
                                        AP_HAL::micros64(),
                                        uint8_t(chan),
                                        rtcm.parsers[chan]->get_id(),
                                        len,
                                        crc);
#endif
            
            bool already_seen = false;
            for (uint8_t c=0; c<ARRAY_SIZE(rtcm.sent_crc); c++) {
                if (rtcm.sent_crc[c] == crc) {
                    // we have already sent this message
                    already_seen = true;
                    break;
                }
            }
            if (already_seen) {
                continue;
            }
            rtcm.sent_crc[rtcm.sent_idx] = crc;
            rtcm.sent_idx = (rtcm.sent_idx+1) % ARRAY_SIZE(rtcm.sent_crc);

            if (buf != nullptr && len > 0) {
                inject_data(buf, len);
            }
            rtcm.parsers[chan]->reset();
        }
    }
    return true;
}
#endif // AP_GPS_RTCM_DECODE_ENABLED

#if HAL_LOGGING_ENABLED
void AP_GPS::Write_AP_Logger_Log_Startup_messages()
{
    for (uint8_t instance=0; instance<num_instances; instance++) {
        if (drivers[instance] == nullptr || state[instance].status == NO_GPS) {
            continue;
        }
        drivers[instance]->Write_AP_Logger_Log_Startup_messages();
    }
}
#endif

/*
  return the expected lag (in seconds) in the position and velocity readings from the gps
  return true if the GPS hardware configuration is known or the delay parameter has been set
 */
bool AP_GPS::get_lag(uint8_t instance, float &lag_sec) const
{
    // always ensure a lag is provided
    lag_sec = 0.1f;

    if (instance >= GPS_MAX_INSTANCES) {
        return false;
    }

#if AP_GPS_BLENDED_ENABLED
    // return lag of blended GPS
    if (instance == GPS_BLENDED_INSTANCE) {
        return drivers[instance]->get_lag(lag_sec);
    }
#endif

    if (params[instance].delay_ms > 0) {
        // if the user has specified a non zero time delay, always return that value
        lag_sec = 0.001f * (float)params[instance].delay_ms;
        // the user is always right !!
        return true;
    } else if (drivers[instance] == nullptr || state[instance].status == NO_GPS) {
        // no GPS was detected in this instance so return the worst possible lag term
        const auto type = params[instance].type;
        if (type == GPS_TYPE_NONE) {
            lag_sec = 0.0f;
            return true;
        }
        return type == GPS_TYPE_AUTO;
    } else {
        // the user has not specified a delay so we determine it from the GPS type
        return drivers[instance]->get_lag(lag_sec);
    }
}

// return a 3D vector defining the offset of the GPS antenna in meters relative to the body frame origin
const Vector3f &AP_GPS::get_antenna_offset(uint8_t instance) const
{
    if (instance >= GPS_MAX_INSTANCES) {
        // we have to return a reference so use instance 0
        return params[0].antenna_offset;
    }

#if AP_GPS_BLENDED_ENABLED
    if (instance == GPS_BLENDED_INSTANCE) {
        // return an offset for the blended GPS solution
        return ((AP_GPS_Blended*)drivers[instance])->get_antenna_offset();
    }
#endif

    return params[instance].antenna_offset;
}

/*
  returns the desired gps update rate in milliseconds
  this does not provide any guarantee that the GPS is updating at the requested
  rate it is simply a helper for use in the backends for determining what rate
  they should be configuring the GPS to run at
*/
uint16_t AP_GPS::get_rate_ms(uint8_t instance) const
{
    // sanity check
    if (instance >= num_instances || params[instance].rate_ms <= 0) {
        return GPS_MAX_RATE_MS;
    }
    return MIN(params[instance].rate_ms, GPS_MAX_RATE_MS);
}

bool AP_GPS::is_healthy(uint8_t instance) const
{
    if (instance >= GPS_MAX_INSTANCES) {
        return false;
    }

    if (get_type(_primary.get()) == GPS_TYPE_NONE) {
        return false;
    }

#ifndef HAL_BUILD_AP_PERIPH
    /*
      on AP_Periph handling of timing is done by the flight controller
      receiving the DroneCAN messages
     */
    /*
      allow two lost frames before declaring the GPS unhealthy, but
      require the average frame rate to be close to 5Hz.

      We allow for a rate of 3Hz average for a moving baseline rover
      due to the packet loss that happens with the RTCMv3 data and the
      fact that the rate of yaw data is not critical
     */
    const uint8_t delay_threshold = 2;
    const float delay_avg_max = is_rtk_rover(instance) ? 333 : 215;
    const GPS_timing &t = timing[instance];
    bool delay_ok = (t.delayed_count < delay_threshold) &&
        t.average_delta_ms < delay_avg_max &&
        state[instance].lagged_sample_count < 5;
    if (!delay_ok) {
        return false;
    }
#endif // HAL_BUILD_AP_PERIPH

    return drivers[instance] != nullptr &&
           drivers[instance]->is_healthy();
}

bool AP_GPS::prepare_for_arming(void) {
    bool all_passed = true;
    for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
        if (drivers[i] != nullptr) {
            all_passed &= drivers[i]->prepare_for_arming();
        }
    }
    return all_passed;
}

bool AP_GPS::pre_arm_checks(char failure_msg[], uint16_t failure_msg_len)
{
    // the DroneCAN class has additional checks for DroneCAN-specific
    // parameters:
#if AP_GPS_DRONECAN_ENABLED
    if (!AP_GPS_DroneCAN::inter_instance_pre_arm_checks(failure_msg, failure_msg_len)) {
        return false;
    }
#endif

    for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
        if (is_rtk_rover(i)) {
            if (AP_HAL::millis() - state[i].gps_yaw_time_ms > 15000) {
                hal.util->snprintf(failure_msg, failure_msg_len, "GPS[%u] yaw not available", unsigned(i+1));
                return false;
            }
        }
    }

#if AP_GPS_BLENDED_ENABLED
    if (!drivers[GPS_BLENDED_INSTANCE]->is_healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "GPS blending unhealthy");
        return false;
    }
#endif

    return true;
}

bool AP_GPS::logging_failed(void) const {
    if (!logging_enabled()) {
        return false;
    }

    for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
        if ((drivers[i] != nullptr) && !(drivers[i]->logging_healthy())) {
            return true;
        }
    }

    return false;
}

// get iTOW, if supported, zero otherwie
uint32_t AP_GPS::get_itow(uint8_t instance) const
{
    if (instance >= GPS_MAX_RECEIVERS || drivers[instance] == nullptr) {
        return 0;
    }
    return drivers[instance]->get_last_itow_ms();
}

bool AP_GPS::get_error_codes(uint8_t instance, uint32_t &error_codes) const
{
    if (instance >= GPS_MAX_RECEIVERS || drivers[instance] == nullptr) {
        return false;
    }

    return drivers[instance]->get_error_codes(error_codes);
}

// get the difference between WGS84 and AMSL. A positive value means
// the AMSL height is higher than WGS84 ellipsoid height
bool AP_GPS::get_undulation(uint8_t instance, float &undulation) const
{
    if (!state[instance].have_undulation) {
        return false;
    }
    undulation = state[instance].undulation;
    return true;
}

#if HAL_LOGGING_ENABLED
// Logging support:
// Write an GPS packet
void AP_GPS::Write_GPS(uint8_t i)
{
    const uint64_t time_us = AP_HAL::micros64();
    const Location &loc = location(i);

    float yaw_deg=0, yaw_accuracy_deg=0;
    uint32_t yaw_time_ms;
    gps_yaw_deg(i, yaw_deg, yaw_accuracy_deg, yaw_time_ms);

    const struct log_GPS pkt {
        LOG_PACKET_HEADER_INIT(LOG_GPS_MSG),
        time_us       : time_us,
        instance      : i,
        status        : (uint8_t)status(i),
        gps_week_ms   : time_week_ms(i),
        gps_week      : time_week(i),
        num_sats      : num_sats(i),
        hdop          : get_hdop(i),
        latitude      : loc.lat,
        longitude     : loc.lng,
        altitude      : loc.alt,
        ground_speed  : ground_speed(i),
        ground_course : ground_course(i),
        vel_z         : velocity(i).z,
        yaw           : yaw_deg,
        used          : (uint8_t)(AP::gps().primary_sensor() == i)
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));

    /* write auxiliary accuracy information as well */
    float hacc = 0, vacc = 0, sacc = 0;
    float undulation = 0;
    horizontal_accuracy(i, hacc);
    vertical_accuracy(i, vacc);
    speed_accuracy(i, sacc);
    get_undulation(i, undulation);
    struct log_GPA pkt2{
        LOG_PACKET_HEADER_INIT(LOG_GPA_MSG),
        time_us       : time_us,
        instance      : i,
        vdop          : get_vdop(i),
        hacc          : (uint16_t)MIN((hacc*100), UINT16_MAX),
        vacc          : (uint16_t)MIN((vacc*100), UINT16_MAX),
        sacc          : (uint16_t)MIN((sacc*100), UINT16_MAX),
        yaw_accuracy  : yaw_accuracy_deg,
        have_vv       : (uint8_t)have_vertical_velocity(i),
        sample_ms     : last_message_time_ms(i),
        delta_ms      : last_message_delta_time_ms(i),
        undulation    : undulation,
        rtcm_fragments_used: rtcm_stats.fragments_used,
        rtcm_fragments_discarded: rtcm_stats.fragments_discarded
    };
    AP::logger().WriteBlock(&pkt2, sizeof(pkt2));
}
#endif

bool AP_GPS::is_rtk_base(uint8_t instance) const
{
    switch (get_type(instance)) {
    case GPS_TYPE_UBLOX_RTK_BASE:
    case GPS_TYPE_UAVCAN_RTK_BASE:
        return true;
    default:
        break;
    }
    return false;
}

bool AP_GPS::is_rtk_rover(uint8_t instance) const
{
    switch (get_type(instance)) {
    case GPS_TYPE_UBLOX_RTK_ROVER:
    case GPS_TYPE_UAVCAN_RTK_ROVER:
        return true;
    default:
        break;
    }
    return false;
}

/*
  get GPS based yaw
 */
bool AP_GPS::gps_yaw_deg(uint8_t instance, float &yaw_deg, float &accuracy_deg, uint32_t &time_ms) const
{
#if GPS_MAX_RECEIVERS > 1
    if (is_rtk_base(instance) && is_rtk_rover(instance^1)) {
        // return the yaw from the rover
        instance ^= 1;
    }
#endif
    if (!have_gps_yaw(instance)) {
        return false;
    }
    yaw_deg = state[instance].gps_yaw;

    // get lagged timestamp
    time_ms = state[instance].gps_yaw_time_ms;
    float lag_s;
    if (get_lag(instance, lag_s)) {
        uint32_t lag_ms = lag_s * 1000;
        time_ms -= lag_ms;
    }

    if (state[instance].have_gps_yaw_accuracy) {
        accuracy_deg = state[instance].gps_yaw_accuracy;
    } else {
        // fall back to 10 degrees as a generic default
        accuracy_deg = 10;
    }
    return true;
}

/*
 * Old parameter metadata.  Until we have versioned parameters, keeping
 * old parameters around for a while can help with an adjustment
 * period.
 */

    // @Param: _TYPE
    // @DisplayName: 1st GPS type
    // @Description: GPS type of 1st GPS.Renamed in 4.6 and later to GPS1_TYPE
    // @Values: 0:None,1:AUTO,2:uBlox,5:NMEA,6:SiRF,7:HIL,8:SwiftNav,9:DroneCAN,10:SBF,11:GSOF,13:ERB,14:MAV,15:NOVA,16:HemisphereNMEA,17:uBlox-MovingBaseline-Base,18:uBlox-MovingBaseline-Rover,19:MSP,20:AllyStar,21:ExternalAHRS,22:DroneCAN-MovingBaseline-Base,23:DroneCAN-MovingBaseline-Rover,24:UnicoreNMEA,25:UnicoreMovingBaselineNMEA,26:SBF-DualAntenna
    // @RebootRequired: True
    // @User: Advanced
    // @Legacy: only included here so GCSs running stable can get the description.  Omitted in the Wiki.

    // @Param: _TYPE2
    // @CopyFieldsFrom: GPS_TYPE
    // @DisplayName: 2nd GPS type.Renamed in 4.6 to GPS2_TYPE
    // @Description: GPS type of 2nd GPS
    // @Legacy: 4.5 param

    // @Param: _GNSS_MODE
    // @DisplayName: GNSS system configuration
    // @Description: Bitmask for what GNSS system to use on the first GPS (all unchecked or zero to leave GPS as configured).Renamed in 4.6 and later to GPS1_GNSS_MODE.
    // @Legacy: 4.5 param
    // @Bitmask: 0:GPS,1:SBAS,2:Galileo,3:Beidou,4:IMES,5:QZSS,6:GLONASS
    // @User: Advanced

    // @Param: _GNSS_MODE2
    // @DisplayName: GNSS system configuration.
    // @Description: Bitmask for what GNSS system to use on the second GPS (all unchecked or zero to leave GPS as configured). Renamed in 4.6 and later to GPS2_GNSS_MODE
    // @Legacy: 4.5 param
    // @Bitmask: 0:GPS,1:SBAS,2:Galileo,3:Beidou,4:IMES,5:QZSS,6:GLONASS
    // @User: Advanced

    // @Param: _RATE_MS
    // @DisplayName: GPS update rate in milliseconds
    // @Description: Controls how often the GPS should provide a position update. Lowering below 5Hz(default) is not allowed. Raising the rate above 5Hz usually provides little benefit and for some GPS (eg Ublox M9N) can severely impact performance.Renamed in 4.6 and later to GPS1_RATE_MS
    // @Legacy: 4.5 param
    // @Units: ms
    // @Values: 100:10Hz,125:8Hz,200:5Hz
    // @Range: 50 200
    // @User: Advanced

    // @Param: _RATE_MS2
    // @DisplayName: GPS 2 update rate in milliseconds
    // @Description: Controls how often the GPS should provide a position update. Lowering below 5Hz(default) is not allowed. Raising the rate above 5Hz usually provides little benefit and for some GPS (eg Ublox M9N) can severely impact performance.Renamed in 4.6 and later to GPS2_RATE_MS
    // @Legacy: 4.5 param
    // @Units: ms
    // @Values: 100:10Hz,125:8Hz,200:5Hz
    // @Range: 50 200
    // @User: Advanced

    // @Param: _POS1_X
    // @DisplayName: Antenna X position offset
    // @Description: X position of the first GPS antenna in body frame. Positive X is forward of the origin. Use antenna phase centroid location if provided by the manufacturer.Renamed in 4.6 and later to GPS1_POS_X.
    // @Legacy: 4.5 param
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _POS1_Y
    // @DisplayName: Antenna Y position offset
    // @Description: Y position of the first GPS antenna in body frame. Positive Y is to the right of the origin. Use antenna phase centroid location if provided by the manufacturer.Renamed in 4.6 and later to GPS1_POS_Y.
    // @Legacy: 4.5 param
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _POS1_Z
    // @DisplayName: Antenna Z position offset
    // @Description: Z position of the first GPS antenna in body frame. Positive Z is down from the origin. Use antenna phase centroid location if provided by the manufacturer.Renamed in 4.6 and later to GPS1_POS_Z.
    // @Legacy: 4.5 param
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _POS2_X
    // @DisplayName: Antenna X position offset
    // @Description: X position of the second GPS antenna in body frame. Positive X is forward of the origin. Use antenna phase centroid location if provided by the manufacturer.Renamed in 4.6 and later to GPS2_POS_X.
    // @Legacy: 4.5 param
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _POS2_Y
    // @DisplayName: Antenna Y position offset
    // @Description: Y position of the second GPS antenna in body frame. Positive Y is to the right of the origin. Use antenna phase centroid location if provided by the manufacturer.Renamed in 4.6 and later to GPS2_POS_Y.
    // @Legacy: 4.5 param
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _POS2_Z
    // @DisplayName: Antenna Z position offset
    // @Description: Z position of the second GPS antenna in body frame. Positive Z is down from the origin. Use antenna phase centroid location if provided by the manufacturer.Renamed in 4.6 and later to GPS2_POS_Z.
    // @Legacy: 4.5 param
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _DELAY_MS
    // @DisplayName: GPS delay in milliseconds
    // @Description: Controls the amount of GPS  measurement delay that the autopilot compensates for. Set to zero to use the default delay for the detected GPS type.Renamed in 4.6 and later to GPS1_DELAY_MS.
    // @Legacy: 4.5 param
    // @Units: ms
    // @Range: 0 250
    // @User: Advanced
    // @RebootRequired: True

    // @Param: _DELAY_MS2
    // @DisplayName: GPS 2 delay in milliseconds
    // @Description: Controls the amount of GPS  measurement delay that the autopilot compensates for. Set to zero to use the default delay for the detected GPS type.Renamed in 4.6 and later to GPS2_DELAY_MS.
    // @Legacy: 4.5 param
    // @Units: ms
    // @Range: 0 250
    // @User: Advanced
    // @RebootRequired: True

    // @Param: _COM_PORT
    // @DisplayName: GPS physical COM port
    // @Description: The physical COM port on the connected device, currently only applies to SBF and GSOF GPS,Renamed in 4.6 and later to GPS1_COM_PORT.
    // @Legacy: 4.5 param
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    // @Values: 0:COM1(RS232) on GSOF, 1:COM2(TTL) on GSOF
    // @RebootRequired: True

    // @Param: _COM_PORT2
    // @DisplayName: GPS physical COM port
    // @Description: The physical COM port on the connected device, currently only applies to SBF and GSOF GPS.Renamed in 4.6 and later to GPS1_COM_PORT.
    // @Legacy: 4.5 param
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True

    // @Group: _MB1_
    // @Path: MovingBase.cpp
    // @Legacy: 4.5 param

    // @Group: _MB2_
    // @Path: MovingBase.cpp
    // @Legacy: 4.5 param

    // @Param: _CAN_NODEID1
    // @DisplayName: GPS Node ID 1
    // @Description: GPS Node id for first-discovered GPS.Renamed in 4.6 and later to GPS1_CAN_NODEID.
    // @Legacy: 4.5 param
    // @ReadOnly: True
    // @User: Advanced

    // @Param: _CAN_NODEID2
    // @DisplayName: GPS Node ID 2
    // @Description: GPS Node id for second-discovered GPS.Renamed in 4.6 and later to GPS2_CAN_NODEID.
    // @Legacy: 4.5 param
    // @ReadOnly: True
    // @User: Advanced

/*
 * end old parameter metadata
 */

namespace AP {

AP_GPS &gps()
{
    return *AP_GPS::get_singleton();
}

};

#endif  // AP_GPS_ENABLED
