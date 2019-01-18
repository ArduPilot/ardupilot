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
#include "AP_GPS.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Notify/AP_Notify.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <climits>

#include "AP_GPS_NOVA.h"
#include "AP_GPS_ERB.h"
#include "AP_GPS_GSOF.h"
#include "AP_GPS_MTK.h"
#include "AP_GPS_MTK19.h"
#include "AP_GPS_NMEA.h"
#include "AP_GPS_SBF.h"
#include "AP_GPS_SBP.h"
#include "AP_GPS_SBP2.h"
#include "AP_GPS_SIRF.h"
#include "AP_GPS_UBLOX.h"
#include "AP_GPS_MAV.h"
#include "GPS_Backend.h"

#if HAL_WITH_UAVCAN
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <AP_UAVCAN/AP_UAVCAN.h>
#include "AP_GPS_UAVCAN.h"
#endif

#define GPS_BAUD_TIME_MS 1200
#define GPS_TIMEOUT_MS 4000u

// defines used to specify the mask position for use of different accuracy metrics in the blending algorithm
#define BLEND_MASK_USE_HPOS_ACC     1
#define BLEND_MASK_USE_VPOS_ACC     2
#define BLEND_MASK_USE_SPD_ACC      4
#define BLEND_COUNTER_FAILURE_INCREMENT 10

extern const AP_HAL::HAL &hal;

// baudrates to try to detect GPSes with
const uint32_t AP_GPS::_baudrates[] = {9600U, 115200U, 4800U, 19200U, 38400U, 57600U, 230400U};

// initialisation blobs to send to the GPS to try to get it into the
// right mode
const char AP_GPS::_initialisation_blob[] = UBLOX_SET_BINARY MTK_SET_BINARY SIRF_SET_BINARY;

AP_GPS *AP_GPS::_singleton;

// table of user settable parameters
const AP_Param::GroupInfo AP_GPS::var_info[] = {
    // @Param: TYPE
    // @DisplayName: GPS type
    // @Description: GPS type
    // @Values: 0:None,1:AUTO,2:uBlox,3:MTK,4:MTK19,5:NMEA,6:SiRF,7:HIL,8:SwiftNav,9:UAVCAN,10:SBF,11:GSOF,13:ERB,14:MAV,15:NOVA
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("TYPE",    0, AP_GPS, _type[0], HAL_GPS_TYPE_DEFAULT),

    // @Param: TYPE2
    // @DisplayName: 2nd GPS type
    // @Description: GPS type of 2nd GPS
    // @Values: 0:None,1:AUTO,2:uBlox,3:MTK,4:MTK19,5:NMEA,6:SiRF,7:HIL,8:SwiftNav,9:UAVCAN,10:SBF,11:GSOF,13:ERB,14:MAV,15:NOVA
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("TYPE2",   1, AP_GPS, _type[1], 0),

    // @Param: NAVFILTER
    // @DisplayName: Navigation filter setting
    // @Description: Navigation filter engine setting
    // @Values: 0:Portable,2:Stationary,3:Pedestrian,4:Automotive,5:Sea,6:Airborne1G,7:Airborne2G,8:Airborne4G
    // @User: Advanced
    AP_GROUPINFO("NAVFILTER", 2, AP_GPS, _navfilter, GPS_ENGINE_AIRBORNE_4G),

    // @Param: AUTO_SWITCH
    // @DisplayName: Automatic Switchover Setting
    // @Description: Automatic switchover to GPS reporting best lock
    // @Values: 0:Disabled,1:UseBest,2:Blend,3:UseSecond
    // @User: Advanced
    AP_GROUPINFO("AUTO_SWITCH", 3, AP_GPS, _auto_switch, 1),

    // @Param: MIN_DGPS
    // @DisplayName: Minimum Lock Type Accepted for DGPS
    // @Description: Sets the minimum type of differential GPS corrections required before allowing to switch into DGPS mode.
    // @Values: 0:Any,50:FloatRTK,100:IntegerRTK
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("MIN_DGPS", 4, AP_GPS, _min_dgps, 100),

    // @Param: SBAS_MODE
    // @DisplayName: SBAS Mode
    // @Description: This sets the SBAS (satellite based augmentation system) mode if available on this GPS. If set to 2 then the SBAS mode is not changed in the GPS. Otherwise the GPS will be reconfigured to enable/disable SBAS. Disabling SBAS may be worthwhile in some parts of the world where an SBAS signal is available but the baseline is too long to be useful.
    // @Values: 0:Disabled,1:Enabled,2:NoChange
    // @User: Advanced
    AP_GROUPINFO("SBAS_MODE", 5, AP_GPS, _sbas_mode, 2),

    // @Param: MIN_ELEV
    // @DisplayName: Minimum elevation
    // @Description: This sets the minimum elevation of satellites above the horizon for them to be used for navigation. Setting this to -100 leaves the minimum elevation set to the GPS modules default.
    // @Range: -100 90
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("MIN_ELEV", 6, AP_GPS, _min_elevation, -100),

    // @Param: INJECT_TO
    // @DisplayName: Destination for GPS_INJECT_DATA MAVLink packets
    // @Description: The GGS can send raw serial packets to inject data to multiple GPSes.
    // @Values: 0:send to first GPS,1:send to 2nd GPS,127:send to all
    // @User: Advanced
    AP_GROUPINFO("INJECT_TO",   7, AP_GPS, _inject_to, GPS_RTK_INJECT_TO_ALL),

    // @Param: SBP_LOGMASK
    // @DisplayName: Swift Binary Protocol Logging Mask
    // @Description: Masked with the SBP msg_type field to determine whether SBR1/SBR2 data is logged
    // @Values: 0:None (0x0000),-1:All (0xFFFF),-256:External only (0xFF00)
    // @User: Advanced
    AP_GROUPINFO("SBP_LOGMASK", 8, AP_GPS, _sbp_logmask, 0xFF00),

    // @Param: RAW_DATA
    // @DisplayName: Raw data logging
    // @Description: Handles logging raw data; on uBlox chips that support raw data this will log RXM messages into dataflash log; on Septentrio this will log on the equipment's SD card and when set to 2, the autopilot will try to stop logging after disarming and restart after arming
    // @Values: 0:Ignore,1:Always log,2:Stop logging when disarmed (SBF only),5:Only log every five samples (uBlox only)
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("RAW_DATA", 9, AP_GPS, _raw_data, 0),

    // @Param: GNSS_MODE
    // @DisplayName: GNSS system configuration
    // @Description: Bitmask for what GNSS system to use on the first GPS (all unchecked or zero to leave GPS as configured)
    // @Values: 0:Leave as currently configured, 1:GPS-NoSBAS, 3:GPS+SBAS, 4:Galileo-NoSBAS, 6:Galileo+SBAS, 8:Beidou, 51:GPS+IMES+QZSS+SBAS (Japan Only), 64:GLONASS, 66:GLONASS+SBAS, 67:GPS+GLONASS+SBAS
    // @Bitmask: 0:GPS,1:SBAS,2:Galileo,3:Beidou,4:IMES,5:QZSS,6:GLOSNASS
    // @User: Advanced
    AP_GROUPINFO("GNSS_MODE", 10, AP_GPS, _gnss_mode[0], 0),

    // @Param: SAVE_CFG
    // @DisplayName: Save GPS configuration
    // @Description: Determines whether the configuration for this GPS should be written to non-volatile memory on the GPS. Currently working for UBlox 6 series and above.
    // @Values: 0:Do not save config,1:Save config,2:Save only when needed
    // @User: Advanced
    AP_GROUPINFO("SAVE_CFG", 11, AP_GPS, _save_config, 2),

    // @Param: GNSS_MODE2
    // @DisplayName: GNSS system configuration
    // @Description: Bitmask for what GNSS system to use on the second GPS (all unchecked or zero to leave GPS as configured)
    // @Values: 0:Leave as currently configured, 1:GPS-NoSBAS, 3:GPS+SBAS, 4:Galileo-NoSBAS, 6:Galileo+SBAS, 8:Beidou, 51:GPS+IMES+QZSS+SBAS (Japan Only), 64:GLONASS, 66:GLONASS+SBAS, 67:GPS+GLONASS+SBAS
    // @Bitmask: 0:GPS,1:SBAS,2:Galileo,3:Beidou,4:IMES,5:QZSS,6:GLOSNASS
    // @User: Advanced
    AP_GROUPINFO("GNSS_MODE2", 12, AP_GPS, _gnss_mode[1], 0),

    // @Param: AUTO_CONFIG
    // @DisplayName: Automatic GPS configuration
    // @Description: Controls if the autopilot should automatically configure the GPS based on the parameters and default settings
    // @Values: 0:Disables automatic configuration,1:Enable automatic configuration
    // @User: Advanced
    AP_GROUPINFO("AUTO_CONFIG", 13, AP_GPS, _auto_config, 1),

    // @Param: RATE_MS
    // @DisplayName: GPS update rate in milliseconds
    // @Description: Controls how often the GPS should provide a position update. Lowering below 5Hz is not allowed
    // @Units: ms
    // @Values: 100:10Hz,125:8Hz,200:5Hz
    // @Range: 50 200
    // @User: Advanced
    AP_GROUPINFO("RATE_MS", 14, AP_GPS, _rate_ms[0], 200),

    // @Param: RATE_MS2
    // @DisplayName: GPS 2 update rate in milliseconds
    // @Description: Controls how often the GPS should provide a position update. Lowering below 5Hz is not allowed
    // @Units: ms
    // @Values: 100:10Hz,125:8Hz,200:5Hz
    // @Range: 50 200
    // @User: Advanced
    AP_GROUPINFO("RATE_MS2", 15, AP_GPS, _rate_ms[1], 200),

    // @Param: POS1_X
    // @DisplayName: Antenna X position offset
    // @Description: X position of the first GPS antenna in body frame. Positive X is forward of the origin. Use antenna phase centroid location if provided by the manufacturer.
    // @Units: m
    // @Range: -10 10
    // @User: Advanced

    // @Param: POS1_Y
    // @DisplayName: Antenna Y position offset
    // @Description: Y position of the first GPS antenna in body frame. Positive Y is to the right of the origin. Use antenna phase centroid location if provided by the manufacturer.
    // @Units: m
    // @Range: -10 10
    // @User: Advanced

    // @Param: POS1_Z
    // @DisplayName: Antenna Z position offset
    // @Description: Z position of the first GPS antenna in body frame. Positive Z is down from the origin. Use antenna phase centroid location if provided by the manufacturer.
    // @Units: m
    // @Range: -10 10
    // @User: Advanced
    AP_GROUPINFO("POS1", 16, AP_GPS, _antenna_offset[0], 0.0f),

    // @Param: POS2_X
    // @DisplayName: Antenna X position offset
    // @Description: X position of the second GPS antenna in body frame. Positive X is forward of the origin. Use antenna phase centroid location if provided by the manufacturer.
    // @Units: m
    // @Range: -10 10
    // @User: Advanced

    // @Param: POS2_Y
    // @DisplayName: Antenna Y position offset
    // @Description: Y position of the second GPS antenna in body frame. Positive Y is to the right of the origin. Use antenna phase centroid location if provided by the manufacturer.
    // @Units: m
    // @Range: -10 10
    // @User: Advanced

    // @Param: POS2_Z
    // @DisplayName: Antenna Z position offset
    // @Description: Z position of the second GPS antenna in body frame. Positive Z is down from the origin. Use antenna phase centroid location if provided by the manufacturer.
    // @Units: m
    // @Range: -10 10
    // @User: Advanced
    AP_GROUPINFO("POS2", 17, AP_GPS, _antenna_offset[1], 0.0f),

    // @Param: DELAY_MS
    // @DisplayName: GPS delay in milliseconds
    // @Description: Controls the amount of GPS  measurement delay that the autopilot compensates for. Set to zero to use the default delay for the detected GPS type.
    // @Units: ms
    // @Range: 0 250
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("DELAY_MS", 18, AP_GPS, _delay_ms[0], 0),

    // @Param: DELAY_MS2
    // @DisplayName: GPS 2 delay in milliseconds
    // @Description: Controls the amount of GPS  measurement delay that the autopilot compensates for. Set to zero to use the default delay for the detected GPS type.
    // @Units: ms
    // @Range: 0 250
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("DELAY_MS2", 19, AP_GPS, _delay_ms[1], 0),

    // @Param: BLEND_MASK
    // @DisplayName: Multi GPS Blending Mask
    // @Description: Determines which of the accuracy measures Horizontal position, Vertical Position and Speed are used to calculate the weighting on each GPS receiver when soft switching has been selected by setting GPS_AUTO_SWITCH to 2
    // @Bitmask: 0:Horiz Pos,1:Vert Pos,2:Speed
    // @User: Advanced
    AP_GROUPINFO("BLEND_MASK", 20, AP_GPS, _blend_mask, 5),

    // @Param: BLEND_TC
    // @DisplayName: Blending time constant
    // @Description: Controls the slowest time constant applied to the calculation of GPS position and height offsets used to adjust different GPS receivers for steady state position differences.
    // @Units: s
    // @Range: 5.0 30.0
    // @User: Advanced
    AP_GROUPINFO("BLEND_TC", 21, AP_GPS, _blend_tc, 10.0f),

    AP_GROUPEND
};

// constructor
AP_GPS::AP_GPS()
{
    static_assert((sizeof(_initialisation_blob) * (CHAR_BIT + 2)) < (4800 * GPS_BAUD_TIME_MS * 1e-3),
                    "GPS initilisation blob is to large to be completely sent before the baud rate changes");

    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_GPS must be singleton");
    }
    _singleton = this;
}

/// Startup initialisation.
void AP_GPS::init(const AP_SerialManager& serial_manager)
{
    primary_instance = 0;

    // search for serial ports with gps protocol
    _port[0] = serial_manager.find_serial(AP_SerialManager::SerialProtocol_GPS, 0);
    _port[1] = serial_manager.find_serial(AP_SerialManager::SerialProtocol_GPS, 1);
    _last_instance_swap_ms = 0;

    // Initialise class variables used to do GPS blending
    _omega_lpf = 1.0f / constrain_float(_blend_tc, 5.0f, 30.0f);

    // prep the state instance fields
    for (uint8_t i = 0; i < GPS_MAX_INSTANCES; i++) {
        state[i].instance = i;
    }

    // sanity check update rate
    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        if (_rate_ms[i] <= 0 || _rate_ms[i] > GPS_MAX_RATE_MS) {
            _rate_ms[i] = GPS_MAX_RATE_MS;
        }
    }
}

// return number of active GPS sensors. Note that if the first GPS
// is not present but the 2nd is then we return 2. Note that a blended
// GPS solution is treated as an additional sensor.
uint8_t AP_GPS::num_sensors(void) const
{
    if (!_output_is_blended) {
        return num_instances;
    } else {
        return num_instances+1;
    }
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


/**
   convert GPS week and milliseconds to unix epoch in milliseconds
 */
uint64_t AP_GPS::time_epoch_convert(uint16_t gps_week, uint32_t gps_ms)
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
    if (istate.last_gps_time_ms == 0) {
        return 0;
    }
    uint64_t fix_time_ms = time_epoch_convert(istate.time_week, istate.time_week_ms);
    // add in the milliseconds since the last fix
    return (fix_time_ms + (AP_HAL::millis() - istate.last_gps_time_ms)) * 1000ULL;
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
  send some more initialisation string bytes if there is room in the
  UART transmit buffer
 */
void AP_GPS::send_blob_update(uint8_t instance)
{
    // exit immediately if no uart for this instance
    if (_port[instance] == nullptr) {
        return;
    }

    // see if we can write some more of the initialisation blob
    if (initblob_state[instance].remaining > 0) {
        int16_t space = _port[instance]->txspace();
        if (space > (int16_t)initblob_state[instance].remaining) {
            space = initblob_state[instance].remaining;
        }
        while (space > 0) {
            _port[instance]->write(*initblob_state[instance].blob);
            initblob_state[instance].blob++;
            space--;
            initblob_state[instance].remaining--;
        }
    }
}

/*
  run detection step for one GPS instance. If this finds a GPS then it
  will fill in drivers[instance] and change state[instance].status
  from NO_GPS to NO_FIX.
 */
void AP_GPS::detect_instance(uint8_t instance)
{
    AP_GPS_Backend *new_gps = nullptr;
    struct detect_state *dstate = &detect_state[instance];
    const uint32_t now = AP_HAL::millis();

    state[instance].status = NO_GPS;
    state[instance].hdop = GPS_UNKNOWN_DOP;
    state[instance].vdop = GPS_UNKNOWN_DOP;

    switch (_type[instance]) {
    // user has to explicitly set the MAV type, do not use AUTO
    // do not try to detect the MAV type, assume it's there
    case GPS_TYPE_MAV:
        dstate->auto_detected_baud = false; // specified, not detected
        new_gps = new AP_GPS_MAV(*this, state[instance], nullptr);
        goto found_gps;
        break;

    // user has to explicitly set the UAVCAN type, do not use AUTO
    case GPS_TYPE_UAVCAN:
#if HAL_WITH_UAVCAN
        dstate->auto_detected_baud = false; // specified, not detected
        new_gps = AP_GPS_UAVCAN::probe(*this, state[instance]);
        goto found_gps;
#endif
        return; // We don't do anything here if UAVCAN is not supported
    default:
        break;
    }

    if (_port[instance] == nullptr) {
        // UART not available
        return;
    }

    // all remaining drivers automatically cycle through baud rates to detect
    // the correct baud rate, and should have the selected baud broadcast
    dstate->auto_detected_baud = true;

    switch (_type[instance]) {
    // by default the sbf/trimble gps outputs no data on its port, until configured.
    case GPS_TYPE_SBF:
        new_gps = new AP_GPS_SBF(*this, state[instance], _port[instance]);
        break;

    case GPS_TYPE_GSOF:
        new_gps = new AP_GPS_GSOF(*this, state[instance], _port[instance]);
        break;

    case GPS_TYPE_NOVA:
        new_gps = new AP_GPS_NOVA(*this, state[instance], _port[instance]);
        break;

    default:
        break;
    }

    if (now - dstate->last_baud_change_ms > GPS_BAUD_TIME_MS) {
        // try the next baud rate
        // incrementing like this will skip the first element in array of bauds
        // this is okay, and relied upon
        dstate->current_baud++;
        if (dstate->current_baud == ARRAY_SIZE(_baudrates)) {
            dstate->current_baud = 0;
        }
        uint32_t baudrate = _baudrates[dstate->current_baud];
        _port[instance]->begin(baudrate);
        _port[instance]->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        dstate->last_baud_change_ms = now;

        if (_auto_config == GPS_AUTO_CONFIG_ENABLE && new_gps == nullptr) {
            send_blob_start(instance, _initialisation_blob, sizeof(_initialisation_blob));
        }
    }

    if (_auto_config == GPS_AUTO_CONFIG_ENABLE && new_gps == nullptr) {
        send_blob_update(instance);
    }

    while (initblob_state[instance].remaining == 0 && _port[instance]->available() > 0
           && new_gps == nullptr) {
        uint8_t data = _port[instance]->read();
        /*
          running a uBlox at less than 38400 will lead to packet
          corruption, as we can't receive the packets in the 200ms
          window for 5Hz fixes. The NMEA startup message should force
          the uBlox into 115200 no matter what rate it is configured
          for.
        */
        if ((_type[instance] == GPS_TYPE_AUTO || _type[instance] == GPS_TYPE_UBLOX) &&
            ((!_auto_config && _baudrates[dstate->current_baud] >= 38400) ||
             _baudrates[dstate->current_baud] == 115200) &&
            AP_GPS_UBLOX::_detect(dstate->ublox_detect_state, data)) {
            new_gps = new AP_GPS_UBLOX(*this, state[instance], _port[instance]);
        }
#if !HAL_MINIMIZE_FEATURES
        // we drop the MTK drivers when building a small build as they are so rarely used
        // and are surprisingly large
        else if ((_type[instance] == GPS_TYPE_AUTO || _type[instance] == GPS_TYPE_MTK19) &&
                 AP_GPS_MTK19::_detect(dstate->mtk19_detect_state, data)) {
            new_gps = new AP_GPS_MTK19(*this, state[instance], _port[instance]);
        } else if ((_type[instance] == GPS_TYPE_AUTO || _type[instance] == GPS_TYPE_MTK) &&
                   AP_GPS_MTK::_detect(dstate->mtk_detect_state, data)) {
            new_gps = new AP_GPS_MTK(*this, state[instance], _port[instance]);
        }
#endif
        else if ((_type[instance] == GPS_TYPE_AUTO || _type[instance] == GPS_TYPE_SBP) &&
                 AP_GPS_SBP2::_detect(dstate->sbp2_detect_state, data)) {
            new_gps = new AP_GPS_SBP2(*this, state[instance], _port[instance]);
        }
        else if ((_type[instance] == GPS_TYPE_AUTO || _type[instance] == GPS_TYPE_SBP) &&
                 AP_GPS_SBP::_detect(dstate->sbp_detect_state, data)) {
            new_gps = new AP_GPS_SBP(*this, state[instance], _port[instance]);
        }
#if !HAL_MINIMIZE_FEATURES
        else if ((_type[instance] == GPS_TYPE_AUTO || _type[instance] == GPS_TYPE_SIRF) &&
                 AP_GPS_SIRF::_detect(dstate->sirf_detect_state, data)) {
            new_gps = new AP_GPS_SIRF(*this, state[instance], _port[instance]);
        }
#endif
        else if ((_type[instance] == GPS_TYPE_AUTO || _type[instance] == GPS_TYPE_ERB) &&
                 AP_GPS_ERB::_detect(dstate->erb_detect_state, data)) {
            new_gps = new AP_GPS_ERB(*this, state[instance], _port[instance]);
        } else if (_type[instance] == GPS_TYPE_NMEA &&
                   AP_GPS_NMEA::_detect(dstate->nmea_detect_state, data)) {
            new_gps = new AP_GPS_NMEA(*this, state[instance], _port[instance]);
        }
    }

found_gps:
    if (new_gps != nullptr) {
        state[instance].status = NO_FIX;
        drivers[instance] = new_gps;
        timing[instance].last_message_time_ms = now;
        timing[instance].delta_time_ms = GPS_TIMEOUT_MS;
        new_gps->broadcast_gps_type();
    }
}

AP_GPS::GPS_Status AP_GPS::highest_supported_status(uint8_t instance) const
{
    if (instance < GPS_MAX_RECEIVERS && drivers[instance] != nullptr) {
        return drivers[instance]->highest_supported_status();
    }
    return AP_GPS::GPS_OK_FIX_3D;
}

bool AP_GPS::should_df_log() const
{
    AP_Logger *instance = AP_Logger::instance();
    if (instance == nullptr) {
        return false;
    }
    if (_log_gps_bit == (uint32_t)-1) {
        return false;
    }
    if (!instance->should_log(_log_gps_bit)) {
        return false;
    }
    return true;
}


/*
  update one GPS instance. This should be called at 10Hz or greater
 */
void AP_GPS::update_instance(uint8_t instance)
{
    if (_type[instance] == GPS_TYPE_HIL) {
        // in HIL, leave info alone
        return;
    }
    if (_type[instance] == GPS_TYPE_NONE) {
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

    if (drivers[instance] == nullptr || state[instance].status == NO_GPS) {
        // we don't yet know the GPS type of this one, or it has timed
        // out and needs to be re-initialised
        detect_instance(instance);
        return;
    }

    if (_auto_config == GPS_AUTO_CONFIG_ENABLE) {
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
            // do not try to detect again if type is MAV
            if (_type[instance] == GPS_TYPE_MAV) {
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
        if (state[instance].uart_timestamp_ms != 0) {
            // set the timestamp for this messages based on
            // set_uart_timestamp() in backend, if available
            tnow = state[instance].uart_timestamp_ms;
            state[instance].uart_timestamp_ms = 0;
        }
        // delta will only be correct after parsing two messages
        timing[instance].delta_time_ms = tnow - timing[instance].last_message_time_ms;
        timing[instance].last_message_time_ms = tnow;
        if (state[instance].status >= GPS_OK_FIX_2D) {
            timing[instance].last_fix_time_ms = tnow;
        }

        data_should_be_logged = true;
    }

    if (data_should_be_logged &&
        should_df_log() &&
        !AP::ahrs().have_ekf_logging()) {
        AP::logger().Write_GPS(instance);
    }

    if (state[instance].status >= GPS_OK_FIX_3D) {
        const uint64_t now = time_epoch_usec(instance);
        AP::rtc().set_utc_usec(now, AP_RTC::SOURCE_GPS);
    }
}

/*
  update all GPS instances
 */
void AP_GPS::update(void)
{
    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        update_instance(i);
    }

    // calculate number of instances
    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        if (state[i].status != NO_GPS) {
            num_instances = i+1;
        }
    }

    // if blending is requested, attempt to calculate weighting for each GPS
    if (_auto_switch == 2) {
        _output_is_blended = calc_blend_weights();
        // adjust blend health counter
        if (!_output_is_blended) {
            _blend_health_counter = MIN(_blend_health_counter+BLEND_COUNTER_FAILURE_INCREMENT, 100);
        } else if (_blend_health_counter > 0) {
            _blend_health_counter--;
        }
        // stop blending if unhealthy
        if (_blend_health_counter >= 50) {
            _output_is_blended = false;
        }
    } else {
        _output_is_blended = false;
        _blend_health_counter = 0;
    }

    if (_output_is_blended) {
        // Use the weighting to calculate blended GPS states
        calc_blended_state();
        // set primary to the virtual instance
        primary_instance = GPS_BLENDED_INSTANCE;
    } else {
        // use switch logic to find best GPS
        uint32_t now = AP_HAL::millis();
        if (_auto_switch == 3) {
            // select the second GPS instance
            primary_instance = 1;
        } else if (_auto_switch >= 1) {
            // handling switching away from blended GPS
            if (primary_instance == GPS_BLENDED_INSTANCE) {
                primary_instance = 0;
                for (uint8_t i=1; i<GPS_MAX_RECEIVERS; i++) {
                    // choose GPS with highest state or higher number of satellites
                    if ((state[i].status > state[primary_instance].status) ||
                        ((state[i].status == state[primary_instance].status) && (state[i].num_sats > state[primary_instance].num_sats))) {
                        primary_instance = i;
                        _last_instance_swap_ms = now;
                    }
                }
            } else {
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
        } else {
            // AUTO_SWITCH is 0 so no switching of GPSs
            primary_instance = 0;
        }

        // copy the primary instance to the blended instance in case it is enabled later
        state[GPS_BLENDED_INSTANCE] = state[primary_instance];
        _blended_antenna_offset = _antenna_offset[primary_instance];
    }

    // update notify with gps status. We always base this on the primary_instance
    AP_Notify::flags.gps_status = state[primary_instance].status;
    AP_Notify::flags.gps_num_sats = state[primary_instance].num_sats;

}

void AP_GPS::handle_gps_inject(const mavlink_message_t *msg)
{
    mavlink_gps_inject_data_t packet;
    mavlink_msg_gps_inject_data_decode(msg, &packet);
    //TODO: check target

    inject_data(packet.data, packet.len);
}

/*
  pass along a mavlink message (for MAV type)
 */
void AP_GPS::handle_msg(const mavlink_message_t *msg)
{
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_GPS_RTCM_DATA:
        // pass data to de-fragmenter
        handle_gps_rtcm_data(msg);
        break;
    case MAVLINK_MSG_ID_GPS_INJECT_DATA:
        handle_gps_inject(msg);
        break;
    default: {
        uint8_t i;
        for (i=0; i<num_instances; i++) {
            if ((drivers[i] != nullptr) && (_type[i] != GPS_TYPE_NONE)) {
                drivers[i]->handle_msg(msg);
            }
        }
        break;
    }
    }
}

/*
  set HIL (hardware in the loop) status for a GPS instance
 */
void AP_GPS::setHIL(uint8_t instance, GPS_Status _status, uint64_t time_epoch_ms,
                    const Location &_location, const Vector3f &_velocity, uint8_t _num_sats,
                    uint16_t hdop)
{
    if (instance >= GPS_MAX_RECEIVERS) {
        return;
    }
    const uint32_t tnow = AP_HAL::millis();
    GPS_State &istate = state[instance];
    istate.status = _status;
    istate.location = _location;
    istate.velocity = _velocity;
    istate.ground_speed = norm(istate.velocity.x, istate.velocity.y);
    istate.ground_course = wrap_360(degrees(atan2f(istate.velocity.y, istate.velocity.x)));
    istate.hdop = hdop;
    istate.num_sats = _num_sats;
    istate.last_gps_time_ms = tnow;
    uint64_t gps_time_ms = time_epoch_ms - UNIX_OFFSET_MSEC;
    istate.time_week     = gps_time_ms / AP_MSEC_PER_WEEK;
    istate.time_week_ms  = gps_time_ms - istate.time_week * AP_MSEC_PER_WEEK;
    timing[instance].last_message_time_ms = tnow;
    timing[instance].last_fix_time_ms = tnow;
    _type[instance].set(GPS_TYPE_HIL);
}

// set accuracy for HIL
void AP_GPS::setHIL_Accuracy(uint8_t instance, float vdop, float hacc, float vacc, float sacc, bool _have_vertical_velocity, uint32_t sample_ms)
{
    if (instance >= GPS_MAX_RECEIVERS) {
        return;
    }
    GPS_State &istate = state[instance];
    istate.vdop = vdop * 100;
    istate.horizontal_accuracy = hacc;
    istate.vertical_accuracy = vacc;
    istate.speed_accuracy = sacc;
    istate.have_horizontal_accuracy = true;
    istate.have_vertical_accuracy = true;
    istate.have_speed_accuracy = true;
    istate.have_vertical_velocity |= _have_vertical_velocity;
    if (sample_ms != 0) {
        timing[instance].last_message_time_ms = sample_ms;
        timing[instance].last_fix_time_ms = sample_ms;
    }
}

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
void AP_GPS::inject_data(uint8_t *data, uint16_t len)
{
    //Support broadcasting to all GPSes.
    if (_inject_to == GPS_RTK_INJECT_TO_ALL) {
        for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
            inject_data(i, data, len);
        }
    } else {
        inject_data(_inject_to, data, len);
    }
}

void AP_GPS::inject_data(uint8_t instance, uint8_t *data, uint16_t len)
{
    if (instance < GPS_MAX_RECEIVERS && drivers[instance] != nullptr) {
        drivers[instance]->inject_data(data, len);
    }
}

void AP_GPS::send_mavlink_gps_raw(mavlink_channel_t chan)
{
    static uint32_t last_send_time_ms[MAVLINK_COMM_NUM_BUFFERS];
    if (status(0) > AP_GPS::NO_GPS) {
        // when we have a GPS then only send new data
        if (last_send_time_ms[chan] == last_message_time_ms(0)) {
            return;
        }
        last_send_time_ms[chan] = last_message_time_ms(0);
    } else {
        // when we don't have a GPS then send at 1Hz
        uint32_t now = AP_HAL::millis();
        if (now - last_send_time_ms[chan] < 1000) {
            return;
        }
        last_send_time_ms[chan] = now;
    }
    const Location &loc = location(0);
    float hacc = 0.0f;
    float vacc = 0.0f;
    float sacc = 0.0f;
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
        0,                    // TODO: Elipsoid height in mm
        hacc * 1000,          // one-sigma standard deviation in mm
        vacc * 1000,          // one-sigma standard deviation in mm
        sacc * 1000,          // one-sigma standard deviation in mm/s
        0);                   // TODO one-sigma heading accuracy standard deviation
}

void AP_GPS::send_mavlink_gps2_raw(mavlink_channel_t chan)
{
    static uint32_t last_send_time_ms[MAVLINK_COMM_NUM_BUFFERS];
    if (num_instances < 2 || status(1) <= AP_GPS::NO_GPS) {
        return;
    }
    // when we have a GPS then only send new data
    if (last_send_time_ms[chan] == last_message_time_ms(1)) {
        return;
    }
    last_send_time_ms[chan] = last_message_time_ms(1);

    const Location &loc = location(1);
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
        rtk_num_sats(1),
        rtk_age_ms(1));
}

void AP_GPS::send_mavlink_gps_rtk(mavlink_channel_t chan, uint8_t inst)
{
    if (inst >= GPS_MAX_RECEIVERS) {
        return;
    }
    if (drivers[inst] != nullptr && drivers[inst]->supports_mavlink_gps_rtk_message()) {
        drivers[inst]->send_mavlink_gps_rtk(chan);
    }
}

uint8_t AP_GPS::first_unconfigured_gps(void) const
{
    for (int i = 0; i < GPS_MAX_RECEIVERS; i++) {
        if (_type[i] != GPS_TYPE_NONE && (drivers[i] == nullptr || !drivers[i]->is_configured())) {
            return i;
        }
    }
    return GPS_ALL_CONFIGURED;
}

void AP_GPS::broadcast_first_configuration_failure_reason(void) const
{
    const uint8_t unconfigured = first_unconfigured_gps();
    if (drivers[unconfigured] == nullptr) {
        gcs().send_text(MAV_SEVERITY_INFO, "GPS %d: was not found", unconfigured + 1);
    } else {
        drivers[unconfigured]->broadcast_configuration_failure_reason();
    }
}

// pre-arm check that all GPSs are close to each other.  farthest distance between GPSs (in meters) is returned
bool AP_GPS::all_consistent(float &distance) const
{
    // return true immediately if only one valid receiver
    if (num_instances <= 1 ||
        drivers[0] == nullptr || _type[0] == GPS_TYPE_NONE) {
        distance = 0;
        return true;
    }

    // calculate distance
    distance = location_3d_diff_NED(state[0].location, state[1].location).length();
    // success if distance is within 50m
    return (distance < 50);
}

// pre-arm check of GPS blending.  True means healthy or that blending is not being used
bool AP_GPS::blend_health_check() const
{
    return (_blend_health_counter < 50);
}

/*
   re-assemble GPS_RTCM_DATA message
 */
void AP_GPS::handle_gps_rtcm_data(const mavlink_message_t *msg)
{
    mavlink_gps_rtcm_data_t packet;
    mavlink_msg_gps_rtcm_data_decode(msg, &packet);

    if (packet.len > MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN) {
        // invalid packet
        return;
    }

    if ((packet.flags & 1) == 0) {
        // it is not fragmented, pass direct
        inject_data(packet.data, packet.len);
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

    uint8_t fragment = (packet.flags >> 1U) & 0x03;
    uint8_t sequence = (packet.flags >> 3U) & 0x1F;

    // see if this fragment is consistent with existing fragments
    if (rtcm_buffer->fragments_received &&
        (rtcm_buffer->sequence != sequence ||
        (rtcm_buffer->fragments_received & (1U<<fragment)))) {
        // we have one or more partial fragments already received
        // which conflict with the new fragment, discard previous fragments
        memset(rtcm_buffer, 0, sizeof(*rtcm_buffer));
    }

    // add this fragment
    rtcm_buffer->sequence = sequence;
    rtcm_buffer->fragments_received |= (1U << fragment);

    // copy the data
    memcpy(&rtcm_buffer->buffer[MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN*(uint16_t)fragment], packet.data, packet.len);

    // when we get a fragment of less than max size then we know the
    // number of fragments. Note that this means if you want to send a
    // block of RTCM data of an exact multiple of the buffer size you
    // need to send a final packet of zero length
    if (packet.len < MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN) {
        rtcm_buffer->fragment_count = fragment+1;
        rtcm_buffer->total_length = (MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN*fragment) + packet.len;
    } else if (rtcm_buffer->fragments_received == 0x0F) {
        // special case of 4 full fragments
        rtcm_buffer->fragment_count = 4;
        rtcm_buffer->total_length = MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN*4;
    }


    // see if we have all fragments
    if (rtcm_buffer->fragment_count != 0 &&
        rtcm_buffer->fragments_received == (1U << rtcm_buffer->fragment_count) - 1) {
        // we have them all, inject
        inject_data(rtcm_buffer->buffer, rtcm_buffer->total_length);
        memset(rtcm_buffer, 0, sizeof(*rtcm_buffer));
    }
}

void AP_GPS::Write_AP_Logger_Log_Startup_messages()
{
    for (uint8_t instance=0; instance<num_instances; instance++) {
        if (drivers[instance] == nullptr || state[instance].status == NO_GPS) {
            continue;
        }
        drivers[instance]->Write_AP_Logger_Log_Startup_messages();
    }
}

/*
  return the expected lag (in seconds) in the position and velocity readings from the gps
  return true if the GPS hardware configuration is known or the delay parameter has been set
 */
bool AP_GPS::get_lag(uint8_t instance, float &lag_sec) const
{
    // always enusre a lag is provided
    lag_sec = GPS_WORST_LAG_SEC;

    // return lag of blended GPS
    if (instance == GPS_BLENDED_INSTANCE) {
        lag_sec = _blended_lag_sec;
        // auto switching uses all GPS receivers, so all must be configured
        return all_configured();
    }

    if (_delay_ms[instance] > 0) {
        // if the user has specified a non zero time delay, always return that value
        lag_sec = 0.001f * (float)_delay_ms[instance];
        // the user is always right !!
        return true;
    } else if (drivers[instance] == nullptr || state[instance].status == NO_GPS) {
        // no GPS was detected in this instance so return the worst possible lag term
        if (_type[instance] == GPS_TYPE_NONE) {
            lag_sec = 0.0f;
            return true;
        }
        return _type[instance] == GPS_TYPE_AUTO;
    } else {
        // the user has not specified a delay so we determine it from the GPS type
        return drivers[instance]->get_lag(lag_sec);
    }
}

// return a 3D vector defining the offset of the GPS antenna in meters relative to the body frame origin
const Vector3f &AP_GPS::get_antenna_offset(uint8_t instance) const
{
    if (instance == GPS_MAX_RECEIVERS) {
        // return an offset for the blended GPS solution
        return _blended_antenna_offset;
    } else {
        return _antenna_offset[instance];
    }
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
    if (instance >= num_instances || _rate_ms[instance] <= 0) {
        return GPS_MAX_RATE_MS;
    }
    return MIN(_rate_ms[instance], GPS_MAX_RATE_MS);
}

/*
 calculate the weightings used to blend GPSs location and velocity data
*/
bool AP_GPS::calc_blend_weights(void)
{
    // zero the blend weights
    memset(&_blend_weights, 0, sizeof(_blend_weights));

    // exit immediately if not enough receivers to do blending
    if (num_instances < 2 || drivers[1] == nullptr || _type[1] == GPS_TYPE_NONE) {
        return false;
    }

    // Use the oldest non-zero time, but if time difference is excessive, use newest to prevent a disconnected receiver from blocking updates
    uint32_t max_ms = 0; // newest non-zero system time of arrival of a GPS message
    uint32_t min_ms = -1; // oldest non-zero system time of arrival of a GPS message
    int16_t max_rate_ms = 0; // largest update interval of a GPS receiver
    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        // Find largest and smallest times
        if (state[i].last_gps_time_ms > max_ms) {
            max_ms = state[i].last_gps_time_ms;
        }
        if ((state[i].last_gps_time_ms < min_ms) && (state[i].last_gps_time_ms > 0)) {
            min_ms = state[i].last_gps_time_ms;
        }
        if (get_rate_ms(i) > max_rate_ms) {
            max_rate_ms = get_rate_ms(i);
        }
    }
    if ((int32_t)(max_ms - min_ms) < (int32_t)(2 * max_rate_ms)) {
        // data is not too delayed so use the oldest time_stamp to give a chance for data from that receiver to be updated
        state[GPS_BLENDED_INSTANCE].last_gps_time_ms = min_ms;
    } else {
        // receiver data has timed out so fail out of blending
        return false;
    }

    // calculate the sum squared speed accuracy across all GPS sensors
    float speed_accuracy_sum_sq = 0.0f;
    if (_blend_mask & BLEND_MASK_USE_SPD_ACC) {
        for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
            if (state[i].status >= GPS_OK_FIX_3D) {
                if (state[i].have_speed_accuracy && state[i].speed_accuracy > 0.0f) {
                    speed_accuracy_sum_sq += state[i].speed_accuracy * state[i].speed_accuracy;
                } else {
                    // not all receivers support this metric so set it to zero and don't use it
                    speed_accuracy_sum_sq = 0.0f;
                    break;
                }
            }
        }
    }

    // calculate the sum squared horizontal position accuracy across all GPS sensors
    float horizontal_accuracy_sum_sq = 0.0f;
    if (_blend_mask & BLEND_MASK_USE_HPOS_ACC) {
        for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
            if (state[i].status >= GPS_OK_FIX_2D) {
                if (state[i].have_horizontal_accuracy && state[i].horizontal_accuracy > 0.0f) {
                    horizontal_accuracy_sum_sq += state[i].horizontal_accuracy * state[i].horizontal_accuracy;
                } else {
                    // not all receivers support this metric so set it to zero and don't use it
                    horizontal_accuracy_sum_sq = 0.0f;
                    break;
                }
            }
        }
    }

    // calculate the sum squared vertical position accuracy across all GPS sensors
    float vertical_accuracy_sum_sq = 0.0f;
    if (_blend_mask & BLEND_MASK_USE_VPOS_ACC) {
        for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
            if (state[i].status >= GPS_OK_FIX_3D) {
                if (state[i].have_vertical_accuracy && state[i].vertical_accuracy > 0.0f) {
                    vertical_accuracy_sum_sq += state[i].vertical_accuracy * state[i].vertical_accuracy;
                } else {
                    // not all receivers support this metric so set it to zero and don't use it
                    vertical_accuracy_sum_sq = 0.0f;
                    break;
                }
            }
        }
    }
    // Check if we can do blending using reported accuracy
    bool can_do_blending = (horizontal_accuracy_sum_sq > 0.0f || vertical_accuracy_sum_sq > 0.0f || speed_accuracy_sum_sq > 0.0f);

    // if we can't do blending using reported accuracy, return false and hard switch logic will be used instead
    if (!can_do_blending) {
        return false;
    }

    float sum_of_all_weights = 0.0f;

    // calculate a weighting using the reported horizontal position
    float hpos_blend_weights[GPS_MAX_RECEIVERS] = {};
    if (horizontal_accuracy_sum_sq > 0.0f && (_blend_mask & BLEND_MASK_USE_HPOS_ACC)) {
        // calculate the weights using the inverse of the variances
        float sum_of_hpos_weights = 0.0f;
        for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
            if (state[i].status >= GPS_OK_FIX_2D && state[i].horizontal_accuracy >= 0.001f) {
                hpos_blend_weights[i] = horizontal_accuracy_sum_sq / (state[i].horizontal_accuracy * state[i].horizontal_accuracy);
                sum_of_hpos_weights += hpos_blend_weights[i];
            }
        }
        // normalise the weights
        if (sum_of_hpos_weights > 0.0f) {
            for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
                hpos_blend_weights[i] = hpos_blend_weights[i] / sum_of_hpos_weights;
            }
            sum_of_all_weights += 1.0f;
        }
    }

    // calculate a weighting using the reported vertical position accuracy
    float vpos_blend_weights[GPS_MAX_RECEIVERS] = {};
    if (vertical_accuracy_sum_sq > 0.0f && (_blend_mask & BLEND_MASK_USE_VPOS_ACC)) {
        // calculate the weights using the inverse of the variances
        float sum_of_vpos_weights = 0.0f;
        for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
            if (state[i].status >= GPS_OK_FIX_3D && state[i].vertical_accuracy >= 0.001f) {
                vpos_blend_weights[i] = vertical_accuracy_sum_sq / (state[i].vertical_accuracy * state[i].vertical_accuracy);
                sum_of_vpos_weights += vpos_blend_weights[i];
            }
        }
        // normalise the weights
        if (sum_of_vpos_weights > 0.0f) {
            for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
                vpos_blend_weights[i] = vpos_blend_weights[i] / sum_of_vpos_weights;
            }
            sum_of_all_weights += 1.0f;
        };
    }

    // calculate a weighting using the reported speed accuracy
    float spd_blend_weights[GPS_MAX_RECEIVERS] = {};
    if (speed_accuracy_sum_sq > 0.0f && (_blend_mask & BLEND_MASK_USE_SPD_ACC)) {
        // calculate the weights using the inverse of the variances
        float sum_of_spd_weights = 0.0f;
        for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
            if (state[i].status >= GPS_OK_FIX_3D && state[i].speed_accuracy >= 0.001f) {
                spd_blend_weights[i] = speed_accuracy_sum_sq / (state[i].speed_accuracy * state[i].speed_accuracy);
                sum_of_spd_weights += spd_blend_weights[i];
            }
        }
        // normalise the weights
        if (sum_of_spd_weights > 0.0f) {
            for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
                spd_blend_weights[i] = spd_blend_weights[i] / sum_of_spd_weights;
            }
            sum_of_all_weights += 1.0f;
        }
    }

    // calculate an overall weight
    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        _blend_weights[i] = (hpos_blend_weights[i] + vpos_blend_weights[i] + spd_blend_weights[i]) / sum_of_all_weights;
    }

    return true;
}

/*
 calculate a blended GPS state
*/
void AP_GPS::calc_blended_state(void)
{
    // initialise the blended states so we can accumulate the results using the weightings for each GPS receiver
    state[GPS_BLENDED_INSTANCE].instance = GPS_BLENDED_INSTANCE;
    state[GPS_BLENDED_INSTANCE].status = NO_FIX;
    state[GPS_BLENDED_INSTANCE].time_week_ms = 0;
    state[GPS_BLENDED_INSTANCE].time_week = 0;
    state[GPS_BLENDED_INSTANCE].ground_speed = 0.0f;
    state[GPS_BLENDED_INSTANCE].ground_course = 0.0f;
    state[GPS_BLENDED_INSTANCE].hdop = GPS_UNKNOWN_DOP;
    state[GPS_BLENDED_INSTANCE].vdop = GPS_UNKNOWN_DOP;
    state[GPS_BLENDED_INSTANCE].num_sats = 0;
    state[GPS_BLENDED_INSTANCE].velocity.zero();
    state[GPS_BLENDED_INSTANCE].speed_accuracy = 1e6f;
    state[GPS_BLENDED_INSTANCE].horizontal_accuracy = 1e6f;
    state[GPS_BLENDED_INSTANCE].vertical_accuracy = 1e6f;
    state[GPS_BLENDED_INSTANCE].have_vertical_velocity = false;
    state[GPS_BLENDED_INSTANCE].have_speed_accuracy = false;
    state[GPS_BLENDED_INSTANCE].have_horizontal_accuracy = false;
    state[GPS_BLENDED_INSTANCE].have_vertical_accuracy = false;
    state[GPS_BLENDED_INSTANCE].location = {};

    _blended_antenna_offset.zero();
    _blended_lag_sec = 0;

    timing[GPS_BLENDED_INSTANCE].last_fix_time_ms = 0;
    timing[GPS_BLENDED_INSTANCE].last_message_time_ms = 0;

    // combine the states into a blended solution
    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        // use the highest status
        if (state[i].status > state[GPS_BLENDED_INSTANCE].status) {
            state[GPS_BLENDED_INSTANCE].status = state[i].status;
        }

        // calculate a blended average velocity
        state[GPS_BLENDED_INSTANCE].velocity += state[i].velocity * _blend_weights[i];

        // report the best valid accuracies and DOP metrics

        if (state[i].have_horizontal_accuracy && state[i].horizontal_accuracy > 0.0f && state[i].horizontal_accuracy < state[GPS_BLENDED_INSTANCE].horizontal_accuracy) {
            state[GPS_BLENDED_INSTANCE].have_horizontal_accuracy = true;
            state[GPS_BLENDED_INSTANCE].horizontal_accuracy = state[i].horizontal_accuracy;
        }

        if (state[i].have_vertical_accuracy && state[i].vertical_accuracy > 0.0f && state[i].vertical_accuracy < state[GPS_BLENDED_INSTANCE].vertical_accuracy) {
            state[GPS_BLENDED_INSTANCE].have_vertical_accuracy = true;
            state[GPS_BLENDED_INSTANCE].vertical_accuracy = state[i].vertical_accuracy;
        }

        if (state[i].have_vertical_velocity) {
            state[GPS_BLENDED_INSTANCE].have_vertical_velocity = true;
        }

        if (state[i].have_speed_accuracy && state[i].speed_accuracy > 0.0f && state[i].speed_accuracy < state[GPS_BLENDED_INSTANCE].speed_accuracy) {
            state[GPS_BLENDED_INSTANCE].have_speed_accuracy = true;
            state[GPS_BLENDED_INSTANCE].speed_accuracy = state[i].speed_accuracy;
        }

        if (state[i].hdop > 0 && state[i].hdop < state[GPS_BLENDED_INSTANCE].hdop) {
            state[GPS_BLENDED_INSTANCE].hdop = state[i].hdop;
        }

        if (state[i].vdop > 0 && state[i].vdop < state[GPS_BLENDED_INSTANCE].vdop) {
            state[GPS_BLENDED_INSTANCE].vdop = state[i].vdop;
        }

        if (state[i].num_sats > 0 && state[i].num_sats > state[GPS_BLENDED_INSTANCE].num_sats) {
            state[GPS_BLENDED_INSTANCE].num_sats = state[i].num_sats;
        }

        // report a blended average GPS antenna position
        Vector3f temp_antenna_offset = _antenna_offset[i];
        temp_antenna_offset *= _blend_weights[i];
        _blended_antenna_offset += temp_antenna_offset;

        // blend the timing data
        if (timing[i].last_fix_time_ms > timing[GPS_BLENDED_INSTANCE].last_fix_time_ms) {
            timing[GPS_BLENDED_INSTANCE].last_fix_time_ms = timing[i].last_fix_time_ms;
        }
        if (timing[i].last_message_time_ms > timing[GPS_BLENDED_INSTANCE].last_message_time_ms) {
            timing[GPS_BLENDED_INSTANCE].last_message_time_ms = timing[i].last_message_time_ms;
        }

    }

    /*
     * Calculate an instantaneous weighted/blended average location from the available GPS instances and store in the _output_state.
     * This will be statistically the most likely location, but will be not stable enough for direct use by the autopilot.
    */

    // Use the GPS with the highest weighting as the reference position
    float best_weight = 0.0f;
    uint8_t best_index = 0;
    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        if (_blend_weights[i] > best_weight) {
            best_weight = _blend_weights[i];
            best_index = i;
            state[GPS_BLENDED_INSTANCE].location = state[i].location;
        }
    }

    // Calculate the weighted sum of horizontal and vertical position offsets relative to the reference position
    Vector2f blended_NE_offset_m;
    float blended_alt_offset_cm = 0.0f;
    blended_NE_offset_m.zero();
    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        if (_blend_weights[i] > 0.0f && i != best_index) {
            blended_NE_offset_m += location_diff(state[GPS_BLENDED_INSTANCE].location, state[i].location) * _blend_weights[i];
            blended_alt_offset_cm += (float)(state[i].location.alt - state[GPS_BLENDED_INSTANCE].location.alt) * _blend_weights[i];
        }
    }

    // Add the sum of weighted offsets to the reference location to obtain the blended location
    location_offset(state[GPS_BLENDED_INSTANCE].location, blended_NE_offset_m.x, blended_NE_offset_m.y);
    state[GPS_BLENDED_INSTANCE].location.alt += (int)blended_alt_offset_cm;

    // Calculate ground speed and course from blended velocity vector
    state[GPS_BLENDED_INSTANCE].ground_speed = norm(state[GPS_BLENDED_INSTANCE].velocity.x, state[GPS_BLENDED_INSTANCE].velocity.y);
    state[GPS_BLENDED_INSTANCE].ground_course = wrap_360(degrees(atan2f(state[GPS_BLENDED_INSTANCE].velocity.y, state[GPS_BLENDED_INSTANCE].velocity.x)));

    /*
     * The blended location in _output_state.location is not stable enough to be used by the autopilot
     * as it will move around as the relative accuracy changes. To mitigate this effect a low-pass filtered
     * offset from each GPS location to the blended location is calculated and the filtered offset is applied
     * to each receiver.
    */

    // Calculate filter coefficients to be applied to the offsets for each GPS position and height offset
    // A weighting of 1 will make the offset adjust the slowest, a weighting of 0 will make it adjust with zero filtering
    float alpha[GPS_MAX_RECEIVERS] = {};
    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        if (state[i].last_gps_time_ms - _last_time_updated[i] > 0) {
            float min_alpha = constrain_float(_omega_lpf * 0.001f * (float)(state[i].last_gps_time_ms - _last_time_updated[i]), 0.0f, 1.0f);
            if (_blend_weights[i] > min_alpha) {
                alpha[i] = min_alpha / _blend_weights[i];
            } else {
                alpha[i] = 1.0f;
            }
            _last_time_updated[i] = state[i].last_gps_time_ms;
        }
    }

    // Calculate the offset from each GPS solution to the blended solution
    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        _NE_pos_offset_m[i] = location_diff(state[i].location, state[GPS_BLENDED_INSTANCE].location) * alpha[i] + _NE_pos_offset_m[i] * (1.0f - alpha[i]);
        _hgt_offset_cm[i] = (float)(state[GPS_BLENDED_INSTANCE].location.alt - state[i].location.alt) *  alpha[i] + _hgt_offset_cm[i] * (1.0f - alpha[i]);
    }

    // Calculate a corrected location for each GPS
    Location corrected_location[GPS_MAX_RECEIVERS];
    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        corrected_location[i] = state[i].location;
        location_offset(corrected_location[i], _NE_pos_offset_m[i].x, _NE_pos_offset_m[i].y);
        corrected_location[i].alt += (int)(_hgt_offset_cm[i]);
    }

    // If the GPS week is the same then use a blended time_week_ms
    // If week is different, then use time stamp from GPS with largest weighting
    // detect inconsistent week data
    uint8_t last_week_instance = 0;
    bool weeks_consistent = true;
    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        if (last_week_instance == 0 && _blend_weights[i] > 0) {
            // this is our first valid sensor week data
            last_week_instance = state[i].time_week;
        } else if (last_week_instance != 0 && _blend_weights[i] > 0 && last_week_instance != state[i].time_week) {
            // there is valid sensor week data that is inconsistent
            weeks_consistent = false;
        }
    }
    // calculate output
    if (!weeks_consistent) {
        // use data from highest weighted sensor
        state[GPS_BLENDED_INSTANCE].time_week = state[best_index].time_week;
        state[GPS_BLENDED_INSTANCE].time_week_ms = state[best_index].time_week_ms;
    } else {
        // use week number from highest weighting GPS (they should all have the same week number)
        state[GPS_BLENDED_INSTANCE].time_week = state[best_index].time_week;
        // calculate a blended value for the number of ms lapsed in the week
        double temp_time_0 = 0.0;
        for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
            if (_blend_weights[i] > 0.0f) {
                temp_time_0 += (double)state[i].time_week_ms * (double)_blend_weights[i];
            }
        }
        state[GPS_BLENDED_INSTANCE].time_week_ms = (uint32_t)temp_time_0;
    }

    // calculate a blended value for the timing data and lag
    double temp_time_1 = 0.0;
    double temp_time_2 = 0.0;
    for (uint8_t i=0; i<GPS_MAX_RECEIVERS; i++) {
        if (_blend_weights[i] > 0.0f) {
            temp_time_1 += (double)timing[i].last_fix_time_ms * (double) _blend_weights[i];
            temp_time_2 += (double)timing[i].last_message_time_ms * (double)_blend_weights[i];
            float gps_lag_sec = 0;
            get_lag(i, gps_lag_sec);
            _blended_lag_sec += gps_lag_sec * _blend_weights[i];
        }
    }
    timing[GPS_BLENDED_INSTANCE].last_fix_time_ms = (uint32_t)temp_time_1;
    timing[GPS_BLENDED_INSTANCE].last_message_time_ms = (uint32_t)temp_time_2;
}

bool AP_GPS::is_healthy(uint8_t instance) const {
    return drivers[instance] != nullptr &&
           last_message_delta_time_ms(instance) < GPS_MAX_DELTA_MS &&
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

namespace AP {

AP_GPS &gps()
{
    return AP_GPS::gps();
}

};
