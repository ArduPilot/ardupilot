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

#include "AP_GPS_NOVA.h"
#include "AP_GPS_ERB.h"
#include "AP_GPS_GSOF.h"
#include "AP_GPS_MTK.h"
#include "AP_GPS_MTK19.h"
#include "AP_GPS_NMEA.h"
#include "AP_GPS_PX4.h"
#include "AP_GPS_QURT.h"
#include "AP_GPS_SBF.h"
#include "AP_GPS_SBP.h"
#include "AP_GPS_SIRF.h"
#include "AP_GPS_UBLOX.h"
#include "AP_GPS_MAV.h"
#include "GPS_Backend.h"

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_GPS::var_info[] = {
    // @Param: TYPE
    // @DisplayName: GPS type
    // @Description: GPS type
    // @Values: 0:None,1:AUTO,2:uBlox,3:MTK,4:MTK19,5:NMEA,6:SiRF,7:HIL,8:SwiftNav,9:PX4-UAVCAN,10:SBF,11:GSOF,12:QURT,13:ERB,14:MAV,15:NOVA
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("TYPE",    0, AP_GPS, _type[0], 1),

    // @Param: TYPE2
    // @DisplayName: 2nd GPS type
    // @Description: GPS type of 2nd GPS
    // @Values: 0:None,1:AUTO,2:uBlox,3:MTK,4:MTK19,5:NMEA,6:SiRF,7:HIL,8:SwiftNav,9:PX4-UAVCAN,10:SBF,11:GSOF,12:QURT,13:ERB,14:MAV,15:NOVA
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
    // @Values: 0:Disabled,1:Enabled
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
    // @Units: Degrees
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
    // @Values: 0x0000:None, 0xFFFF:All, 0xFF00:External only
    // @User: Advanced
    AP_GROUPINFO("SBP_LOGMASK", 8, AP_GPS, _sbp_logmask, 0xFF00),

    // @Param: RAW_DATA
    // @DisplayName: Raw data logging
    // @Description: Enable logging of RXM raw data from uBlox which includes carrier phase and pseudo range information. This allows for post processing of dataflash logs for more precise positioning. Note that this requires a raw capable uBlox such as the 6P or 6T.
    // @Values: 0:Disabled,1:log every sample,5:log every 5 samples
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
    AP_GROUPINFO("SAVE_CFG", 11, AP_GPS, _save_config, 0),

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
    // @Units: milliseconds
    // @Values: 100:10Hz,125:8Hz,200:5Hz
    // @User: Advanced
    AP_GROUPINFO("RATE_MS", 14, AP_GPS, _rate_ms[0], 200),

    // @Param: RATE_MS2
    // @DisplayName: GPS 2 update rate in milliseconds
    // @Description: Controls how often the GPS should provide a position update. Lowering below 5Hz is not allowed
    // @Units: milliseconds
    // @Values: 100:10Hz,125:8Hz,200:5Hz
    // @User: Advanced
    AP_GROUPINFO("RATE_MS2", 15, AP_GPS, _rate_ms[1], 200),

    // @Param: POS1_X
    // @DisplayName: Antenna X position offset
    // @Description: X position of the first GPS antenna in body frame. Positive X is forward of the origin. Use antenna phase centroid location if provided by the manufacturer.
    // @Units: m
    // @User: Advanced

    // @Param: POS1_Y
    // @DisplayName: Antenna Y position offset
    // @Description: Y position of the first GPS antenna in body frame. Positive Y is to the right of the origin. Use antenna phase centroid location if provided by the manufacturer.
    // @Units: m
    // @User: Advanced

    // @Param: POS1_Z
    // @DisplayName: Antenna Z position offset
    // @Description: Z position of the first GPS antenna in body frame. Positive Z is down from the origin. Use antenna phase centroid location if provided by the manufacturer.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("POS1", 16, AP_GPS, _antenna_offset[0], 0.0f),

    // @Param: POS2_X
    // @DisplayName: Antenna X position offset
    // @Description: X position of the second GPS antenna in body frame. Positive X is forward of the origin. Use antenna phase centroid location if provided by the manufacturer.
    // @Units: m
    // @User: Advanced

    // @Param: POS2_Y
    // @DisplayName: Antenna Y position offset
    // @Description: Y position of the second GPS antenna in body frame. Positive Y is to the right of the origin. Use antenna phase centroid location if provided by the manufacturer.
    // @Units: m
    // @User: Advanced

    // @Param: POS2_Z
    // @DisplayName: Antenna Z position offset
    // @Description: Z position of the second GPS antenna in body frame. Positive Z is down from the origin. Use antenna phase centroid location if provided by the manufacturer.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("POS2", 17, AP_GPS, _antenna_offset[1], 0.0f),

    // @Param: DELAY_MS
    // @DisplayName: GPS delay in milliseconds
    // @Description: Controls the amount of GPS  measurement delay that the autopilot compensates for. Set to zero to use the default delay for the detected GPS type.
    // @Units: milliseconds
    // @Range: 0 250
    // @User: Advanced
    AP_GROUPINFO("DELAY_MS", 18, AP_GPS, _delay_ms[0], 0),

    // @Param: DELAY_MS2
    // @DisplayName: GPS 2 delay in milliseconds
    // @Description: Controls the amount of GPS  measurement delay that the autopilot compensates for. Set to zero to use the default delay for the detected GPS type.
    // @Units: milliseconds
    // @Range: 0 250
    // @User: Advanced
    AP_GROUPINFO("DELAY_MS2", 19, AP_GPS, _delay_ms[1], 0),
    AP_GROUPEND
};

/// Startup initialisation.
void AP_GPS::init(DataFlash_Class *dataflash, const AP_SerialManager& serial_manager)
{
    _DataFlash = dataflash;
    primary_instance = 0;

    // search for serial ports with gps protocol
    _port[0] = serial_manager.find_serial(AP_SerialManager::SerialProtocol_GPS, 0);
    _port[1] = serial_manager.find_serial(AP_SerialManager::SerialProtocol_GPS, 1);
    _last_instance_swap_ms = 0;
}

// baudrates to try to detect GPSes with
const uint32_t AP_GPS::_baudrates[] = {4800U, 19200U, 38400U, 115200U, 57600U, 9600U, 230400U};

// initialisation blobs to send to the GPS to try to get it into the
// right mode
const char AP_GPS::_initialisation_blob[] = UBLOX_SET_BINARY MTK_SET_BINARY SIRF_SET_BINARY;

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
void
AP_GPS::detect_instance(uint8_t instance)
{
    AP_GPS_Backend *new_gps = nullptr;
    struct detect_state *dstate = &detect_state[instance];
    uint32_t now = AP_HAL::millis();

    switch (_type[instance]) {
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    case GPS_TYPE_PX4:
        // check for explicitly chosen PX4 GPS beforehand
        // it is not possible to autodetect it, nor does it require a real UART
        _broadcast_gps_type("PX4", instance, -1); // baud rate isn't valid
        new_gps = new AP_GPS_PX4(*this, state[instance], _port[instance]);
        goto found_gps;
        break;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_QURT
    case GPS_TYPE_QURT:
        _broadcast_gps_type("QURTGPS", instance, -1); // baud rate isn't valid
        new_gps = new AP_GPS_QURT(*this, state[instance], _port[instance]);
        goto found_gps;
        break;
#endif

    // user has to explicitly set the MAV type, do not use AUTO
    // do not try to detect the MAV type, assume it's there
    case GPS_TYPE_MAV:
        _broadcast_gps_type("MAV", instance, -1);
        new_gps = new AP_GPS_MAV(*this, state[instance], nullptr);
        goto found_gps;
        break;

    default:
        break;
    }

    if (_port[instance] == nullptr) {
        // UART not available
        return;
    }

    state[instance].instance = instance;
    state[instance].status = NO_GPS;
    state[instance].hdop = 9999;

    switch (_type[instance]) {
    // by default the sbf/trimble gps outputs no data on its port, until configured.
    case GPS_TYPE_SBF:
        _broadcast_gps_type("SBF", instance, -1); // baud rate isn't valid
        new_gps = new AP_GPS_SBF(*this, state[instance], _port[instance]);
        break;

    case GPS_TYPE_GSOF:
        _broadcast_gps_type("GSOF", instance, -1); // baud rate isn't valid
        new_gps = new AP_GPS_GSOF(*this, state[instance], _port[instance]);
        break;

    case GPS_TYPE_NOVA:
        _broadcast_gps_type("NOVA", instance, -1); // baud rate isn't valid
        new_gps = new AP_GPS_NOVA(*this, state[instance], _port[instance]);
        break;

    default:
        break;
    }

    // record the time when we started detection. This is used to try
    // to avoid initialising a uBlox as a NMEA GPS
    if (dstate->detect_started_ms == 0) {
        dstate->detect_started_ms = now;
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

        if (_auto_config == 1) {
            send_blob_start(instance, _initialisation_blob, sizeof(_initialisation_blob));
        }
    }

    if (_auto_config == 1) {
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
            _broadcast_gps_type("u-blox", instance, dstate->current_baud);
            new_gps = new AP_GPS_UBLOX(*this, state[instance], _port[instance]);
        } 
        else if ((_type[instance] == GPS_TYPE_AUTO || _type[instance] == GPS_TYPE_MTK19) &&
                 AP_GPS_MTK19::_detect(dstate->mtk19_detect_state, data)) {
            _broadcast_gps_type("MTK19", instance, dstate->current_baud);
            new_gps = new AP_GPS_MTK19(*this, state[instance], _port[instance]);
        } 
        else if ((_type[instance] == GPS_TYPE_AUTO || _type[instance] == GPS_TYPE_MTK) &&
                 AP_GPS_MTK::_detect(dstate->mtk_detect_state, data)) {
            _broadcast_gps_type("MTK", instance, dstate->current_baud);
            new_gps = new AP_GPS_MTK(*this, state[instance], _port[instance]);
        }
        else if ((_type[instance] == GPS_TYPE_AUTO || _type[instance] == GPS_TYPE_SBP) &&
                 AP_GPS_SBP::_detect(dstate->sbp_detect_state, data)) {
            _broadcast_gps_type("SBP", instance, dstate->current_baud);
            new_gps = new AP_GPS_SBP(*this, state[instance], _port[instance]);
        }
        // save a bit of code space on a 1280
        else if ((_type[instance] == GPS_TYPE_AUTO || _type[instance] == GPS_TYPE_SIRF) &&
                 AP_GPS_SIRF::_detect(dstate->sirf_detect_state, data)) {
            _broadcast_gps_type("SIRF", instance, dstate->current_baud);
            new_gps = new AP_GPS_SIRF(*this, state[instance], _port[instance]);
        }
        else if ((_type[instance] == GPS_TYPE_AUTO || _type[instance] == GPS_TYPE_ERB) &&
                 AP_GPS_ERB::_detect(dstate->erb_detect_state, data)) {
            _broadcast_gps_type("ERB", instance, dstate->current_baud);
            new_gps = new AP_GPS_ERB(*this, state[instance], _port[instance]);
        }
        else if (now - dstate->detect_started_ms > (ARRAY_SIZE(_baudrates) * GPS_BAUD_TIME_MS)) {
            // prevent false detection of NMEA mode in
            // a MTK or UBLOX which has booted in NMEA mode
            if ((_type[instance] == GPS_TYPE_AUTO || _type[instance] == GPS_TYPE_NMEA) &&
                AP_GPS_NMEA::_detect(dstate->nmea_detect_state, data)) {
                _broadcast_gps_type("NMEA", instance, dstate->current_baud);
                new_gps = new AP_GPS_NMEA(*this, state[instance], _port[instance]);
            }
        }
    }

found_gps:
    if (new_gps != nullptr) {
        state[instance].status = NO_FIX;
        drivers[instance] = new_gps;
        timing[instance].last_message_time_ms = now;
    }
}

AP_GPS::GPS_Status 
AP_GPS::highest_supported_status(uint8_t instance) const
{
    if (drivers[instance] != nullptr) {
        return drivers[instance]->highest_supported_status();
    }
    return AP_GPS::GPS_OK_FIX_3D;
}

AP_GPS::GPS_Status 
AP_GPS::highest_supported_status(void) const
{
    if (drivers[primary_instance] != nullptr) {
        return drivers[primary_instance]->highest_supported_status();
    }
    return AP_GPS::GPS_OK_FIX_3D;
}


/*
  update one GPS instance. This should be called at 10Hz or greater
 */
void
AP_GPS::update_instance(uint8_t instance)
{
    if (_type[instance] == GPS_TYPE_HIL) {
        // in HIL, leave info alone
        return;
    }
    if (_type[instance] == GPS_TYPE_NONE) {
        // not enabled
        state[instance].status = NO_GPS;
        state[instance].hdop = 9999;
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

    if (_auto_config == 1) {
        send_blob_update(instance);
    }

    // we have an active driver for this instance
    bool result = drivers[instance]->read();
    uint32_t tnow = AP_HAL::millis();

    // if we did not get a message, and the idle timer of 2 seconds
    // has expired, re-initialise the GPS. This will cause GPS
    // detection to run again
    if (!result) {
        if (tnow - timing[instance].last_message_time_ms > 2000) {
            // free the driver before we run the next detection, so we
            // don't end up with two allocated at any time
            delete drivers[instance];
            drivers[instance] = nullptr;
            memset(&state[instance], 0, sizeof(state[instance]));
            state[instance].instance = instance;
            state[instance].status = NO_GPS;
            state[instance].hdop = 9999;
            timing[instance].last_message_time_ms = tnow;
        }
    } else {
        timing[instance].last_message_time_ms = tnow;
        if (state[instance].status >= GPS_OK_FIX_2D) {
            timing[instance].last_fix_time_ms = tnow;
        }
    }
}

/*
  update all GPS instances
 */
void
AP_GPS::update(void)
{
    for (uint8_t i=0; i<GPS_MAX_INSTANCES; i++) {
        update_instance(i);
    }

    // work out which GPS is the primary, and how many sensors we have
    for (uint8_t i=0; i<GPS_MAX_INSTANCES; i++) {
        if (state[i].status != NO_GPS) {
            num_instances = i+1;
        }
        if (_auto_switch) {            
            if (i == primary_instance) {
                continue;
            }
            if (state[i].status > state[primary_instance].status) {
                // we have a higher status lock, change GPS
                primary_instance = i;
                continue;
            }

            bool another_gps_has_1_or_more_sats = (state[i].num_sats >= state[primary_instance].num_sats + 1);

            if (state[i].status == state[primary_instance].status && another_gps_has_1_or_more_sats) {

                uint32_t now = AP_HAL::millis();
                bool another_gps_has_2_or_more_sats = (state[i].num_sats >= state[primary_instance].num_sats + 2);

                if ( (another_gps_has_1_or_more_sats && (now - _last_instance_swap_ms) >= 20000) ||
                     (another_gps_has_2_or_more_sats && (now - _last_instance_swap_ms) >= 5000 ) ) {
                // this GPS has more satellites than the
                // current primary, switch primary. Once we switch we will
                // then tend to stick to the new GPS as primary. We don't
                // want to switch too often as it will look like a
                // position shift to the controllers.
                primary_instance = i;
                _last_instance_swap_ms = now;
                }
            }
        } else {
            primary_instance = 0;
        }
    }

    // update notify with gps status. We always base this on the primary_instance
    AP_Notify::flags.gps_status = state[primary_instance].status;
    AP_Notify::flags.gps_num_sats = state[primary_instance].num_sats;
}

/*
  pass along a mavlink message (for MAV type)
 */
void
AP_GPS::handle_msg(const mavlink_message_t *msg)
{
    if (msg->msgid == MAVLINK_MSG_ID_GPS_RTCM_DATA) {
        // pass data to de-fragmenter
        handle_gps_rtcm_data(msg);
        return;
    }
    uint8_t i;
    for (i=0; i<num_instances; i++) {
        if ((drivers[i] != nullptr) && (_type[i] != GPS_TYPE_NONE)) {
            drivers[i]->handle_msg(msg);
        }
    }
}

/*
  set HIL (hardware in the loop) status for a GPS instance
 */
void 
AP_GPS::setHIL(uint8_t instance, GPS_Status _status, uint64_t time_epoch_ms, 
               const Location &_location, const Vector3f &_velocity, uint8_t _num_sats, 
               uint16_t hdop)
{
    if (instance >= GPS_MAX_INSTANCES) {
        return;
    }
    uint32_t tnow = AP_HAL::millis();
    GPS_State &istate = state[instance];
    istate.status = _status;
    istate.location = _location;
    istate.location.options = 0;
    istate.velocity = _velocity;
    istate.ground_speed = norm(istate.velocity.x, istate.velocity.y);
    istate.ground_course = wrap_360(degrees(atan2f(istate.velocity.y, istate.velocity.x)));
    istate.hdop = hdop;
    istate.num_sats = _num_sats;
    istate.last_gps_time_ms = tnow;
    uint64_t gps_time_ms = time_epoch_ms - (17000ULL*86400ULL + 52*10*7000ULL*86400ULL - GPS_LEAPSECONDS_MILLIS);
    istate.time_week     = gps_time_ms / (86400*7*(uint64_t)1000);
    istate.time_week_ms  = gps_time_ms - istate.time_week*(86400*7*(uint64_t)1000);
    timing[instance].last_message_time_ms = tnow;
    timing[instance].last_fix_time_ms = tnow;
    _type[instance].set(GPS_TYPE_HIL);
}

// set accuracy for HIL
void AP_GPS::setHIL_Accuracy(uint8_t instance, float vdop, float hacc, float vacc, float sacc, bool _have_vertical_velocity, uint32_t sample_ms)
{
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
   Lock a GPS port, prevening the GPS driver from using it. This can
   be used to allow a user to control a GPS port via the
   SERIAL_CONTROL protocol
 */
void 
AP_GPS::lock_port(uint8_t instance, bool lock)
{

    if (instance >= GPS_MAX_INSTANCES) {
        return;
    }
    if (lock) {
        locked_ports |= (1U<<instance);
    } else {
        locked_ports &= ~(1U<<instance);
    }
}

    //Inject a packet of raw binary to a GPS
void 
AP_GPS::inject_data(uint8_t *data, uint8_t len)
{
    //Support broadcasting to all GPSes.
    if (_inject_to == GPS_RTK_INJECT_TO_ALL) {
        for (uint8_t i=0; i<GPS_MAX_INSTANCES; i++) {
            inject_data(i, data, len);
        }
    } else {
        inject_data(_inject_to, data, len);
    }
}

void 
AP_GPS::inject_data(uint8_t instance, uint8_t *data, uint8_t len)
{
    if (instance < GPS_MAX_INSTANCES && drivers[instance] != nullptr) {
        drivers[instance]->inject_data(data, len);
    }
}  

void 
AP_GPS::send_mavlink_gps_raw(mavlink_channel_t chan)
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
        num_sats(0));
}

void 
AP_GPS::send_mavlink_gps2_raw(mavlink_channel_t chan)
{
    static uint32_t last_send_time_ms[MAVLINK_COMM_NUM_BUFFERS];
    if (num_sensors() < 2 || status(1) <= AP_GPS::NO_GPS) {
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
        0,
        0);
}

void 
AP_GPS::send_mavlink_gps_rtk(mavlink_channel_t chan)
{
    if (drivers[0] != nullptr && drivers[0]->highest_supported_status() > AP_GPS::GPS_OK_FIX_3D) {
        drivers[0]->send_mavlink_gps_rtk(chan);
    }
}

void 
AP_GPS::send_mavlink_gps2_rtk(mavlink_channel_t chan)
{
    if (drivers[1] != nullptr && drivers[1]->highest_supported_status() > AP_GPS::GPS_OK_FIX_3D) {
        drivers[1]->send_mavlink_gps_rtk(chan);
    }
}

uint8_t
AP_GPS::first_unconfigured_gps(void) const
{
    for(int i = 0; i < GPS_MAX_INSTANCES; i++) {
        if(_type[i] != GPS_TYPE_NONE && (drivers[i] == nullptr || !drivers[i]->is_configured())) {
            return i;
        }
    }
    return GPS_ALL_CONFIGURED;
}

void
AP_GPS::broadcast_first_configuration_failure_reason(void) const {
    uint8_t unconfigured = first_unconfigured_gps();
    if (drivers[unconfigured] == nullptr) {
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "GPS %d: was not found", unconfigured + 1);
    } else {
        drivers[unconfigured]->broadcast_configuration_failure_reason();
    }
}

void
AP_GPS::_broadcast_gps_type(const char *type, uint8_t instance, int8_t baud_index)
{
    char buffer[64];
    if (baud_index >= 0) {
        hal.util->snprintf(buffer, sizeof(buffer),
                 "GPS %d: detected as %s at %d baud",
                 instance,
                 type,
                 _baudrates[baud_index]);
    } else {
        hal.util->snprintf(buffer, sizeof(buffer),
                 "GPS %d: detected as %s",
                 instance,
                 type);
    }
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, buffer);
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
        inject_data_all(packet.data, packet.len);
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
         rtcm_buffer->fragments_received & (1U<<fragment))) {
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
        inject_data_all(rtcm_buffer->buffer, rtcm_buffer->total_length);
        memset(rtcm_buffer, 0, sizeof(*rtcm_buffer));
    }
}

/*
  inject data into all backends
*/
void AP_GPS::inject_data_all(const uint8_t *data, uint16_t len)
{
    uint8_t i;
    for (i=0; i<num_instances; i++) {
        if ((drivers[i] != nullptr) && (_type[i] != GPS_TYPE_NONE)) {
            drivers[i]->inject_data(data, len);
        }
    }
    
}

/*
  return expected lag from a GPS
 */
float AP_GPS::get_lag(uint8_t instance) const
{
    if (_delay_ms[instance] > 0) {
        // if the user has specified a non zero time delay, always return that value
        return 0.001f * (float)_delay_ms[instance];
    } else if (drivers[instance] == nullptr || state[instance].status == NO_GPS) {
        // no GPS was detected in this instance
        // so return a default delay of 1 measurement interval
        return 0.001f * (float)_rate_ms[instance];
    } else {
        // the user has not specified a delay so we determine it from the GPS type
        return drivers[instance]->get_lag();
    }
}
