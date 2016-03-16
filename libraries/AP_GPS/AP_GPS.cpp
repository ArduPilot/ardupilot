// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_GPS::var_info[] = {
    // @Param: TYPE
    // @DisplayName: GPS type
    // @Description: GPS type
    // @Values: 0:None,1:AUTO,2:uBlox,3:MTK,4:MTK19,5:NMEA,6:SiRF,7:HIL,8:SwiftNav,9:PX4-UAVCAN,10:SBF,11:GSOF
    // @RebootRequired: True
    AP_GROUPINFO("TYPE",    0, AP_GPS, _type[0], 1),

    // @Param: TYPE2
    // @DisplayName: 2nd GPS type
    // @Description: GPS type of 2nd GPS
    // @Values: 0:None,1:AUTO,2:uBlox,3:MTK,4:MTK19,5:NMEA,6:SiRF,7:HIL,8:SwiftNav,9:PX4-UAVCAN,10:SBF,11:GSOF
    // @RebootRequired: True
    AP_GROUPINFO("TYPE2",   1, AP_GPS, _type[1], 0),

    // @Param: NAVFILTER
    // @DisplayName: Navigation filter setting
    // @Description: Navigation filter engine setting
    // @Values: 0:Portable,2:Stationary,3:Pedestrian,4:Automotive,5:Sea,6:Airborne1G,7:Airborne2G,8:Airborne4G
    AP_GROUPINFO("NAVFILTER", 2, AP_GPS, _navfilter, GPS_ENGINE_AIRBORNE_4G),

    // @Param: AUTO_SWITCH
    // @DisplayName: Automatic Switchover Setting
    // @Description: Automatic switchover to GPS reporting best lock
    // @Values: 0:Disabled,1:Use the better lock,2:Treat GPS2 as backup
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
const uint32_t AP_GPS::_baudrates[] = {4800U, 38400U, 115200U, 57600U, 9600U, 230400U};

// initialisation blobs to send to the GPS to try to get it into the
// right mode
const char AP_GPS::_initialisation_blob[] = UBLOX_SET_BINARY MTK_SET_BINARY SIRF_SET_BINARY;
const char AP_GPS::_initialisation_raw_blob[] = UBLOX_SET_BINARY_RAW_BAUD MTK_SET_BINARY SIRF_SET_BINARY;

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
    if (_port[instance] == NULL) {
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
    AP_GPS_Backend *new_gps = NULL;
    struct detect_state *dstate = &detect_state[instance];
    uint32_t now = AP_HAL::millis();

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    if (_type[instance] == GPS_TYPE_PX4) {
        // check for explicitely chosen PX4 GPS beforehand
        // it is not possible to autodetect it, nor does it require a real UART
        _broadcast_gps_type("PX4", instance, -1); // baud rate isn't valid
        new_gps = new AP_GPS_PX4(*this, state[instance], _port[instance]);
        goto found_gps;
    }
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_QURT
    if (_type[instance] == GPS_TYPE_QURT) {
        _broadcast_gps_type("QURTGPS", instance, -1); // baud rate isn't valid
        new_gps = new AP_GPS_QURT(*this, state[instance], _port[instance]);
        goto found_gps;
    }
#endif
    
    if (_port[instance] == NULL) {
        // UART not available
        return;
    }

    state[instance].instance = instance;
    state[instance].status = NO_GPS;
    state[instance].hdop = 9999;

	// by default the sbf/trimble gps outputs no data on its port, until configured.
	if (_type[instance] == GPS_TYPE_SBF) {
		_broadcast_gps_type("SBF", instance, -1); // baud rate isn't valid
		new_gps = new AP_GPS_SBF(*this, state[instance], _port[instance]);
	} else if ((_type[instance] == GPS_TYPE_GSOF)) {
		_broadcast_gps_type("GSOF", instance, -1); // baud rate isn't valid
		new_gps = new AP_GPS_GSOF(*this, state[instance], _port[instance]);
	}

    // record the time when we started detection. This is used to try
    // to avoid initialising a uBlox as a NMEA GPS
    if (dstate->detect_started_ms == 0) {
        dstate->detect_started_ms = now;
    }

    if (now - dstate->last_baud_change_ms > GPS_BAUD_TIME_MS) {
        // try the next baud rate
		dstate->last_baud++;
		if (dstate->last_baud == ARRAY_SIZE(_baudrates)) {
			dstate->last_baud = 0;
		}
		uint32_t baudrate = _baudrates[dstate->last_baud];
		_port[instance]->begin(baudrate);
		_port[instance]->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
		dstate->last_baud_change_ms = now;
#if UBLOX_RXM_RAW_LOGGING
    if(_raw_data != 0)
        send_blob_start(instance, _initialisation_raw_blob, sizeof(_initialisation_raw_blob));
    else
#endif
        send_blob_start(instance, _initialisation_blob, sizeof(_initialisation_blob));
    }

    send_blob_update(instance);

    while (initblob_state[instance].remaining == 0 && _port[instance]->available() > 0
            && new_gps == NULL) {
        uint8_t data = _port[instance]->read();
        /*
          running a uBlox at less than 38400 will lead to packet
          corruption, as we can't receive the packets in the 200ms
          window for 5Hz fixes. The NMEA startup message should force
          the uBlox into 38400 no matter what rate it is configured
          for.
        */
        if ((_type[instance] == GPS_TYPE_AUTO || _type[instance] == GPS_TYPE_UBLOX) &&
            _baudrates[dstate->last_baud] >= 38400 &&
            AP_GPS_UBLOX::_detect(dstate->ublox_detect_state, data)) {
            _broadcast_gps_type("u-blox", instance, dstate->last_baud);
            new_gps = new AP_GPS_UBLOX(*this, state[instance], _port[instance]);
        } 
		else if ((_type[instance] == GPS_TYPE_AUTO || _type[instance] == GPS_TYPE_MTK19) &&
                 AP_GPS_MTK19::_detect(dstate->mtk19_detect_state, data)) {
			_broadcast_gps_type("MTK19", instance, dstate->last_baud);
			new_gps = new AP_GPS_MTK19(*this, state[instance], _port[instance]);
		} 
		else if ((_type[instance] == GPS_TYPE_AUTO || _type[instance] == GPS_TYPE_MTK) &&
                 AP_GPS_MTK::_detect(dstate->mtk_detect_state, data)) {
			_broadcast_gps_type("MTK", instance, dstate->last_baud);
			new_gps = new AP_GPS_MTK(*this, state[instance], _port[instance]);
		}
        else if ((_type[instance] == GPS_TYPE_AUTO || _type[instance] == GPS_TYPE_SBP) &&
                 AP_GPS_SBP::_detect(dstate->sbp_detect_state, data)) {
            _broadcast_gps_type("SBP", instance, dstate->last_baud);
            new_gps = new AP_GPS_SBP(*this, state[instance], _port[instance]);
        }
		// save a bit of code space on a 1280
		else if ((_type[instance] == GPS_TYPE_AUTO || _type[instance] == GPS_TYPE_SIRF) &&
                 AP_GPS_SIRF::_detect(dstate->sirf_detect_state, data)) {
			_broadcast_gps_type("SIRF", instance, dstate->last_baud);
			new_gps = new AP_GPS_SIRF(*this, state[instance], _port[instance]);
		}
        else if ((_type[instance] == GPS_TYPE_AUTO || _type[instance] == GPS_TYPE_ERB) &&
                 AP_GPS_ERB::_detect(dstate->erb_detect_state, data)) {
            _broadcast_gps_type("ERB", instance, dstate->last_baud);
            new_gps = new AP_GPS_ERB(*this, state[instance], _port[instance]);
        }
		else if (now - dstate->detect_started_ms > (ARRAY_SIZE(_baudrates) * GPS_BAUD_TIME_MS)) {
			// prevent false detection of NMEA mode in
			// a MTK or UBLOX which has booted in NMEA mode
			if ((_type[instance] == GPS_TYPE_AUTO || _type[instance] == GPS_TYPE_NMEA) &&
                AP_GPS_NMEA::_detect(dstate->nmea_detect_state, data)) {
				_broadcast_gps_type("NMEA", instance, dstate->last_baud);
				new_gps = new AP_GPS_NMEA(*this, state[instance], _port[instance]);
			}
		}
	}

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_QURT
found_gps:
#endif
	if (new_gps != NULL) {
        state[instance].status = NO_FIX;
        drivers[instance] = new_gps;
        timing[instance].last_message_time_ms = now;
	}
}

AP_GPS::GPS_Status 
AP_GPS::highest_supported_status(uint8_t instance) const
{
    if (drivers[instance] != NULL)
        return drivers[instance]->highest_supported_status();
    return AP_GPS::GPS_OK_FIX_3D;
}

AP_GPS::GPS_Status 
AP_GPS::highest_supported_status(void) const
{
    if (drivers[primary_instance] != NULL)
        return drivers[primary_instance]->highest_supported_status();
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

    if (drivers[instance] == NULL || state[instance].status == NO_GPS) {
        // we don't yet know the GPS type of this one, or it has timed
        // out and needs to be re-initialised
        detect_instance(instance);
        return;
    }

    send_blob_update(instance);

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
            drivers[instance] = NULL;
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
    uint32_t now = AP_HAL::millis();

    for (uint8_t i=0; i<GPS_MAX_INSTANCES; i++) {
        update_instance(i);
    }

    // work out how many sensors we have
    uint8_t num_instances_temp = 0;
    if (state[0].status != NO_GPS) {
        num_instances_temp = 1;
    }
    if (state[1].status != NO_GPS) {
        num_instances_temp = 2;
    }
    num_instances = num_instances_temp;

    // work out which GPS is the primary
    if (now - _last_instance_swap_ms > 20000 ||
            state[primary_instance].status < GPS_OK_FIX_3D) {
        // don't allow switches any faster than every 20 seconds except if primary is in trouble

        switch (_auto_switch) {
        default:
        case GPS_AUTO_SWITCH_DISABLED:
            // auto-switch disabled, always use GPS1 as primary
            primary_instance = 0;
            break;

        case GPS_AUTO_SWITCH_BETTER_LOCK:
            if (state[0].status > state[1].status && (state[0].num_sats >= state[1].num_sats + 2)) {
                // there is a higher status lock or 2+ sats, change GPS
                primary_instance_change_request = 0;
            } else if (state[1].status > state[0].status && (state[1].num_sats >= state[0].num_sats + 2)) {
                // there is a higher status lock or 2+ sats, change GPS
                primary_instance_change_request = 1;
            }
            break;

        case GPS_AUTO_SWITCH_GPS2_AS_BACKUP:
            if (state[0].status >= GPS_OK_FIX_3D || // GPS1 has a lock
                    state[0].status >= state[1].status || // GPS1 lock >= GPS2 lock
                    timing[0].last_fix_time_ms == 0) { // GPS1 has never locked before. GPS2 is an in-flight backup, not pre-flight backup.
                primary_instance_change_request = 0;
            } else {
                primary_instance_change_request = 1;
            }
            break;
        } // switch

        // if primary index changed note the time to inhibit doing this too often with a 1000ms debounce
        if (primary_instance_change_request != primary_instance) {
            // to protect against glitches, require the instance change request to persist for 1000ms
            if (primary_instance_change_debounce_ms == 0) {
                primary_instance_change_debounce_ms = now;
            } else if (now - primary_instance_change_debounce_ms >= 1000) {
                // a change request has persisted, do the swap
                _last_instance_swap_ms = now;
                primary_instance = primary_instance_change_request;
                primary_instance_change_debounce_ms = 0;
            }
        } else {
            primary_instance_change_debounce_ms = 0;
        }
    } // if >10s


	// update notify with gps status. We always base this on the primary_instance
    AP_Notify::flags.gps_status = state[primary_instance].status;
    AP_Notify::flags.gps_num_sats = state[primary_instance].num_sats;
}

/*
  set HIL (hardware in the loop) status for a GPS instance
 */
void 
AP_GPS::setHIL(uint8_t instance, GPS_Status _status, uint64_t time_epoch_ms, 
               const Location &_location, const Vector3f &_velocity, uint8_t _num_sats, 
               uint16_t hdop, bool _have_vertical_velocity)
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
    istate.ground_speed = pythagorous2(istate.velocity.x, istate.velocity.y);
    istate.ground_course_cd = wrap_360_cd(degrees(atan2f(istate.velocity.y, istate.velocity.x)) * 100UL);
    istate.hdop = hdop;
    istate.num_sats = _num_sats;
    istate.have_vertical_velocity |= _have_vertical_velocity;
    istate.last_gps_time_ms = tnow;
    uint64_t gps_time_ms = time_epoch_ms - (17000ULL*86400ULL + 52*10*7000ULL*86400ULL - 15000ULL);
    istate.time_week     = gps_time_ms / (86400*7*(uint64_t)1000);
    istate.time_week_ms  = gps_time_ms - istate.time_week*(86400*7*(uint64_t)1000);
    timing[instance].last_message_time_ms = tnow;
    timing[instance].last_fix_time_ms = tnow;
    _type[instance].set(GPS_TYPE_HIL);
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
    if (instance < GPS_MAX_INSTANCES && drivers[instance] != NULL)
        drivers[instance]->inject_data(data, len);
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
        ground_course_cd(0), // 1/100 degrees,
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
        ground_course_cd(1), // 1/100 degrees,
        num_sats(1),
        0,
        0);
}

void 
AP_GPS::send_mavlink_gps_rtk(mavlink_channel_t chan)
{
    if (drivers[0] != NULL && drivers[0]->highest_supported_status() > AP_GPS::GPS_OK_FIX_3D) {
        drivers[0]->send_mavlink_gps_rtk(chan);
    }
}

void 
AP_GPS::send_mavlink_gps2_rtk(mavlink_channel_t chan)
{
    if (drivers[1] != NULL && drivers[1]->highest_supported_status() > AP_GPS::GPS_OK_FIX_3D) {
        drivers[1]->send_mavlink_gps_rtk(chan);
    }
}

uint8_t
AP_GPS::first_unconfigured_gps(void) const
{
    for(int i = 0; i < GPS_MAX_INSTANCES; i++) {
        if(_type[i] != GPS_TYPE_NONE && (drivers[i] == NULL || !drivers[i]->is_configured())) {
            return i;
        }
    }
    return GPS_ALL_CONFIGURED;
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
