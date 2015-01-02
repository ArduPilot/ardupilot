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

#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_HAL.h>
#include <AP_Notify.h>
#include <AP_GPS.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_GPS::var_info[] PROGMEM = {
    // @Param: TYPE
    // @DisplayName: GPS type
    // @Description: GPS type
    // @Values: 0:None,1:AUTO,2:uBlox,3:MTK,4:MTK19,5:NMEA,6:SiRF,7:HIL,8:SwiftNav,9:PX4EXPERIMENTAL
    AP_GROUPINFO("TYPE",    0, AP_GPS, _type[0], 1),

#if GPS_MAX_INSTANCES > 1

    // @Param: TYPE2
    // @DisplayName: 2nd GPS type
    // @Description: GPS type of 2nd GPS
    // @Values: 0:None,1:AUTO,2:uBlox,3:MTK,4:MTK19,5:NMEA,6:SiRF,7:HIL,8:SwiftNav,9:PX4EXPERIMENTAL
    AP_GROUPINFO("TYPE2",   1, AP_GPS, _type[1], 0),

#endif

    // @Param: NAVFILTER
    // @DisplayName: Navigation filter setting
    // @Description: Navigation filter engine setting
    // @Values: 0:Portable,2:Stationary,3:Pedestrian,4:Automotive,5:Sea,6:Airborne1G,7:Airborne2G,8:Airborne4G
    AP_GROUPINFO("NAVFILTER", 2, AP_GPS, _navfilter, GPS_ENGINE_AIRBORNE_4G),

#if GPS_MAX_INSTANCES > 1
    // @Param: AUTO_SWITCH
    // @DisplayName: Automatic Switchover Setting
    // @Description: Automatic switchover to GPS reporting best lock
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("AUTO_SWITCH", 3, AP_GPS, _auto_switch, 1),
#endif

#if GPS_RTK_AVAILABLE
    // @Param: DGPS_MIN_LOCK
    // @DisplayName: Minimum Lock Type Accepted for DGPS
    // @Description: Sets the minimum type of differential GPS corrections required before allowing to switch into DGPS mode.
    // @Values: 0:Any,50:FloatRTK,100:IntegerRTK
    // @User: Advanced
    AP_GROUPINFO("MIN_DGPS", 4, AP_GPS, _min_dgps, 100),
#endif

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

    AP_GROUPEND
};

/// Startup initialisation.
void AP_GPS::init(DataFlash_Class *dataflash)
{
    _DataFlash = dataflash;
    hal.uartB->begin(38400UL, 256, 16);
    primary_instance = 0;
#if GPS_MAX_INSTANCES > 1
    if (hal.uartE != NULL) {
        hal.uartE->begin(38400UL, 256, 16);        
    }
#endif
}

// baudrates to try to detect GPSes with
const uint32_t AP_GPS::_baudrates[] PROGMEM = {4800U, 38400U, 115200U, 57600U, 9600U};

// initialisation blobs to send to the GPS to try to get it into the
// right mode
const prog_char AP_GPS::_initialisation_blob[] PROGMEM = UBLOX_SET_BINARY MTK_SET_BINARY SIRF_SET_BINARY;

/*
  send some more initialisation string bytes if there is room in the
  UART transmit buffer
 */
void AP_GPS::send_blob_start(uint8_t instance, const prog_char *_blob, uint16_t size)
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
    // see if we can write some more of the initialisation blob
    if (initblob_state[instance].remaining > 0) {
        AP_HAL::UARTDriver *port = instance==0?hal.uartB:hal.uartE;
        int16_t space = port->txspace();
        if (space > (int16_t)initblob_state[instance].remaining) {
            space = initblob_state[instance].remaining;
        }
        while (space > 0) {
            port->write(pgm_read_byte(initblob_state[instance].blob));
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
    AP_HAL::UARTDriver *port = instance==0?hal.uartB:hal.uartE;
    struct detect_state *dstate = &detect_state[instance];
    uint32_t now = hal.scheduler->millis();

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    if (_type[instance] == GPS_TYPE_PX4) {
        // check for explicitely chosen PX4 GPS beforehand
        // it is not possible to autodetect it, nor does it require a real UART
        hal.console->print_P(PSTR(" PX4 "));
        new_gps = new AP_GPS_PX4(*this, state[instance], port);
        goto found_gps;
    }
#endif

    if (port == NULL) {
        // UART not available
        return;
    }

    state[instance].instance = instance;
    state[instance].status = NO_GPS;

    // record the time when we started detection. This is used to try
    // to avoid initialising a uBlox as a NMEA GPS
    if (dstate->detect_started_ms == 0) {
        dstate->detect_started_ms = now;
    }

    if (now - dstate->last_baud_change_ms > 1200) {
        // try the next baud rate
		dstate->last_baud++;
		if (dstate->last_baud == sizeof(_baudrates) / sizeof(_baudrates[0])) {
			dstate->last_baud = 0;
		}
		uint32_t baudrate = pgm_read_dword(&_baudrates[dstate->last_baud]);
		port->begin(baudrate, 256, 16);		
		dstate->last_baud_change_ms = now;
        send_blob_start(instance, _initialisation_blob, sizeof(_initialisation_blob));
    }

    send_blob_update(instance);

    while (port->available() > 0 && new_gps == NULL) {
        uint8_t data = port->read();
        /*
          running a uBlox at less than 38400 will lead to packet
          corruption, as we can't receive the packets in the 200ms
          window for 5Hz fixes. The NMEA startup message should force
          the uBlox into 38400 no matter what rate it is configured
          for.
        */
        if ((_type[instance] == GPS_TYPE_AUTO || _type[instance] == GPS_TYPE_UBLOX) &&
            pgm_read_dword(&_baudrates[dstate->last_baud]) >= 38400 && 
            AP_GPS_UBLOX::_detect(dstate->ublox_detect_state, data)) {
            hal.console->print_P(PSTR(" ublox "));
            new_gps = new AP_GPS_UBLOX(*this, state[instance], port);
        } 
		else if ((_type[instance] == GPS_TYPE_AUTO || _type[instance] == GPS_TYPE_MTK19) &&
                 AP_GPS_MTK19::_detect(dstate->mtk19_detect_state, data)) {
			hal.console->print_P(PSTR(" MTK19 "));
			new_gps = new AP_GPS_MTK19(*this, state[instance], port);
		} 
		else if ((_type[instance] == GPS_TYPE_AUTO || _type[instance] == GPS_TYPE_MTK) &&
                 AP_GPS_MTK::_detect(dstate->mtk_detect_state, data)) {
			hal.console->print_P(PSTR(" MTK "));
			new_gps = new AP_GPS_MTK(*this, state[instance], port);
		}
#if GPS_RTK_AVAILABLE
        else if ((_type[instance] == GPS_TYPE_AUTO || _type[instance] == GPS_TYPE_SBP) &&
                 AP_GPS_SBP::_detect(dstate->sbp_detect_state, data)) {
            hal.console->print_P(PSTR(" SBP "));
            new_gps = new AP_GPS_SBP(*this, state[instance], port);
        }
#endif // HAL_CPU_CLASS
#if !defined(GPS_SKIP_SIRF_NMEA)
		// save a bit of code space on a 1280
		else if ((_type[instance] == GPS_TYPE_AUTO || _type[instance] == GPS_TYPE_SIRF) &&
                 AP_GPS_SIRF::_detect(dstate->sirf_detect_state, data)) {
			hal.console->print_P(PSTR(" SIRF "));
			new_gps = new AP_GPS_SIRF(*this, state[instance], port);
		}
		else if (now - dstate->detect_started_ms > 5000) {
			// prevent false detection of NMEA mode in
			// a MTK or UBLOX which has booted in NMEA mode
			if ((_type[instance] == GPS_TYPE_AUTO || _type[instance] == GPS_TYPE_NMEA) &&
                AP_GPS_NMEA::_detect(dstate->nmea_detect_state, data)) {
				hal.console->print_P(PSTR(" NMEA "));
				new_gps = new AP_GPS_NMEA(*this, state[instance], port);
			}
		}
#endif
	}

found_gps:
	if (new_gps != NULL) {
        state[instance].status = NO_FIX;
        drivers[instance] = new_gps;
        timing[instance].last_message_time_ms = now;
	}
}

bool 
AP_GPS::can_calculate_base_pos(void)
{
#if GPS_RTK_AVAILABLE
    for (uint8_t i=0; i<GPS_MAX_INSTANCES; i++) {
        if (drivers[i] != NULL && drivers[i]->can_calculate_base_pos()) {
            return true;
        }
    }
#endif
    return false;
}

/*
    Tells the underlying GPS drivers to capture its current position as home.
 */
void 
AP_GPS::calculate_base_pos(void) 
{
#if GPS_RTK_AVAILABLE
    for (uint8_t i = 0; i<GPS_MAX_INSTANCES; i++) {
        if (drivers[i] != NULL && drivers[i]->can_calculate_base_pos()) {
            drivers[i]->calculate_base_pos(); 
        }
    }
#endif
}

AP_GPS::GPS_Status 
AP_GPS::highest_supported_status(uint8_t instance) const
{
#if GPS_RTK_AVAILABLE
    if (drivers[instance] != NULL)
        return drivers[instance]->highest_supported_status();
#endif
    return AP_GPS::GPS_OK_FIX_3D;
}

AP_GPS::GPS_Status 
AP_GPS::highest_supported_status(void) const
{
#if GPS_RTK_AVAILABLE

    if (drivers[primary_instance] != NULL)
        return drivers[primary_instance]->highest_supported_status();
#endif
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
    uint32_t tnow = hal.scheduler->millis();

    // if we did not get a message, and the idle timer of 1.2 seconds
    // has expired, re-initialise the GPS. This will cause GPS
    // detection to run again
    if (!result) {
        if (tnow - timing[instance].last_message_time_ms > 1200) {
            // free the driver before we run the next detection, so we
            // don't end up with two allocated at any time
            delete drivers[instance];
            drivers[instance] = NULL;
            memset(&state[instance], 0, sizeof(state[instance]));
            state[instance].instance = instance;
            state[instance].status = NO_GPS;
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

#if GPS_MAX_INSTANCES > 1
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
            if (state[i].status == state[primary_instance].status &&
                state[i].num_sats >= state[primary_instance].num_sats + 2) {
                // this GPS has at least 2 more satellites than the
                // current primary, switch primary. Once we switch we will
                // then tend to stick to the new GPS as primary. We don't
                // want to switch too often as it will look like a
                // position shift to the controllers.
                primary_instance = i;
            }
        } else {
            primary_instance = 0;
        }
    }
#else
    num_instances = 1;
#endif // GPS_MAX_INSTANCES
	// update notify with gps status. We always base this on the primary_instance
    AP_Notify::flags.gps_status = state[primary_instance].status;
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
    uint32_t tnow = hal.scheduler->millis();
    GPS_State &istate = state[instance];
    istate.status = _status;
    istate.location = _location;
    istate.location.options = 0;
    istate.velocity = _velocity;
    istate.have_vertical_velocity = true;
    istate.ground_speed = pythagorous2(istate.velocity.x, istate.velocity.y);
    istate.ground_course_cd = degrees(atan2f(istate.velocity.y, istate.velocity.x)) * 100UL;
    istate.hdop = hdop;
    istate.num_sats = _num_sats;
    istate.have_vertical_velocity = _have_vertical_velocity;
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

void 
AP_GPS::send_mavlink_gps_raw(mavlink_channel_t chan)
{
    static uint32_t last_send_time_ms;
    if (status(0) > AP_GPS::NO_GPS) {
        // when we have a GPS then only send new data
        if (last_send_time_ms == last_message_time_ms(0)) {
            return;
        }
        last_send_time_ms = last_message_time_ms(0);
    } else {
        // when we don't have a GPS then send at 1Hz
        uint32_t now = hal.scheduler->millis();
        if (now - last_send_time_ms < 1000) {
            return;
        }
        last_send_time_ms = now;
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
        65535,
        ground_speed(0)*100,  // cm/s
        ground_course_cd(0), // 1/100 degrees,
        num_sats(0));
}

#if GPS_MAX_INSTANCES > 1
void 
AP_GPS::send_mavlink_gps2_raw(mavlink_channel_t chan)
{
    static uint32_t last_send_time_ms;
    if (num_sensors() < 2 || status(1) <= AP_GPS::NO_GPS) {
        return;
    }
    // when we have a GPS then only send new data
    if (last_send_time_ms == last_message_time_ms(1)) {
        return;
    }
    last_send_time_ms = last_message_time_ms(1);

    const Location &loc = location(1);
    mavlink_msg_gps2_raw_send(
        chan,
        last_fix_time_ms(1)*(uint64_t)1000,
        status(1),
        loc.lat,
        loc.lng,
        loc.alt * 10UL,
        get_hdop(1),
        65535,
        ground_speed(1)*100,  // cm/s
        ground_course_cd(1), // 1/100 degrees,
        num_sats(1),
        0,
        0);
}
#endif

#if GPS_RTK_AVAILABLE
void 
AP_GPS::send_mavlink_gps_rtk(mavlink_channel_t chan)
{
    if (drivers[0] != NULL && drivers[0]->highest_supported_status() > AP_GPS::GPS_OK_FIX_3D) {
        drivers[0]->send_mavlink_gps_rtk(chan);
    }
}

#if GPS_MAX_INSTANCES > 1
void 
AP_GPS::send_mavlink_gps2_rtk(mavlink_channel_t chan)
{
    if (drivers[1] != NULL && drivers[1]->highest_supported_status() > AP_GPS::GPS_OK_FIX_3D) {
        drivers[1]->send_mavlink_gps_rtk(chan);
    }
}
#endif
#endif
