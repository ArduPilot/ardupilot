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

//
//  u-blox GPS driver for ArduPilot
//	Origin code by Michael Smith, Jordi Munoz and Jose Julio, DIYDrones.com
//  Substantially rewritten for new GPS driver structure by Andrew Tridgell
//
#include "AP_GPS.h"
#include "AP_GPS_UBLOX.h"
#include <AP_HAL/Util.h>
#include <DataFlash/DataFlash.h>
#include <GCS_MAVLink/GCS.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BH
    #define UBLOX_SPEED_CHANGE  1
#else
    #define UBLOX_SPEED_CHANGE 0
#endif


#define UBLOX_DEBUGGING 0
#define UBLOX_FAKE_3DLOCK 0

extern const AP_HAL::HAL& hal;

#if UBLOX_DEBUGGING
 # define Debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
 # define Debug(fmt, args ...)
#endif

AP_GPS_UBLOX::AP_GPS_UBLOX(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port),
    _step(0),
    _msg_id(0),
    _payload_length(0),
    _payload_counter(0),
    _class(0),
    _cfg_saved(false),
    _last_cfg_sent_time(0),
    _num_cfg_save_tries(0),
    _last_config_time(0),
    _delay_time(0),
    _next_message(STEP_PVT),
    _ublox_port(255),
    _have_version(false),
    _unconfigured_messages(CONFIG_ALL),
    _hardware_generation(0),
    _new_position(0),
    _new_speed(0),
    _disable_counter(0),
    next_fix(AP_GPS::NO_FIX),
    _cfg_needs_save(false),
    noReceivedHdop(true),
    havePvtMsg(false)
{
    // stop any config strings that are pending
    gps.send_blob_start(state.instance, nullptr, 0);

    // start the process of updating the GPS rates
    _request_next_config();
}

void
AP_GPS_UBLOX::_request_next_config(void)
{
    // don't request config if we shouldn't configure the GPS
    if (gps._auto_config == 0) {
        return;
    }

    // Ensure there is enough space for the largest possible outgoing message
    if (port->txspace() < (int16_t)(sizeof(struct ubx_header)+sizeof(struct ubx_cfg_nav_rate)+2)) {
        // not enough space - do it next time
        return;
    }

   Debug("Unconfigured messages: %d Current message: %d\n", _unconfigured_messages, _next_message);

   // check AP_GPS_UBLOX.h for the enum that controls the order.
   // This switch statement isn't maintained against the enum in order to reduce code churn
    switch (_next_message++) {
    case STEP_PVT:
        if(!_request_message_rate(CLASS_NAV, MSG_PVT)) {
            _next_message--;
        }
        break;
    case STEP_PORT:
        _request_port();
        break;
    case STEP_POLL_SVINFO:
        // not required once we know what generation we are on
        if(_hardware_generation == 0) {
            if (!_send_message(CLASS_NAV, MSG_NAV_SVINFO, 0, 0)) {
                _next_message--;
            }
        }
        break;
    case STEP_POLL_SBAS:
        if (gps._sbas_mode != 2) {
            _send_message(CLASS_CFG, MSG_CFG_SBAS, nullptr, 0);
        } else {
            _unconfigured_messages &= ~CONFIG_SBAS;
        }
        break;
    case STEP_POLL_NAV:
        if (!_send_message(CLASS_CFG, MSG_CFG_NAV_SETTINGS, nullptr, 0)) {
            _next_message--;
        }
        break;
    case STEP_POLL_GNSS:
        if (!_send_message(CLASS_CFG, MSG_CFG_GNSS, nullptr, 0)) {
            _next_message--;
        }
        break;
    case STEP_NAV_RATE:
        if (!_send_message(CLASS_CFG, MSG_CFG_RATE, nullptr, 0)) {
            _next_message--;
        }
        break;
    case STEP_POSLLH:
        if(!_request_message_rate(CLASS_NAV, MSG_POSLLH)) {
            _next_message--;
        }
        break;
    case STEP_STATUS:
        if(!_request_message_rate(CLASS_NAV, MSG_STATUS)) {
            _next_message--;
        }
        break;
    case STEP_SOL:
        if(!_request_message_rate(CLASS_NAV, MSG_SOL)) {
            _next_message--;
        }
        break;
    case STEP_VELNED:
        if(!_request_message_rate(CLASS_NAV, MSG_VELNED)) {
            _next_message--;
        }
        break;
    case STEP_DOP:
       if(! _request_message_rate(CLASS_NAV, MSG_DOP)) {
            _next_message--;
        }
        break;
    case STEP_MON_HW:
        if(!_request_message_rate(CLASS_MON, MSG_MON_HW)) {
            _next_message--;
        }
        break;
    case STEP_MON_HW2:
        if(!_request_message_rate(CLASS_MON, MSG_MON_HW2)) {
            _next_message--;
        }
        break;
    case STEP_RAW:
#if UBLOX_RXM_RAW_LOGGING
        if(gps._raw_data == 0) {
            _unconfigured_messages &= ~CONFIG_RATE_RAW;
        } else if(!_request_message_rate(CLASS_RXM, MSG_RXM_RAW)) {
            _next_message--;
        }
#else
        _unconfigured_messages & = ~CONFIG_RATE_RAW;
#endif
        break;
    case STEP_RAWX:
#if UBLOX_RXM_RAW_LOGGING
        if(gps._raw_data == 0) {
            _unconfigured_messages &= ~CONFIG_RATE_RAW;
        } else if(!_request_message_rate(CLASS_RXM, MSG_RXM_RAWX)) {
            _next_message--;
        }
#else
        _unconfigured_messages & = ~CONFIG_RATE_RAW;
#endif
        break;
    case STEP_VERSION:
        if(!_have_version && !hal.util->get_soft_armed()) {
            _request_version();
        } else {
            _unconfigured_messages &= ~CONFIG_VERSION;
        }
        // no need to send the initial rates, move to checking only
        _next_message = STEP_PVT;
        break;
    default:
        // this case should never be reached, do a full reset if it is hit
        _next_message = STEP_PVT;
        break;
    }
}

void
AP_GPS_UBLOX::_verify_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate) {
    uint8_t desired_rate;

    switch(msg_class) {
    case CLASS_NAV:
        switch(msg_id) {
        case MSG_POSLLH:
            desired_rate = havePvtMsg ? 0 : RATE_POSLLH;
            if(rate == desired_rate) {
                _unconfigured_messages &= ~CONFIG_RATE_POSLLH;
            } else {
                _configure_message_rate(msg_class, msg_id, desired_rate);
                _unconfigured_messages |= CONFIG_RATE_POSLLH;
                _cfg_needs_save = true;
            }
            break;
        case MSG_STATUS:
            desired_rate = havePvtMsg ? 0 : RATE_STATUS;
            if(rate == desired_rate) {
                _unconfigured_messages &= ~CONFIG_RATE_STATUS;
            } else {
                _configure_message_rate(msg_class, msg_id, desired_rate);
                _unconfigured_messages |= CONFIG_RATE_STATUS;
                _cfg_needs_save = true;
            }
            break;
        case MSG_SOL:
            if(rate == RATE_SOL) {
                _unconfigured_messages &= ~CONFIG_RATE_SOL;
            } else {
                _configure_message_rate(msg_class, msg_id, RATE_SOL);
                _unconfigured_messages |= CONFIG_RATE_SOL;
                _cfg_needs_save = true;
            }
            break;
        case MSG_PVT:
            if(rate == RATE_PVT) {
                _unconfigured_messages &= ~CONFIG_RATE_PVT;
            } else {
                _configure_message_rate(msg_class, msg_id, RATE_PVT);
                _unconfigured_messages |= CONFIG_RATE_PVT;
                _cfg_needs_save = true;
            }
            break;
        case MSG_VELNED:
            desired_rate = havePvtMsg ? 0 : RATE_VELNED;
            if(rate == desired_rate) {
                _unconfigured_messages &= ~CONFIG_RATE_VELNED;
            } else {
                _configure_message_rate(msg_class, msg_id, desired_rate);
                _unconfigured_messages |= CONFIG_RATE_VELNED;
                _cfg_needs_save = true;
            }
            break;
        case MSG_DOP:
            if(rate == RATE_DOP) {
                _unconfigured_messages &= ~CONFIG_RATE_DOP;
            } else {
                _configure_message_rate(msg_class, msg_id, RATE_DOP);
                _unconfigured_messages |= CONFIG_RATE_DOP;
                _cfg_needs_save = true;
            }
            break;
        }
        break;
    case CLASS_MON:
        switch(msg_id) {
        case MSG_MON_HW:
            if(rate == RATE_HW) {
                _unconfigured_messages &= ~CONFIG_RATE_MON_HW;
            } else {
                _configure_message_rate(msg_class, msg_id, RATE_HW);
                _unconfigured_messages |= CONFIG_RATE_MON_HW;
                _cfg_needs_save = true;
            }
            break;
        case MSG_MON_HW2:
            if(rate == RATE_HW2) {
                _unconfigured_messages &= ~CONFIG_RATE_MON_HW2;
            } else {
                _configure_message_rate(msg_class, msg_id, RATE_HW2);
                _unconfigured_messages |= CONFIG_RATE_MON_HW2;
                _cfg_needs_save = true;
            }
            break;
        }
        break;
#if UBLOX_RXM_RAW_LOGGING
    case CLASS_RXM:
        switch(msg_id) {
        case MSG_RXM_RAW:
            if(rate == gps._raw_data) {
                _unconfigured_messages &= ~CONFIG_RATE_RAW;
            } else {
                _configure_message_rate(msg_class, msg_id, gps._raw_data);
                _unconfigured_messages |= CONFIG_RATE_RAW;
                _cfg_needs_save = true;
            }
            break;
        case MSG_RXM_RAWX:
            if(rate == gps._raw_data) {
                _unconfigured_messages &= ~CONFIG_RATE_RAW;
            } else {
                _configure_message_rate(msg_class, msg_id, gps._raw_data);
                _unconfigured_messages |= CONFIG_RATE_RAW;
                _cfg_needs_save = true;
            }
            break;
        }
        break;
#endif // UBLOX_RXM_RAW_LOGGING
    }
}

// Requests the ublox driver to identify what port we are using to communicate
void
AP_GPS_UBLOX::_request_port(void)
{
    if (port->txspace() < (int16_t)(sizeof(struct ubx_header)+2)) {
        // not enough space - do it next time
        return;
    }
    _send_message(CLASS_CFG, MSG_CFG_PRT, nullptr, 0);
}
    // Ensure there is enough space for the largest possible outgoing message
// Process bytes available from the stream
//
// The stream is assumed to contain only messages we recognise.  If it
// contains other messages, and those messages contain the preamble
// bytes, it is possible for this code to fail to synchronise to the
// stream immediately.  Without buffering the entire message and
// re-processing it from the top, this is unavoidable. The parser
// attempts to avoid this when possible.
//
bool
AP_GPS_UBLOX::read(void)
{
    uint8_t data;
    int16_t numc;
    bool parsed = false;
    uint32_t millis_now = AP_HAL::millis();

    // walk through the gps configuration at 1 message per second
    if (millis_now - _last_config_time >= _delay_time) {
        _request_next_config();
        _last_config_time = millis_now;
        if (_unconfigured_messages) { // send the updates faster until fully configured
            if (!havePvtMsg && (_unconfigured_messages & CONFIG_REQUIRED_INITIAL)) {
                _delay_time = 300;
            } else {
                _delay_time = 750;
            }
        } else {
            _delay_time = 2000;
        }
    }

    if(!_unconfigured_messages && gps._save_config && !_cfg_saved &&
       _num_cfg_save_tries < 5 && (millis_now - _last_cfg_sent_time) > 5000 &&
       !hal.util->get_soft_armed()) {
        //save the configuration sent until now
        if (gps._save_config == 1 ||
            (gps._save_config == 2 && _cfg_needs_save)) {
            _save_cfg();
        }
    }

    numc = port->available();
    for (int16_t i = 0; i < numc; i++) {        // Process bytes received

        // read the next byte
        data = port->read();

	reset:
        switch(_step) {

        // Message preamble detection
        //
        // If we fail to match any of the expected bytes, we reset
        // the state machine and re-consider the failed byte as
        // the first byte of the preamble.  This improves our
        // chances of recovering from a mismatch and makes it less
        // likely that we will be fooled by the preamble appearing
        // as data in some other message.
        //
        case 1:
            if (PREAMBLE2 == data) {
                _step++;
                break;
            }
            _step = 0;
            Debug("reset %u", __LINE__);
            /* no break */
        case 0:
            if(PREAMBLE1 == data)
                _step++;
            break;

        // Message header processing
        //
        // We sniff the class and message ID to decide whether we
        // are going to gather the message bytes or just discard
        // them.
        //
        // We always collect the length so that we can avoid being
        // fooled by preamble bytes in messages.
        //
        case 2:
            _step++;
            _class = data;
            _ck_b = _ck_a = data;                               // reset the checksum accumulators
            break;
        case 3:
            _step++;
            _ck_b += (_ck_a += data);                   // checksum byte
            _msg_id = data;
            break;
        case 4:
            _step++;
            _ck_b += (_ck_a += data);                   // checksum byte
            _payload_length = data;                             // payload length low byte
            break;
        case 5:
            _step++;
            _ck_b += (_ck_a += data);                   // checksum byte

            _payload_length += (uint16_t)(data<<8);
            if (_payload_length > sizeof(_buffer)) {
                Debug("large payload %u", (unsigned)_payload_length);
                // assume any payload bigger then what we know about is noise
                _payload_length = 0;
                _step = 0;
				goto reset;
            }
            _payload_counter = 0;                               // prepare to receive payload
            break;

        // Receive message data
        //
        case 6:
            _ck_b += (_ck_a += data);                   // checksum byte
            if (_payload_counter < sizeof(_buffer)) {
                _buffer[_payload_counter] = data;
            }
            if (++_payload_counter == _payload_length)
                _step++;
            break;

        // Checksum and message processing
        //
        case 7:
            _step++;
            if (_ck_a != data) {
                Debug("bad cka %x should be %x", data, _ck_a);
                _step = 0;
				goto reset;
            }
            break;
        case 8:
            _step = 0;
            if (_ck_b != data) {
                Debug("bad ckb %x should be %x", data, _ck_b);
                break;                                                  // bad checksum
            }

            if (_parse_gps()) {
                parsed = true;
            }
            break;
        }
    }
    return parsed;
}

// Private Methods /////////////////////////////////////////////////////////////
void AP_GPS_UBLOX::log_mon_hw(void)
{
    if (gps._DataFlash == nullptr || !gps._DataFlash->logging_started()) {
        return;
    }
    struct log_Ubx1 pkt = {
        LOG_PACKET_HEADER_INIT(_ubx_msg_log_index(LOG_GPS_UBX1_MSG)),
        time_us    : AP_HAL::micros64(),
        instance   : state.instance,
        noisePerMS : _buffer.mon_hw_60.noisePerMS,
        jamInd     : _buffer.mon_hw_60.jamInd,
        aPower     : _buffer.mon_hw_60.aPower,
        agcCnt     : _buffer.mon_hw_60.agcCnt,
    };
    if (_payload_length == 68) {
        pkt.noisePerMS = _buffer.mon_hw_68.noisePerMS;
        pkt.jamInd     = _buffer.mon_hw_68.jamInd;
        pkt.aPower     = _buffer.mon_hw_68.aPower;
        pkt.agcCnt     = _buffer.mon_hw_68.agcCnt;
    }
    gps._DataFlash->WriteBlock(&pkt, sizeof(pkt));
}

void AP_GPS_UBLOX::log_mon_hw2(void)
{
    if (gps._DataFlash == nullptr || !gps._DataFlash->logging_started()) {
        return;
    }

    struct log_Ubx2 pkt = {
        LOG_PACKET_HEADER_INIT(_ubx_msg_log_index(LOG_GPS_UBX2_MSG)),
        time_us   : AP_HAL::micros64(),
        instance  : state.instance,
        ofsI      : _buffer.mon_hw2.ofsI,
        magI      : _buffer.mon_hw2.magI,
        ofsQ      : _buffer.mon_hw2.ofsQ,
        magQ      : _buffer.mon_hw2.magQ,
    };
    gps._DataFlash->WriteBlock(&pkt, sizeof(pkt));
}

#if UBLOX_RXM_RAW_LOGGING
void AP_GPS_UBLOX::log_rxm_raw(const struct ubx_rxm_raw &raw)
{
    if (gps._DataFlash == nullptr || !gps._DataFlash->logging_started()) {
        return;
    }
    uint64_t now = AP_HAL::micros64();
    for (uint8_t i=0; i<raw.numSV; i++) {
        struct log_GPS_RAW pkt = {
            LOG_PACKET_HEADER_INIT(LOG_GPS_RAW_MSG),
            time_us    : now,
            iTOW       : raw.iTOW,
            week       : raw.week,
            numSV      : raw.numSV,
            sv         : raw.svinfo[i].sv,
            cpMes      : raw.svinfo[i].cpMes,
            prMes      : raw.svinfo[i].prMes,
            doMes      : raw.svinfo[i].doMes,
            mesQI      : raw.svinfo[i].mesQI,
            cno        : raw.svinfo[i].cno,
            lli        : raw.svinfo[i].lli
        };
        gps._DataFlash->WriteBlock(&pkt, sizeof(pkt));
    }
}

void AP_GPS_UBLOX::log_rxm_rawx(const struct ubx_rxm_rawx &raw)
{
    if (gps._DataFlash == nullptr || !gps._DataFlash->logging_started()) {
        return;
    }
    uint64_t now = AP_HAL::micros64();

    struct log_GPS_RAWH header = {
        LOG_PACKET_HEADER_INIT(LOG_GPS_RAWH_MSG),
        time_us    : now,
        rcvTow     : raw.rcvTow,
        week       : raw.week,
        leapS      : raw.leapS,
        numMeas    : raw.numMeas,
        recStat    : raw.recStat
    };
    gps._DataFlash->WriteBlock(&header, sizeof(header));

    for (uint8_t i=0; i<raw.numMeas; i++) {
        struct log_GPS_RAWS pkt = {
            LOG_PACKET_HEADER_INIT(LOG_GPS_RAWS_MSG),
            time_us    : now,
            prMes      : raw.svinfo[i].prMes,
            cpMes      : raw.svinfo[i].cpMes,
            doMes      : raw.svinfo[i].doMes,
            gnssId     : raw.svinfo[i].gnssId,
            svId       : raw.svinfo[i].svId,
            freqId     : raw.svinfo[i].freqId,
            locktime   : raw.svinfo[i].locktime,
            cno        : raw.svinfo[i].cno,
            prStdev    : raw.svinfo[i].prStdev,
            cpStdev    : raw.svinfo[i].cpStdev,
            doStdev    : raw.svinfo[i].doStdev,
            trkStat    : raw.svinfo[i].trkStat
        };
        gps._DataFlash->WriteBlock(&pkt, sizeof(pkt));
    }
}
#endif // UBLOX_RXM_RAW_LOGGING

void AP_GPS_UBLOX::unexpected_message(void)
{
    Debug("Unexpected message 0x%02x 0x%02x", (unsigned)_class, (unsigned)_msg_id);
    if (++_disable_counter == 0) {
        // disable future sends of this message id, but
        // only do this every 256 messages, as some
        // message types can't be disabled and we don't
        // want to get into an ack war
        Debug("Disabling message 0x%02x 0x%02x", (unsigned)_class, (unsigned)_msg_id);
        _configure_message_rate(_class, _msg_id, 0);
    }
}

bool
AP_GPS_UBLOX::_parse_gps(void)
{
    if (_class == CLASS_ACK) {
        Debug("ACK %u", (unsigned)_msg_id);

        if(_msg_id == MSG_ACK_ACK) {
            switch(_buffer.ack.clsID) {
            case CLASS_CFG:
                switch(_buffer.ack.msgID) {
                case MSG_CFG_CFG:
                    _cfg_saved = true;
                    _cfg_needs_save = false;
                    break;
                case MSG_CFG_GNSS:
                    _unconfigured_messages &= ~CONFIG_GNSS;
                    break;
                case MSG_CFG_MSG:
                    // There is no way to know what MSG config was ack'ed, assume it was the last
                    // one requested. To verify it rerequest the last config we sent. If we miss
                    // the actual ack we will catch it next time through the poll loop, but that
                    // will be a good chunk of time later.
                    break;
                case MSG_CFG_NAV_SETTINGS:
                    _unconfigured_messages &= ~CONFIG_NAV_SETTINGS;
                    break;
                case MSG_CFG_RATE:
                    // The GPS will ACK a update rate that is invalid. in order to detect this
                    // only accept the rate as configured by reading the settings back and
                   //  validating that they all match the target values
                    break;
                case MSG_CFG_SBAS:
                    _unconfigured_messages &= ~CONFIG_SBAS;
                    break;
                }
                break;
            case CLASS_MON:
                switch(_buffer.ack.msgID) {
                case MSG_MON_HW:
                    _unconfigured_messages &= ~CONFIG_RATE_MON_HW;
                    break;
                case MSG_MON_HW2:
                    _unconfigured_messages &= ~CONFIG_RATE_MON_HW2;
                    break;
                }
            }
        }
        return false;
    }

    if (_class == CLASS_CFG) {
        switch(_msg_id) {
        case  MSG_CFG_NAV_SETTINGS:
	    Debug("Got settings %u min_elev %d drLimit %u\n", 
                  (unsigned)_buffer.nav_settings.dynModel,
                  (int)_buffer.nav_settings.minElev,
                  (unsigned)_buffer.nav_settings.drLimit);
            _buffer.nav_settings.mask = 0;
            if (gps._navfilter != AP_GPS::GPS_ENGINE_NONE &&
                _buffer.nav_settings.dynModel != gps._navfilter) {
                // we've received the current nav settings, change the engine
                // settings and send them back
                Debug("Changing engine setting from %u to %u\n",
                      (unsigned)_buffer.nav_settings.dynModel, (unsigned)gps._navfilter);
                _buffer.nav_settings.dynModel = gps._navfilter;
                _buffer.nav_settings.mask |= 1;
            }
            if (gps._min_elevation != -100 &&
                _buffer.nav_settings.minElev != gps._min_elevation) {
                Debug("Changing min elevation to %d\n", (int)gps._min_elevation);
                _buffer.nav_settings.minElev = gps._min_elevation;
                _buffer.nav_settings.mask |= 2;
            }
            if (_buffer.nav_settings.mask != 0) {
                _send_message(CLASS_CFG, MSG_CFG_NAV_SETTINGS,
                              &_buffer.nav_settings,
                              sizeof(_buffer.nav_settings));
                _unconfigured_messages |= CONFIG_NAV_SETTINGS;
                _cfg_needs_save = true;
            } else {
                _unconfigured_messages &= ~CONFIG_NAV_SETTINGS;
            }
            return false;

#if UBLOX_GNSS_SETTINGS
        case MSG_CFG_GNSS:
            if (gps._gnss_mode[state.instance] != 0) {
                struct ubx_cfg_gnss start_gnss = _buffer.gnss;
                uint8_t gnssCount = 0;
                Debug("Got GNSS Settings %u %u %u %u:\n",
                    (unsigned)_buffer.gnss.msgVer,
                    (unsigned)_buffer.gnss.numTrkChHw,
                    (unsigned)_buffer.gnss.numTrkChUse,
                    (unsigned)_buffer.gnss.numConfigBlocks);
#if UBLOX_DEBUGGING
                for(int i = 0; i < _buffer.gnss.numConfigBlocks; i++) {
                    Debug("  %u %u %u 0x%08x\n",
                    (unsigned)_buffer.gnss.configBlock[i].gnssId,
                    (unsigned)_buffer.gnss.configBlock[i].resTrkCh,
                    (unsigned)_buffer.gnss.configBlock[i].maxTrkCh,
                    (unsigned)_buffer.gnss.configBlock[i].flags);
                }
#endif

                for(int i = 0; i < UBLOX_MAX_GNSS_CONFIG_BLOCKS; i++) {
                    if((gps._gnss_mode[state.instance] & (1 << i)) && i != GNSS_SBAS) {
                        gnssCount++;
                    }
                }

                for(int i = 0; i < _buffer.gnss.numConfigBlocks; i++) {
                    // Reserve an equal portion of channels for all enabled systems
                    if(gps._gnss_mode[state.instance] & (1 << _buffer.gnss.configBlock[i].gnssId)) {
                        if(GNSS_SBAS !=_buffer.gnss.configBlock[i].gnssId) {
                            _buffer.gnss.configBlock[i].resTrkCh = (_buffer.gnss.numTrkChHw - 3) / (gnssCount * 2);
                            _buffer.gnss.configBlock[i].maxTrkCh = _buffer.gnss.numTrkChHw;
                        } else {
                            _buffer.gnss.configBlock[i].resTrkCh = 1;
                            _buffer.gnss.configBlock[i].maxTrkCh = 3;
                        }
                        _buffer.gnss.configBlock[i].flags = _buffer.gnss.configBlock[i].flags | 0x00000001;
                    } else {
                        _buffer.gnss.configBlock[i].resTrkCh = 0;
                        _buffer.gnss.configBlock[i].maxTrkCh = 0;
                        _buffer.gnss.configBlock[i].flags = _buffer.gnss.configBlock[i].flags & 0xFFFFFFFE;
                    }
                }
                if (!memcmp(&start_gnss, &_buffer.gnss, sizeof(start_gnss))) {
                    _send_message(CLASS_CFG, MSG_CFG_GNSS, &_buffer.gnss, 4 + (8 * _buffer.gnss.numConfigBlocks));
                    _unconfigured_messages |= CONFIG_GNSS;
                    _cfg_needs_save = true;
                } else {
                    _unconfigured_messages &= ~CONFIG_GNSS;
                }
            } else {
                _unconfigured_messages &= ~CONFIG_GNSS;
            }
            return false;
#endif

        case MSG_CFG_SBAS:
            if (gps._sbas_mode != 2) {
	        Debug("Got SBAS settings %u %u %u 0x%x 0x%x\n", 
                      (unsigned)_buffer.sbas.mode,
                      (unsigned)_buffer.sbas.usage,
                      (unsigned)_buffer.sbas.maxSBAS,
                      (unsigned)_buffer.sbas.scanmode2,
                      (unsigned)_buffer.sbas.scanmode1);
                if (_buffer.sbas.mode != gps._sbas_mode) {
                    _buffer.sbas.mode = gps._sbas_mode;
                    _send_message(CLASS_CFG, MSG_CFG_SBAS,
                                  &_buffer.sbas,
                                  sizeof(_buffer.sbas));
                    _unconfigured_messages |= CONFIG_SBAS;
                    _cfg_needs_save = true;
                } else {
                    _unconfigured_messages &= ~CONFIG_SBAS;
                }
            } else {
                    _unconfigured_messages &= ~CONFIG_SBAS;
            }
            return false;
        case MSG_CFG_MSG:
            if(_payload_length == sizeof(ubx_cfg_msg_rate_6)) {
                // can't verify the setting without knowing the port
                // request the port again
                if(_ublox_port >= UBLOX_MAX_PORTS) {
                    _request_port();
                    return false;
                }
                _verify_rate(_buffer.msg_rate_6.msg_class, _buffer.msg_rate_6.msg_id,
                             _buffer.msg_rate_6.rates[_ublox_port]);
            } else {
                _verify_rate(_buffer.msg_rate.msg_class, _buffer.msg_rate.msg_id,
                             _buffer.msg_rate.rate);
            }
            return false;
        case MSG_CFG_PRT:
           _ublox_port = _buffer.prt.portID;
           return false;
        case MSG_CFG_RATE:
            if(_buffer.nav_rate.measure_rate_ms != gps._rate_ms[state.instance] ||
               _buffer.nav_rate.nav_rate != 1 ||
               _buffer.nav_rate.timeref != 0) {
               _configure_rate();
                _unconfigured_messages |= CONFIG_RATE_NAV;
                _cfg_needs_save = true;
            } else {
                _unconfigured_messages &= ~CONFIG_RATE_NAV;
            }
            return false;
        }
           
    }

    if (_class == CLASS_MON) {
        switch(_msg_id) {
        case MSG_MON_HW:
            if (_payload_length == 60 || _payload_length == 68) {
                log_mon_hw();
            }
            break;
        case MSG_MON_HW2:
            if (_payload_length == 28) {
                log_mon_hw2();  
            }
            break;
        case MSG_MON_VER:
            _have_version = true;
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, 
                                             "u-blox %d HW: %s SW: %s",
                                             state.instance,
                                             _buffer.mon_ver.hwVersion,
                                             _buffer.mon_ver.swVersion);
            break;
        default:
            unexpected_message();
        }
        return false;
    }

#if UBLOX_RXM_RAW_LOGGING
    if (_class == CLASS_RXM && _msg_id == MSG_RXM_RAW && gps._raw_data != 0) {
        log_rxm_raw(_buffer.rxm_raw);
        return false;
    } else if (_class == CLASS_RXM && _msg_id == MSG_RXM_RAWX && gps._raw_data != 0) {
        log_rxm_rawx(_buffer.rxm_rawx);
        return false;
    }
#endif // UBLOX_RXM_RAW_LOGGING

    if (_class != CLASS_NAV) {
        unexpected_message();
        return false;
    }

    switch (_msg_id) {
    case MSG_POSLLH:
        Debug("MSG_POSLLH next_fix=%u", next_fix);
        if (havePvtMsg) {
            _unconfigured_messages |= CONFIG_RATE_POSLLH;
            break;
        }
        _last_pos_time        = _buffer.posllh.time;
        state.location.lng    = _buffer.posllh.longitude;
        state.location.lat    = _buffer.posllh.latitude;
        state.location.alt    = _buffer.posllh.altitude_msl / 10;
        state.status          = next_fix;
        _new_position = true;
        state.horizontal_accuracy = _buffer.posllh.horizontal_accuracy*1.0e-3f;
        state.vertical_accuracy = _buffer.posllh.vertical_accuracy*1.0e-3f;
        state.have_horizontal_accuracy = true;
        state.have_vertical_accuracy = true;
#if UBLOX_FAKE_3DLOCK
        state.location.lng = 1491652300L;
        state.location.lat = -353632610L;
        state.location.alt = 58400;
        state.vertical_accuracy = 0;
        state.horizontal_accuracy = 0;
#endif
        break;
    case MSG_STATUS:
        Debug("MSG_STATUS fix_status=%u fix_type=%u",
              _buffer.status.fix_status,
              _buffer.status.fix_type);
        if (havePvtMsg) {
            _unconfigured_messages |= CONFIG_RATE_STATUS;
            break;
        }
        if (_buffer.status.fix_status & NAV_STATUS_FIX_VALID) {
            if( (_buffer.status.fix_type == AP_GPS_UBLOX::FIX_3D) &&
                (_buffer.status.fix_status & AP_GPS_UBLOX::NAV_STATUS_DGPS_USED)) {
                next_fix = AP_GPS::GPS_OK_FIX_3D_DGPS;
            }else if( _buffer.status.fix_type == AP_GPS_UBLOX::FIX_3D) {
                next_fix = AP_GPS::GPS_OK_FIX_3D;
            }else if (_buffer.status.fix_type == AP_GPS_UBLOX::FIX_2D) {
                next_fix = AP_GPS::GPS_OK_FIX_2D;
            }else{
                next_fix = AP_GPS::NO_FIX;
                state.status = AP_GPS::NO_FIX;
            }
        }else{
            next_fix = AP_GPS::NO_FIX;
            state.status = AP_GPS::NO_FIX;
        }
#if UBLOX_FAKE_3DLOCK
        state.status = AP_GPS::GPS_OK_FIX_3D;
        next_fix = state.status;
#endif
        break;
    case MSG_DOP:
        Debug("MSG_DOP");
        noReceivedHdop = false;
        state.hdop        = _buffer.dop.hDOP;
        state.vdop        = _buffer.dop.vDOP;
#if UBLOX_FAKE_3DLOCK
        state.hdop = 130;
        state.hdop = 170;
#endif
        break;
    case MSG_SOL:
        Debug("MSG_SOL fix_status=%u fix_type=%u",
              _buffer.solution.fix_status,
              _buffer.solution.fix_type);
        if (havePvtMsg) {
            state.time_week = _buffer.solution.week;
            break;
        }
        if (_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) {
            if( (_buffer.solution.fix_type == AP_GPS_UBLOX::FIX_3D) &&
                (_buffer.solution.fix_status & AP_GPS_UBLOX::NAV_STATUS_DGPS_USED)) {
                next_fix = AP_GPS::GPS_OK_FIX_3D_DGPS;
            }else if( _buffer.solution.fix_type == AP_GPS_UBLOX::FIX_3D) {
                next_fix = AP_GPS::GPS_OK_FIX_3D;
            }else if (_buffer.solution.fix_type == AP_GPS_UBLOX::FIX_2D) {
                next_fix = AP_GPS::GPS_OK_FIX_2D;
            }else{
                next_fix = AP_GPS::NO_FIX;
                state.status = AP_GPS::NO_FIX;
            }
        }else{
            next_fix = AP_GPS::NO_FIX;
            state.status = AP_GPS::NO_FIX;
        }
        if(noReceivedHdop) {
            state.hdop = _buffer.solution.position_DOP;
        }
        state.num_sats    = _buffer.solution.satellites;
        if (next_fix >= AP_GPS::GPS_OK_FIX_2D) {
            state.last_gps_time_ms = AP_HAL::millis();
            state.time_week_ms    = _buffer.solution.time;
            state.time_week       = _buffer.solution.week;
        }
#if UBLOX_FAKE_3DLOCK
        next_fix = state.status;
        state.num_sats = 10;
        state.time_week = 1721;
        state.time_week_ms = AP_HAL::millis() + 3*60*60*1000 + 37000;
        state.last_gps_time_ms = AP_HAL::millis();
        state.hdop = 130;
#endif
        break;
    case MSG_PVT:
        Debug("MSG_PVT");
        havePvtMsg = true;
        // position
        _last_pos_time        = _buffer.pvt.itow;
        state.location.lng    = _buffer.pvt.lon;
        state.location.lat    = _buffer.pvt.lat;
        state.location.alt    = _buffer.pvt.h_msl / 10;
        switch (_buffer.pvt.fix_type) 
        {
            case 0:
                state.status = AP_GPS::NO_FIX;
                break;
            case 1:
                state.status = AP_GPS::NO_FIX;
                break;
            case 2:
                state.status = AP_GPS::GPS_OK_FIX_2D;
                break;
            case 3:
                state.status = AP_GPS::GPS_OK_FIX_3D;
                if (_buffer.pvt.flags & 0b00000010)  // diffsoln
                    state.status = AP_GPS::GPS_OK_FIX_3D_DGPS;
                if (_buffer.pvt.flags & 0b01000000)  // carrsoln - float
                    state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT;
                if (_buffer.pvt.flags & 0b10000000)  // carrsoln - fixed
                    state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FIXED;
                break;
            case 4:
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO,
                                "Unexpected state %d", _buffer.pvt.flags);
                state.status = AP_GPS::GPS_OK_FIX_3D;
                break;
            case 5:
                state.status = AP_GPS::NO_FIX;
                break;
            default:
                state.status = AP_GPS::NO_FIX;
                break;
        }
        next_fix = state.status;
        _new_position = true;
        state.horizontal_accuracy = _buffer.pvt.h_acc*1.0e-3f;
        state.vertical_accuracy = _buffer.pvt.v_acc*1.0e-3f;
        state.have_horizontal_accuracy = true;
        state.have_vertical_accuracy = true;
        // SVs
        state.num_sats    = _buffer.pvt.num_sv;
        // velocity     
        _last_vel_time         = _buffer.pvt.itow;
        state.ground_speed     = _buffer.pvt.gspeed*0.001f;          // m/s
        state.ground_course    = wrap_360(_buffer.pvt.head_mot * 1.0e-5f);       // Heading 2D deg * 100000
        state.have_vertical_velocity = true;
        state.velocity.x = _buffer.pvt.velN * 0.001f;
        state.velocity.y = _buffer.pvt.velE * 0.001f;
        state.velocity.z = _buffer.pvt.velD * 0.001f;
        state.have_speed_accuracy = true;
        state.speed_accuracy = _buffer.pvt.s_acc*0.001f;
        _new_speed = true;
        // dop
        if(noReceivedHdop) {
            state.hdop        = _buffer.pvt.p_dop;
            state.vdop        = _buffer.pvt.p_dop;
        }
                    
        state.last_gps_time_ms = AP_HAL::millis();
        
        // time
        state.time_week_ms    = _buffer.pvt.itow;
#if UBLOX_FAKE_3DLOCK
        state.location.lng = 1491652300L;
        state.location.lat = -353632610L;
        state.location.alt = 58400;
        state.vertical_accuracy = 0;
        state.horizontal_accuracy = 0;
        state.status = AP_GPS::GPS_OK_FIX_3D;
        state.num_sats = 10;
        state.time_week = 1721;
        state.time_week_ms = AP_HAL::millis() + 3*60*60*1000 + 37000;
        state.last_gps_time_ms = AP_HAL::millis();
        state.hdop = 130;
        next_fix = state.status;
#endif
        break;
    case MSG_VELNED:
        Debug("MSG_VELNED");
        if (havePvtMsg) {
            _unconfigured_messages |= CONFIG_RATE_VELNED;
            break;
        }
        _last_vel_time         = _buffer.velned.time;
        state.ground_speed     = _buffer.velned.speed_2d*0.01f;          // m/s
        state.ground_course    = wrap_360(_buffer.velned.heading_2d * 1.0e-5f);       // Heading 2D deg * 100000
        state.have_vertical_velocity = true;
        state.velocity.x = _buffer.velned.ned_north * 0.01f;
        state.velocity.y = _buffer.velned.ned_east * 0.01f;
        state.velocity.z = _buffer.velned.ned_down * 0.01f;
        state.ground_course = wrap_360(degrees(atan2f(state.velocity.y, state.velocity.x)));
        state.ground_speed = norm(state.velocity.y, state.velocity.x);
        state.have_speed_accuracy = true;
        state.speed_accuracy = _buffer.velned.speed_accuracy*0.01f;
#if UBLOX_FAKE_3DLOCK
        state.speed_accuracy = 0;
#endif
        _new_speed = true;
        break;
    case MSG_NAV_SVINFO:
        {
        Debug("MSG_NAV_SVINFO\n");
        static const uint8_t HardwareGenerationMask = 0x07;
        _hardware_generation = _buffer.svinfo_header.globalFlags & HardwareGenerationMask;
        switch (_hardware_generation) {
            case UBLOX_5:
            case UBLOX_6:
                // only 7 and newer support CONFIG_GNSS
                _unconfigured_messages &= ~CONFIG_GNSS;
                break;
            case UBLOX_7:
            case UBLOX_M8:
#if UBLOX_SPEED_CHANGE
                port->begin(4000000U);
                Debug("Changed speed to 4Mhz for SPI-driven UBlox\n");
#endif
                break;
            default:
                hal.console->printf("Wrong Ublox Hardware Version%u\n", _hardware_generation);
                break;
        };
        _unconfigured_messages &= ~CONFIG_VERSION;
        /* We don't need that anymore */
        _configure_message_rate(CLASS_NAV, MSG_NAV_SVINFO, 0);
        break;
        }
    default:
        Debug("Unexpected NAV message 0x%02x", (unsigned)_msg_id);
        if (++_disable_counter == 0) {
            Debug("Disabling NAV message 0x%02x", (unsigned)_msg_id);
            _configure_message_rate(CLASS_NAV, _msg_id, 0);
        }
        return false;
    }

    // we only return true when we get new position and speed data
    // this ensures we don't use stale data
    if (_new_position && _new_speed && _last_vel_time == _last_pos_time) {
        _new_speed = _new_position = false;
        return true;
    }
    return false;
}


// UBlox auto configuration

/*
 *  update checksum for a set of bytes
 */
void
AP_GPS_UBLOX::_update_checksum(uint8_t *data, uint16_t len, uint8_t &ck_a, uint8_t &ck_b)
{
    while (len--) {
        ck_a += *data;
        ck_b += ck_a;
        data++;
    }
}


/*
 *  send a ublox message
 */
bool
AP_GPS_UBLOX::_send_message(uint8_t msg_class, uint8_t msg_id, void *msg, uint16_t size)
{
    if (port->txspace() < (sizeof(struct ubx_header) + 2 + size)) {
        return false;
    }
    struct ubx_header header;
    uint8_t ck_a=0, ck_b=0;
    header.preamble1 = PREAMBLE1;
    header.preamble2 = PREAMBLE2;
    header.msg_class = msg_class;
    header.msg_id    = msg_id;
    header.length    = size;

    _update_checksum((uint8_t *)&header.msg_class, sizeof(header)-2, ck_a, ck_b);
    _update_checksum((uint8_t *)msg, size, ck_a, ck_b);

    port->write((const uint8_t *)&header, sizeof(header));
    port->write((const uint8_t *)msg, size);
    port->write((const uint8_t *)&ck_a, 1);
    port->write((const uint8_t *)&ck_b, 1);
    return true;
}

/*
 *  requests the given message rate for a specific message class
 *  and msg_id
 *  returns true if it sent the request, false if waiting on knowing the port
 */
bool
AP_GPS_UBLOX::_request_message_rate(uint8_t msg_class, uint8_t msg_id)
{
    // Without knowing what communication port is being used it isn't possible to verify
    // always ensure we have a port before sending the request
    if(_ublox_port >= UBLOX_MAX_PORTS) {
        _request_port();
        return false;
    } else {
        struct ubx_cfg_msg msg;
        msg.msg_class = msg_class;
        msg.msg_id    = msg_id;
        return _send_message(CLASS_CFG, MSG_CFG_MSG, &msg, sizeof(msg));
    }
}

/*
 *  configure a UBlox GPS for the given message rate for a specific
 *  message class and msg_id
 */
bool
AP_GPS_UBLOX::_configure_message_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate)
{
    if (port->txspace() < (int16_t)(sizeof(struct ubx_header)+sizeof(struct ubx_cfg_msg_rate)+2)) {
        return false;
    }

    struct ubx_cfg_msg_rate msg;
    msg.msg_class = msg_class;
    msg.msg_id    = msg_id;
    msg.rate      = rate;
    return _send_message(CLASS_CFG, MSG_CFG_MSG, &msg, sizeof(msg));
}

/*
 * save gps configurations to non-volatile memory sent until the call of
 * this message
 */
void
AP_GPS_UBLOX::_save_cfg()
{
    ubx_cfg_cfg save_cfg;
    save_cfg.clearMask = 0;
    save_cfg.saveMask = SAVE_CFG_ALL;
    save_cfg.loadMask = 0;
    _send_message(CLASS_CFG, MSG_CFG_CFG, &save_cfg, sizeof(save_cfg));
    _last_cfg_sent_time = AP_HAL::millis();
    _num_cfg_save_tries++;
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO,
                                     "GPS: u-blox %d saving config",
                                     state.instance + 1);
}

/*
  detect a Ublox GPS. Adds one byte, and returns true if the stream
  matches a UBlox
 */
bool
AP_GPS_UBLOX::_detect(struct UBLOX_detect_state &state, uint8_t data)
{
reset:
	switch (state.step) {
        case 1:
            if (PREAMBLE2 == data) {
                state.step++;
                break;
            }
            state.step = 0;
            /* no break */
        case 0:
            if (PREAMBLE1 == data)
                state.step++;
            break;
        case 2:
            state.step++;
            state.ck_b = state.ck_a = data;
            break;
        case 3:
            state.step++;
            state.ck_b += (state.ck_a += data);
            break;
        case 4:
            state.step++;
            state.ck_b += (state.ck_a += data);
            state.payload_length = data;
            break;
        case 5:
            state.step++;
            state.ck_b += (state.ck_a += data);
            state.payload_counter = 0;
            break;
        case 6:
            state.ck_b += (state.ck_a += data);
            if (++state.payload_counter == state.payload_length)
                state.step++;
            break;
        case 7:
            state.step++;
            if (state.ck_a != data) {
                state.step = 0;
				goto reset;
            }
            break;
        case 8:
            state.step = 0;
			if (state.ck_b == data) {
				// a valid UBlox packet
				return true;
			} else {
				goto reset;
			}
    }
    return false;
}

void
AP_GPS_UBLOX::_request_version(void)
{
    _send_message(CLASS_MON, MSG_MON_VER, nullptr, 0);
}

void
AP_GPS_UBLOX::_configure_rate(void)
{
    struct ubx_cfg_nav_rate msg;
    // require a minimum measurement rate of 5Hz
    msg.measure_rate_ms = gps.get_rate_ms(state.instance);
    msg.nav_rate        = 1;
    msg.timeref         = 0;     // UTC time
    _send_message(CLASS_CFG, MSG_CFG_RATE, &msg, sizeof(msg));
}

void
AP_GPS_UBLOX::inject_data(const uint8_t *data, uint16_t len)
{
    if (port->txspace() > len) {
        port->write(data, len);
    } else {
        Debug("UBX: Not enough TXSPACE");
    }
}


static const char *reasons[] = {"navigation rate",
                                "posllh rate",
                                "status rate",
                                "solution rate",
                                "velned rate",
                                "dop rate",
                                "hw monitor rate",
                                "hw2 monitor rate",
                                "raw rate",
                                "version",
                                "navigation settings",
                                "GNSS settings",
                                "SBAS settings",
                                "PVT rate"};


void
AP_GPS_UBLOX::broadcast_configuration_failure_reason(void) const {
    for (uint8_t i = 0; i < ARRAY_SIZE(reasons); i++) {
        if (_unconfigured_messages & (1 << i)) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "GPS %d: u-blox %s configuration 0x%02x",
                state.instance +1, reasons[i], _unconfigured_messages);
            break;
        }
    }
}

/*
  return velocity lag in seconds
 */
float AP_GPS_UBLOX::get_lag(void) const
{
    switch (_hardware_generation) {
    case UBLOX_5:
    case UBLOX_6:
    default:
        return 0.22f;
    case UBLOX_7:
    case UBLOX_M8:
        // based on flight logs the 7 and 8 series seem to produce about 120ms lag
        return 0.12f;
        break;
    };
}
