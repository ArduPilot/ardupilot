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
 */

/*
 * AP_CRSF_Out.cpp - High-level driver for CRSF RC Output
 *
 * This class provides the high-level "application" logic for the
 * CRSF RC Output feature. It is responsible for reading servo output values
 * from the main SRV_Channels and telling its underlying CRSF protocol instance
 * to send them at a user-configurable rate.
 */
#include "AP_CRSF_config.h"

#if AP_CRSF_OUT_ENABLED

#include "AP_CRSF_Out.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_RCProtocol/AP_RCProtocol_CRSF.h>
#include <GCS_MAVLink/GCS.h>

//#define CRSF_RCOUT_DEBUG
//#define CRSF_RCOUT_DEBUG_FRAME
#ifdef CRSF_RCOUT_DEBUG
# include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;
static uint32_t last_update_debug_ms = 0;
static uint32_t num_frames = 0;
# define debug_rcout(fmt, args...) do { if (hal.console) { hal.console->printf("CRSF_OUT: " fmt "\n", ##args); } } while(0)
#else
# define debug_rcout(fmt, args...)
#endif

#define DEFAULT_CRSF_OUTPUT_RATE      250U // equivalent to tracer

AP_CRSF_Out* AP_CRSF_Out::_singleton;

extern const AP_HAL::HAL& hal;

AP_CRSF_Out::AP_CRSF_Out(AP_HAL::UARTDriver* uart, uint8_t instance, AP_CRSF_OutManager& frontend) :
    _instance_idx(instance), _uart(uart), _frontend(frontend)
{
    if (_singleton == nullptr) {
        _singleton = this;
    }
    init(uart);
}

// Initialise the CRSF output driver
bool AP_CRSF_Out::init(AP_HAL::UARTDriver* uart)
{
    if (_state != State::WAITING_FOR_PORT) {
        return false;
    }

    _crsf_port = NEW_NOTHROW AP_RCProtocol_CRSF(AP::RC(), AP_RCProtocol_CRSF::PortMode::DIRECT_RCOUT, uart);

    if (_crsf_port == nullptr) {
        debug_rcout("Init failed: could not create CRSF output port");
        return false;
    }

    const uint16_t rate = _frontend._rate_hz.get();
    if (rate > 0) {
        _frame_interval_us = 1000000UL / rate;
        _heartbeat_to_frame_ratio = MAX(1U, rate / DEFAULT_CRSF_OUTPUT_RATE);
    } else {
        _frame_interval_us = 1000000UL / DEFAULT_CRSF_OUTPUT_RATE;
        _heartbeat_to_frame_ratio = 1;
    }

    _state = State::WAITING_FOR_RC_LOCK;

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_CRSF_Out::crsf_out_thread, void), "crsf", 2048, AP_HAL::Scheduler::PRIORITY_RCOUT, 1)) {
        delete _crsf_port;
        debug_rcout("Failed to create CRSF_Out thread");
        return false;
    }

    return true;
}

bool AP_CRSF_Out::should_do_status_update()
{
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_status_update_ms > 1000) {
        _last_status_update_ms = now_ms;
        return true;
    }

    return false;
}

void AP_CRSF_Out::crsf_out_thread()
{
    uint8_t frame_counter = 0;

    // make sure the current thread is the uart owner
    _crsf_port->start_uart();

    while (true) {

#ifdef CRSF_RCOUT_DEBUG
        const uint32_t now_ms = AP_HAL::millis();
        if (now_ms - last_update_debug_ms > 1000) {
            debug_rcout("Frame rate: %uHz, wanted: %uHz.", unsigned(num_frames), unsigned(_frontend._rate_hz.get()));
            last_update_debug_ms = now_ms;
            num_frames = 0;
        }
#endif

        const uint32_t now = AP_HAL::micros();

        // Calculate time remaining until the next scheduled frame to maintain precise rate
        uint32_t next_run = _last_frame_us + _frame_interval_us;
        uint8_t frame_ratio = _heartbeat_to_frame_ratio;

        // if we have not negotiated a faster baudrate do not go above the default output rate
        if (uint16_t(_frontend._rate_hz.get()) > DEFAULT_CRSF_OUTPUT_RATE && _uart->get_baud_rate() == CRSF_BAUDRATE) {
            next_run = _last_frame_us + 1000000UL / DEFAULT_CRSF_OUTPUT_RATE;
            frame_ratio = 1;
        }

        // Check for overrun - if we are late by more than 50% of an interval,
        // give up on the old timeline and reset.
        if (now > next_run + (_frame_interval_us / 2)) {
             next_run = now; 
        }

        if (now < next_run) {
            hal.scheduler->delay_microseconds(next_run - now);
        }

        // last time we sent and received a frame
        _last_frame_us = next_run;
#ifdef CRSF_RCOUT_DEBUG
        num_frames++;
#endif
        if (_state == State::RUNNING && ++frame_counter < frame_ratio) {
            send_heartbeat();
        } else {
            loop();
            frame_counter = 0;
        }

        // process bytes from the UART
        _crsf_port->update_uart();
    }
}

// callback from the RC thread
void AP_CRSF_Out::update()
{

}

// Main update call, sends RC frames at the configured rate
void AP_CRSF_Out::loop()
{
    const uint32_t now = AP_HAL::micros();

    const uint32_t BAUD_NEG_TIMEOUT_US = 1500000; // 1.5 second timeout for a response
    const uint32_t BAUD_NEG_INTERVAL_US = 500000; // send proposal every 500ms
    const uint32_t LIVENESS_CHECK_TIMEOUT_US = 500000; // timeout for health check 500ms

    switch (_state) {
    case State::WAITING_FOR_PORT:
        // should have been handled above
        break;

    case State::WAITING_FOR_RC_LOCK:
        // ArduPilot requires 3 good RC frames before it considers the protocol
        // detected, so keep sending RC frames until the rx registers as active
        // because we received something back
        send_rc_frame();

        if (should_do_status_update()) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CRSFOut: waiting for RC lock");
        }

        if (_crsf_port->is_rx_active()) {
            _state = State::WAITING_FOR_DEVICE_INFO;
            debug_rcout("Telemetry active, requesting device information");
        }
        break;

    case State::WAITING_FOR_DEVICE_INFO:
        send_ping_frame();

        if (version.major > 0 && _crsf_port->is_rx_active()) {
            _state = State::NEGOTIATING_2M;
            _target_baudrate = 2000000;
            _crsf_port->reset_bootstrap_baudrate();
            _baud_negotiation_result = BaudNegotiationResult::PENDING;
            debug_rcout("Initialised, negotiating baudrate");
        }
        break;

    case State::NEGOTIATING_2M:
    case State::NEGOTIATING_1M: {
        // Continue sending ping frames to keep the link alive
        send_ping_frame();

        // Check for response
        if (_baud_negotiation_result == BaudNegotiationResult::SUCCESS) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CRSFOut: negotiated %uMBd", unsigned(_target_baudrate/1000000));
            _state = State::RUNNING;
            break;
        }

        if (_baud_neg_start_us == 0) {
            _baud_neg_start_us = now;
        }

        // Check for explicit failure (rejection)
        if (_baud_negotiation_result == BaudNegotiationResult::FAILED) {
            if (_state == State::NEGOTIATING_2M) {
                debug_rcout("2M baud negotiation rejected, trying 1M");
                _state = State::NEGOTIATING_1M;
                _target_baudrate = 1000000;
                _crsf_port->reset_bootstrap_baudrate();
                _baud_negotiation_result = BaudNegotiationResult::PENDING;
                _last_baud_neg_us = 0; // force immediate send
                _baud_neg_start_us = 0;
            } else { // NEGOTIATING_1M
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CRSFOut: baud negotiation failed, using 416kBd");
                _state = State::RUNNING;
            }
            break;
        }

        // Check for timeout
        if (now - _baud_neg_start_us > BAUD_NEG_TIMEOUT_US) {
            if (_state == State::NEGOTIATING_2M) {
                debug_rcout("2M baud negotiation timed out, trying 1M");
                _state = State::NEGOTIATING_1M;
                _target_baudrate = 1000000;
                _crsf_port->reset_bootstrap_baudrate();
                _baud_negotiation_result = BaudNegotiationResult::PENDING;
                _last_baud_neg_us = 0; // force immediate send
                _baud_neg_start_us = 0;
            } else { // NEGOTIATING_1M
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CRSFOut: baud negotiation timed out, using 416kBd");
                _state = State::RUNNING;
            }
            break;
        }

        // If pending, send proposal periodically
        if (now - _last_baud_neg_us > BAUD_NEG_INTERVAL_US) {
            _last_baud_neg_us = now;
            send_speed_proposal(_target_baudrate);
            debug_rcout("Sent speed proposal for %u", (unsigned)_target_baudrate);
        }
        break;
    }

    case State::RUNNING:
        if (_crsf_port->is_rx_active()) {   // the remote side is sending data, we can send frames
            // periodically send link stats info
            if (now - _last_liveness_check_us > LIVENESS_CHECK_TIMEOUT_US) {
                send_link_stats_tx(_frontend._rate_hz);
                _last_liveness_check_us = now;
            } else {
                send_rc_frame();
            }
        } else {
            debug_rcout("Connection lost, checking liveness");
            _last_liveness_check_us = now;
            _state = State::HEALTH_CHECK_PING;
            send_ping_frame(true);
        }
        break;

    case State::HEALTH_CHECK_PING:
        if (_crsf_port->is_rx_active()) {
            _state = State::RUNNING;
        } else if (now - _last_liveness_check_us > LIVENESS_CHECK_TIMEOUT_US) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CRSFOut: connection lost");
            _crsf_port->reset_bootstrap_baudrate();
            _baud_negotiation_result = BaudNegotiationResult::PENDING;
            _state = State::WAITING_FOR_RC_LOCK;
        }
        break;
    }
}

bool AP_CRSF_Out::decode_crsf_packet(const AP_CRSF_Protocol::Frame& _frame)
{
#ifdef CRSF_RCOUT_DEBUG_FRAME
    hal.console->printf("CRSFOut: received %s:", AP_CRSF_Protocol::get_frame_type(_frame.type));
    uint8_t* fptr = (uint8_t*)&_frame;
    for (uint8_t i = 0; i < _frame.length + 2; i++) {
        hal.console->printf(" 0x%x", fptr[i]);
    }
    hal.console->printf("\n");
#endif

    switch (_frame.type) {
        case AP_CRSF_Protocol::FrameType::CRSF_FRAMETYPE_COMMAND: {
            const AP_CRSF_Protocol::CommandFrame* cmd = (const AP_CRSF_Protocol::CommandFrame*)_frame.payload;
            if (cmd->origin == AP_CRSF_Protocol::DeviceAddress::CRSF_ADDRESS_FLIGHT_CONTROLLER &&
                cmd->command_id == AP_CRSF_Protocol::CRSF_COMMAND_GENERAL &&
                cmd->payload[0] == AP_CRSF_Protocol::CRSF_COMMAND_GENERAL_CRSF_SPEED_RESPONSE) {
                const bool success = cmd->payload[2];
                if (success) {
                    debug_rcout("CRSF Speed Response Received: SUCCESS");
                    _baud_negotiation_result = BaudNegotiationResult::SUCCESS;
                    // change baud on our end now
                    _crsf_port->change_baud_rate(_target_baudrate);
                } else {
                    debug_rcout("CRSF Speed Response Received: FAILED");
                    _baud_negotiation_result = BaudNegotiationResult::FAILED;
                }
            }
        }
            break;
        case AP_CRSF_Protocol::FrameType::CRSF_FRAMETYPE_PARAM_DEVICE_INFO:
            AP_CRSF_Protocol::process_device_info_frame((AP_CRSF_Protocol::ParameterDeviceInfoFrame*)_frame.payload,
                                                         &version, true);
            break;

        case AP_CRSF_Protocol::FrameType::CRSF_FRAMETYPE_PARAM_DEVICE_PING:
            send_device_info();
            break;

        default:
            break;
    }

    return true;
}

// sends RC frames at the configured rate
void AP_CRSF_Out::send_rc_frame()
{
    uint16_t channels[CRSF_MAX_CHANNELS] {};
    const uint8_t nchan = MIN(NUM_SERVO_CHANNELS, (uint8_t)CRSF_MAX_CHANNELS);

    for (uint8_t i = 0; i < nchan; ++i) {
        SRV_Channel *c = SRV_Channels::srv_channel(i);
        if (c != nullptr) {
            channels[i] = c->get_output_pwm();
        } else {
            channels[i] = 1500; // Default to neutral if channel is null
        }
    }

    AP_CRSF_Protocol::Frame frame {};

    frame.device_address = DeviceAddress::CRSF_ADDRESS_SYNC_BYTE;
    frame.type = FrameType::CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED;
    uint8_t payload_len = AP_CRSF_Protocol::encode_variable_bit_channels(frame.payload, channels, nchan);
    frame.length = payload_len + 2; // +1 for type, +1 for CRC

    _crsf_port->write_frame(&frame);
}

void AP_CRSF_Out::send_ping_frame(bool force)
{
    // only send pings at 50Hz max
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_ping_frame_ms < 20 && !force) {
        return;
    }
    _last_ping_frame_ms = now_ms;

    debug_rcout("send_ping_frame()");

    AP_CRSF_Protocol::Frame frame;
    AP_CRSF_Protocol::encode_ping_frame(frame, DeviceAddress::CRSF_ADDRESS_FLIGHT_CONTROLLER, DeviceAddress::CRSF_ADDRESS_CRSF_RECEIVER);

    _crsf_port->write_frame(&frame);
}

// send a baudrate proposal
void AP_CRSF_Out::send_speed_proposal(uint32_t baudrate)
{
    debug_rcout("send_speed_proposal(%u)", (unsigned)baudrate);

    _crsf_port->change_baud_rate(baudrate);

    AP_CRSF_Protocol::Frame frame;
    AP_CRSF_Protocol::encode_speed_proposal(baudrate, frame, DeviceAddress::CRSF_ADDRESS_FLIGHT_CONTROLLER, DeviceAddress::CRSF_ADDRESS_CRSF_RECEIVER);

    _crsf_port->write_frame(&frame);
}

void AP_CRSF_Out::send_device_info()
{
    debug_rcout("send_device_info()");

    AP_CRSF_Protocol::Frame frame;
    AP_CRSF_Protocol::encode_device_info_frame(frame, DeviceAddress::CRSF_ADDRESS_FLIGHT_CONTROLLER, DeviceAddress::CRSF_ADDRESS_CRSF_RECEIVER);

    _crsf_port->write_frame(&frame);
}

void AP_CRSF_Out::send_link_stats_tx(uint32_t fps)
{
    debug_rcout("send_link_stats_tx()");

    AP_CRSF_Protocol::Frame frame;
    AP_CRSF_Protocol::encode_link_stats_tx_frame(fps, frame, DeviceAddress::CRSF_ADDRESS_FLIGHT_CONTROLLER, DeviceAddress::CRSF_ADDRESS_CRSF_RECEIVER);

    _crsf_port->write_frame(&frame);
}

void AP_CRSF_Out::send_heartbeat()
{
    AP_CRSF_Protocol::Frame frame;
    AP_CRSF_Protocol::encode_heartbeat_frame(frame, DeviceAddress::CRSF_ADDRESS_CRSF_RECEIVER);

    _crsf_port->write_frame(&frame);
}


namespace AP {
    AP_CRSF_Out* crsf_out() {
        return AP_CRSF_Out::get_singleton();
    }
};

#endif // AP_CRSF_OUT_ENABLED

