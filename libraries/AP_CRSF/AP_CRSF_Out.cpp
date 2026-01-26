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
 * Code by Andy Piper <github@andypiper.com>
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
#include <AP_Scheduler/AP_Scheduler.h>

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

AP_CRSF_Out* AP_CRSF_Out::singleton;

extern const AP_HAL::HAL& hal;

AP_CRSF_Out::AP_CRSF_Out(AP_HAL::UARTDriver& _uart, uint8_t instance, AP_CRSF_OutManager& _frontend) :
    instance_idx(instance), uart(_uart), frontend(_frontend)
{
    // in the future we could consider supporting multiple output handlers
    if (singleton != nullptr) {
        AP_HAL::panic("Duplicate CRSF_Out handler");
    }

    singleton = this;

    init(uart);
}

// Initialise the CRSF output driver
bool AP_CRSF_Out::init(AP_HAL::UARTDriver& _uart)
{
    if (state != State::WAITING_FOR_PORT) {
        return false;
    }

    crsf_port = NEW_NOTHROW AP_RCProtocol_CRSF(AP::RC(), AP_RCProtocol_CRSF::PortMode::DIRECT_RCOUT, &_uart);

    if (crsf_port == nullptr) {
        debug_rcout("Init failed: could not create CRSF output port");
        return false;
    }

    const uint16_t rate = frontend.rate_hz.get();
    if (rate > 0) {
        frame_interval_us = 1000000UL / rate;
    } else {
        frame_interval_us = 1000000UL / DEFAULT_CRSF_OUTPUT_RATE;
    }

    scheduler.init(tasks, rate);
    state = State::WAITING_FOR_RC_LOCK;
    scheduler.set_task_rate(REPORTING, frontend.reporting_rate_hz);


    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_CRSF_Out::crsf_out_thread, void), "crsf", 2048, AP_HAL::Scheduler::PRIORITY_RCOUT, 1)) {
        delete crsf_port;
        crsf_port = nullptr;
        debug_rcout("Failed to create CRSF_Out thread");
        return false;
    }

    return true;
}

void AP_CRSF_Out::update_rates_status()
{
    const float report_rate = frontend.reporting_rate_hz.get();

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CRSFOut: RC: %uHz, Lat: %.1fms",
                    int16_t(rate_rc_counter*report_rate), latency_us / 1000.0f);
    rate_rc_counter = 0;
}

void AP_CRSF_Out::crsf_out_thread()
{
    // make sure the current thread is the uart owner
    crsf_port->start_uart();

    while (true) {

#ifdef CRSF_RCOUT_DEBUG
        const uint32_t now_ms = AP_HAL::millis();
        if (now_ms - last_update_debug_ms > 1000) {
            debug_rcout("Frame rate: %uHz, wanted: %uHz.", unsigned(num_frames), unsigned(frontend.rate_hz.get()));
            last_update_debug_ms = now_ms;
            num_frames = 0;
        }
#endif

        const uint32_t now_us = AP_HAL::micros();
        uint32_t interval_us = frame_interval_us;

        // if we have not negotiated a faster baudrate do not go above the default output rate
        if (uint16_t(frontend.rate_hz.get()) > DEFAULT_CRSF_OUTPUT_RATE && uart.get_baud_rate() == CRSF_BAUDRATE) {
            interval_us = 1000000UL / DEFAULT_CRSF_OUTPUT_RATE;
        }

        const uint32_t timeout_remaining_us = AP_HAL::timeout_remaining(last_frame_us, now_us, interval_us);
        if (timeout_remaining_us > 0) {
            hal.scheduler->delay_microseconds(timeout_remaining_us);
        }

        // Check for overrun - if we are late by more than 50% of an interval,
        // give up on the old timeline and reset.
        if (AP_HAL::timeout_remaining(last_frame_us, now_us, interval_us + interval_us/2) == 0) {
            last_frame_us = now_us;
        } else {
            // Use scheduled frame time to maintain precise rate
            last_frame_us = last_frame_us + interval_us;  // this may wrap, but that is still correct
        }

#ifdef CRSF_RCOUT_DEBUG
        num_frames++;
#endif
        if (state == State::RUNNING && crsf_port->is_rx_active()) {
            const bool send_rc_frame = pwm_is_fresh;
            pwm_is_fresh = false;

            if (send_rc_frame) {
                scheduler.run_task_immediately(AETR_RC_FRAME);
            }

            if (!scheduler.update() && !send_rc_frame) {
                send_heartbeat();
            }
        } else {
            run_state_machine();
        }

        // process bytes from the UART
        crsf_port->update_uart();
    }
}

// callback from the RC thread
void AP_CRSF_Out::update()
{

}

// PWM push called from SRV_Channels::push
void AP_CRSF_Out::push()
{
    pwm_is_fresh = true;
}

// run the state machine to get us to the running state
void AP_CRSF_Out::run_state_machine()
{
    const uint32_t now = AP_HAL::micros();
    const uint32_t now_ms = AP_HAL::millis();

    const uint32_t BAUD_NEG_TIMEOUT_US = 1500000; // 1.5 second timeout for a response
    const uint32_t BAUD_NEG_INTERVAL_US = 500000; // send proposal every 500ms
    const uint32_t LIVENESS_CHECK_TIMEOUT_US = 500000; // timeout for health check 500ms

    switch (state) {
    case State::WAITING_FOR_PORT:
        // should have been handled above
        break;

    case State::WAITING_FOR_RC_LOCK:
        // ArduPilot requires 3 good RC frames before it considers the protocol
        // detected, so keep sending RC frames until the rx registers as active
        // because we received something back
        send_aetr_rc_frame();

        if (AP_HAL::timeout_expired(last_status_update_ms, now_ms, 1000UL)) {
            last_status_update_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CRSFOut: waiting for RC lock");
        }

        if (crsf_port->is_rx_active()) {
            state = State::WAITING_FOR_DEVICE_INFO;
            debug_rcout("Telemetry active, requesting device information");
        }
        break;

    case State::WAITING_FOR_DEVICE_INFO:
        send_ping_frame();

        if (version.major > 0 && crsf_port->is_rx_active()) {
            state = State::NEGOTIATING_2M;
            target_baudrate = 2000000;
            crsf_port->reset_bootstrap_baudrate();
            baud_negotiation_result = BaudNegotiationResult::PENDING;
            debug_rcout("Initialised, negotiating baudrate");
        }
        break;

    case State::NEGOTIATING_2M:
    case State::NEGOTIATING_1M: {
        // Continue sending ping frames to keep the link alive
        send_ping_frame();

        // Check for response
        if (baud_negotiation_result == BaudNegotiationResult::SUCCESS) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CRSFOut: negotiated %uMBd", unsigned(target_baudrate/1000000));
            state = State::RUNNING;
            break;
        }

        if (baud_neg_start_us == 0) {
            baud_neg_start_us = now;
        }

        // Check for explicit failure (rejection)
        if (baud_negotiation_result == BaudNegotiationResult::FAILED) {
            if (state == State::NEGOTIATING_2M) {
                debug_rcout("2M baud negotiation rejected, trying 1M");
                state = State::NEGOTIATING_1M;
                target_baudrate = 1000000;
                crsf_port->reset_bootstrap_baudrate();
                baud_negotiation_result = BaudNegotiationResult::PENDING;
                last_baud_neg_us = 0; // force immediate send
                baud_neg_start_us = 0;
            } else { // NEGOTIATING_1M
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CRSFOut: baud negotiation failed, using 416kBd");
                scheduler.set_loop_rate(DEFAULT_CRSF_OUTPUT_RATE);
                state = State::RUNNING;
            }
            break;
        }

        // Check for timeout
        if (now - baud_neg_start_us > BAUD_NEG_TIMEOUT_US) {
            if (state == State::NEGOTIATING_2M) {
                debug_rcout("2M baud negotiation timed out, trying 1M");
                state = State::NEGOTIATING_1M;
                target_baudrate = 1000000;
                crsf_port->reset_bootstrap_baudrate();
                baud_negotiation_result = BaudNegotiationResult::PENDING;
                last_baud_neg_us = 0; // force immediate send
                baud_neg_start_us = 0;
            } else { // NEGOTIATING_1M
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CRSFOut: baud negotiation timed out, using 416kBd");
                state = State::RUNNING;
            }
            break;
        }

        // If pending, send proposal periodically
        if (now - last_baud_neg_us > BAUD_NEG_INTERVAL_US) {
            last_baud_neg_us = now;
            send_speed_proposal(target_baudrate);
            debug_rcout("Sent speed proposal for %u", (unsigned)target_baudrate);
        }
        break;
    }

    case State::RUNNING:
        if (!crsf_port->is_rx_active()) {
            debug_rcout("Connection lost, checking liveness");
            last_liveness_check_us = now;
            state = State::HEALTH_CHECK_PING;
            send_ping_frame(true);
        }
        break;

    case State::HEALTH_CHECK_PING:
        if (crsf_port->is_rx_active()) {
            state = State::RUNNING;
        } else if (now - last_liveness_check_us > LIVENESS_CHECK_TIMEOUT_US) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CRSFOut: connection lost");
            crsf_port->reset_bootstrap_baudrate();
            baud_negotiation_result = BaudNegotiationResult::PENDING;
            state = State::WAITING_FOR_RC_LOCK;
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
                    baud_negotiation_result = BaudNegotiationResult::SUCCESS;
                    // change baud on our end now
                    crsf_port->change_baud_rate(target_baudrate);
                } else {
                    debug_rcout("CRSF Speed Response Received: FAILED");
                    baud_negotiation_result = BaudNegotiationResult::FAILED;
                }
            }
        }
            break;
        case AP_CRSF_Protocol::FrameType::CRSF_FRAMETYPE_PARAM_DEVICE_INFO:
            if (last_latency_ping_us > 0) {
                latency_us = AP_HAL::micros() - last_latency_ping_us;
                last_latency_ping_us = 0;
            }
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

// send control frame, this goes out at the loop rate and so
// needs to be kept small
void AP_CRSF_Out::send_aetr_rc_frame()
{
    send_rc_frame(0, 4);
    rate_rc_counter++;
}

// send aux frame, this can go out at a low rate - 50Hz
void AP_CRSF_Out::send_aux_rc_frame()
{
    send_rc_frame(4, MIN(NUM_SERVO_CHANNELS, (uint8_t)CRSF_MAX_CHANNELS) - 4);
}

// sends RC frames at the configured rate
void AP_CRSF_Out::send_rc_frame(uint8_t start_chan, uint8_t nchan)
{
    uint16_t channels[CRSF_MAX_CHANNELS] {};
    for (uint8_t i = start_chan; i < start_chan + nchan; ++i) {
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
    uint8_t payload_len = AP_CRSF_Protocol::encode_variable_bit_channels(frame.payload, channels, nchan, start_chan);
    frame.length = payload_len + 2; // +1 for type, +1 for CRC

    crsf_port->write_frame(&frame);
}

void AP_CRSF_Out::send_latency_ping_frame()
{
    last_latency_ping_us = AP_HAL::micros();
    send_ping_frame(true);
}

void AP_CRSF_Out::send_ping_frame(bool force)
{
    // only send pings at 50Hz max
    const uint32_t now_ms = AP_HAL::millis();
    if (!AP_HAL::timeout_expired(last_ping_frame_ms, now_ms, 20UL) && !force) {
        return;
    }
    last_ping_frame_ms = now_ms;

    debug_rcout("send_ping_frame()");

    AP_CRSF_Protocol::Frame frame;
    AP_CRSF_Protocol::encode_ping_frame(frame, DeviceAddress::CRSF_ADDRESS_FLIGHT_CONTROLLER, DeviceAddress::CRSF_ADDRESS_CRSF_RECEIVER);

    crsf_port->write_frame(&frame);
}

// send a baudrate proposal
void AP_CRSF_Out::send_speed_proposal(uint32_t baudrate)
{
    debug_rcout("send_speed_proposal(%u)", (unsigned)baudrate);

    crsf_port->change_baud_rate(baudrate);

    AP_CRSF_Protocol::Frame frame;
    AP_CRSF_Protocol::encode_speed_proposal(baudrate, frame, DeviceAddress::CRSF_ADDRESS_FLIGHT_CONTROLLER, DeviceAddress::CRSF_ADDRESS_CRSF_RECEIVER);

    crsf_port->write_frame(&frame);
}

void AP_CRSF_Out::send_device_info()
{
    debug_rcout("send_device_info()");

    AP_CRSF_Protocol::Frame frame;
    AP_CRSF_Protocol::encode_device_info_frame(frame, DeviceAddress::CRSF_ADDRESS_FLIGHT_CONTROLLER, DeviceAddress::CRSF_ADDRESS_CRSF_RECEIVER);

    crsf_port->write_frame(&frame);
}

void AP_CRSF_Out::send_link_stats_tx()
{
    debug_rcout("send_link_stats_tx()");

    AP_CRSF_Protocol::Frame frame;
    AP_CRSF_Protocol::encode_link_stats_tx_frame(frontend.rate_hz, frame, DeviceAddress::CRSF_ADDRESS_FLIGHT_CONTROLLER, DeviceAddress::CRSF_ADDRESS_CRSF_RECEIVER);

    crsf_port->write_frame(&frame);
}

void AP_CRSF_Out::send_heartbeat()
{
    AP_CRSF_Protocol::Frame frame;
    AP_CRSF_Protocol::encode_heartbeat_frame(frame, DeviceAddress::CRSF_ADDRESS_CRSF_RECEIVER);

    crsf_port->write_frame(&frame);
}

namespace AP {
    AP_CRSF_Out* crsf_out() {
        return AP_CRSF_Out::get_singleton();
    }
};

#endif // AP_CRSF_OUT_ENABLED

