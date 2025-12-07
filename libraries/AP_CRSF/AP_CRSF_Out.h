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
 * AP_CRSF_Out.h - High-level driver for CRSF RC Output
 */
#pragma once

#include "AP_CRSF_config.h"

#if AP_CRSF_OUT_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include "AP_CRSF_Protocol.h"
#include "AP_CRSF_OutManager.h"

class AP_RCProtocol_CRSF;
class AP_CRSF_OutManager;

/*
 * The AP_CRSF_Out class provides the high-level "application" logic for the
 * CRSF RC Output feature. It is responsible for reading servo output values
 * from the main SRV_Channels and telling its underlying CRSF protocol instance
 * to send them at a user-configurable rate. It also handles baud rate negotiation.
 */
class AP_CRSF_Out : public AP_CRSF_Protocol {
public:
    // constructor for serial interaction
    AP_CRSF_Out(AP_HAL::UARTDriver& uart, uint8_t instance, AP_CRSF_OutManager& frontend);

    ~AP_CRSF_Out() override {}

    /* Do not allow copies */
    CLASS_NO_COPY(AP_CRSF_Out);

    // one-time initialisation
    bool init(AP_HAL::UARTDriver& uart);

    // rc periodic update, called from loop
    void update() override;

    // main loop for the CRSF output thread
    void loop();

    // main loop for the CRSF output thread
    void crsf_out_thread();

    bool decode_crsf_packet(const AP_CRSF_Protocol::Frame& _frame);

    static const struct AP_Param::GroupInfo var_info[];
    static AP_CRSF_Out* get_singleton() { return _singleton; }

private:
    enum class State : uint8_t {
        WAITING_FOR_PORT,
        WAITING_FOR_RC_LOCK,
        WAITING_FOR_DEVICE_INFO,
        NEGOTIATING_2M,
        NEGOTIATING_1M,
        HEALTH_CHECK_PING,
        RUNNING,
    };

    // import enums from AP_CRSF_Protocol for convenience
    using FrameType = AP_CRSF_Protocol::FrameType;
    using DeviceAddress = AP_CRSF_Protocol::DeviceAddress;
    using CommandID = AP_CRSF_Protocol::CommandID;
    using CommandGeneral = AP_CRSF_Protocol::CommandGeneral;
    
    enum class BaudNegotiationResult : uint8_t {
        PENDING,
        SUCCESS,
        FAILED,
    };

    bool should_do_status_update();

    // sends RC frames at the configured rate
    void send_rc_frame();
    // send a baudrate proposal
    void send_speed_proposal(uint32_t baudrate);
    // send a ping frame
    void send_ping_frame(bool force = false);
    // send a device info frame
    void send_device_info();
    // send the link stats frame
    void send_link_stats_tx(uint32_t fps);
    // low bandwidth heartbeat to provoke telemetry return
    void send_heartbeat();

    static AP_CRSF_Out* _singleton;
    static uint8_t _num_instances;

    State _state = State::WAITING_FOR_PORT;
    uint32_t _last_frame_us;
    uint32_t _last_status_update_ms;
    uint32_t _last_baud_neg_us;
    uint32_t _baud_neg_start_us;
    uint32_t _frame_interval_us;
    uint8_t _heartbeat_to_frame_ratio;
    uint32_t _target_baudrate;
    uint32_t _last_liveness_check_us;
    uint32_t _last_ping_frame_ms;
    uint8_t _instance_idx;

    AP_CRSF_Protocol::VersionInfo version;
    BaudNegotiationResult _baud_negotiation_result;
    // check baudrate negotiation status
    BaudNegotiationResult get_baud_negotiation_result() const { return _baud_negotiation_result; }
    void reset_baud_negotiation();

    // pointer to the CRSF protocol engine instance for our assigned UART
    AP_RCProtocol_CRSF* _crsf_port;
    AP_HAL::UARTDriver& _uart;
    AP_CRSF_OutManager& _frontend;
};

namespace AP {
    AP_CRSF_Out* crsf_out();
};

#endif // AP_CRSF_OUT_ENABLED
