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
 * Author: Eugene Shamaev, Siddharth Bharat Purohit
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#if HAL_ENABLE_DRONECAN_DRIVERS
#include "AP_DroneCAN.h"
#include <GCS_MAVLink/GCS.h>

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_CANManager/AP_CANManager.h>

#include <AP_Arming/AP_Arming.h>
#include <AP_GPS/AP_GPS_DroneCAN.h>
#include <AP_Compass/AP_Compass_DroneCAN.h>
#include <AP_Baro/AP_Baro_DroneCAN.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_BattMonitor/AP_BattMonitor_DroneCAN.h>
#include <AP_Airspeed/AP_Airspeed_DroneCAN.h>
#include <AP_OpticalFlow/AP_OpticalFlow_HereFlow.h>
#include <AP_RangeFinder/AP_RangeFinder_DroneCAN.h>
#include <AP_RCProtocol/AP_RCProtocol_DroneCAN.h>
#include <AP_EFI/AP_EFI_DroneCAN.h>
#include <AP_GPS/AP_GPS_DroneCAN.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_BattMonitor/AP_BattMonitor_DroneCAN.h>
#include <AP_Compass/AP_Compass_DroneCAN.h>
#include <AP_Airspeed/AP_Airspeed_DroneCAN.h>
#include <AP_Proximity/AP_Proximity_DroneCAN.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_ADSB/AP_ADSB.h>
#include "AP_DroneCAN_DNA_Server.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_OpenDroneID/AP_OpenDroneID.h>
#include <AP_Mount/AP_Mount_Xacti.h>
#include <string.h>

extern const AP_HAL::HAL& hal;

// setup default pool size
#ifndef DRONECAN_NODE_POOL_SIZE
#if HAL_CANFD_SUPPORTED
#define DRONECAN_NODE_POOL_SIZE 16384
#else
#define DRONECAN_NODE_POOL_SIZE 8192
#endif
#endif

#if HAL_CANFD_SUPPORTED
#define DRONECAN_STACK_SIZE     8192
#else
#define DRONECAN_STACK_SIZE     4096
#endif

#ifndef AP_DRONECAN_VOLZ_FEEDBACK_ENABLED
#define AP_DRONECAN_VOLZ_FEEDBACK_ENABLED 0
#endif

#define AP_DRONECAN_GETSET_TIMEOUT_MS 100       // timeout waiting for response from node after 0.1 sec

#define debug_dronecan(level_debug, fmt, args...) do { AP::can().log_text(level_debug, "DroneCAN", fmt, ##args); } while (0)

// Translation of all messages from DroneCAN structures into AP structures is done
// in AP_DroneCAN and not in corresponding drivers.
// The overhead of including definitions of DSDL is very high and it is best to
// concentrate in one place.

// table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_DroneCAN::var_info[] = {
    // @Param: NODE
    // @DisplayName: DroneCAN node that is used for this network
    // @Description: DroneCAN node should be set implicitly
    // @Range: 1 250
    // @User: Advanced
    AP_GROUPINFO("NODE", 1, AP_DroneCAN, _dronecan_node, 10),

    // @Param: SRV_BM
    // @DisplayName: Output channels to be transmitted as servo over DroneCAN
    // @Description: Bitmask with one set for channel to be transmitted as a servo command over DroneCAN
    // @Bitmask: 0: Servo 1, 1: Servo 2, 2: Servo 3, 3: Servo 4, 4: Servo 5, 5: Servo 6, 6: Servo 7, 7: Servo 8, 8: Servo 9, 9: Servo 10, 10: Servo 11, 11: Servo 12, 12: Servo 13, 13: Servo 14, 14: Servo 15, 15: Servo 16, 16: Servo 17, 17: Servo 18, 18: Servo 19, 19: Servo 20, 20: Servo 21, 21: Servo 22, 22: Servo 23, 23: Servo 24, 24: Servo 25, 25: Servo 26, 26: Servo 27, 27: Servo 28, 28: Servo 29, 29: Servo 30, 30: Servo 31, 31: Servo 32

    // @User: Advanced
    AP_GROUPINFO("SRV_BM", 2, AP_DroneCAN, _servo_bm, 0),

    // @Param: ESC_BM
    // @DisplayName: Output channels to be transmitted as ESC over DroneCAN
    // @Description: Bitmask with one set for channel to be transmitted as a ESC command over DroneCAN
    // @Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32
    // @User: Advanced
    AP_GROUPINFO("ESC_BM", 3, AP_DroneCAN, _esc_bm, 0),

    // @Param: SRV_RT
    // @DisplayName: Servo output rate
    // @Description: Maximum transmit rate for servo outputs
    // @Range: 1 200
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("SRV_RT", 4, AP_DroneCAN, _servo_rate_hz, 50),

    // @Param: OPTION
    // @DisplayName: DroneCAN options
    // @Description: Option flags
    // @Bitmask: 0:ClearDNADatabase,1:IgnoreDNANodeConflicts,2:EnableCanfd,3:IgnoreDNANodeUnhealthy,4:SendServoAsPWM,5:SendGNSS,6:UseHimarkServo,7:HobbyWingESC,8:EnableStats
    // @User: Advanced
    AP_GROUPINFO("OPTION", 5, AP_DroneCAN, _options, 0),
    
    // @Param: NTF_RT
    // @DisplayName: Notify State rate
    // @Description: Maximum transmit rate for Notify State Message
    // @Range: 1 200
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("NTF_RT", 6, AP_DroneCAN, _notify_state_hz, 20),

    // @Param: ESC_OF
    // @DisplayName: ESC Output channels offset
    // @Description: Offset for ESC numbering in DroneCAN ESC RawCommand messages. This allows for more efficient packing of ESC command messages. If your ESCs are on servo functions 5 to 8 and you set this parameter to 4 then the ESC RawCommand will be sent with the first 4 slots filled. This can be used for more efficint usage of CAN bandwidth
    // @Range: 0 18
    // @User: Advanced
    AP_GROUPINFO("ESC_OF", 7, AP_DroneCAN, _esc_offset, 0),

    // @Param: POOL
    // @DisplayName: CAN pool size
    // @Description: Amount of memory in bytes to allocate for the DroneCAN memory pool. More memory is needed for higher CAN bus loads
    // @Range: 1024 16384
    // @User: Advanced
    AP_GROUPINFO("POOL", 8, AP_DroneCAN, _pool_size, DRONECAN_NODE_POOL_SIZE),

    // @Param: ESC_RV
    // @DisplayName: Bitmask for output channels for reversible ESCs over DroneCAN.
    // @Description: Bitmask with one set for each output channel that uses a reversible ESC over DroneCAN. Reversible ESCs use both positive and negative values in RawCommands, with positive commanding the forward direction and negative commanding the reverse direction.
    // @Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32
    // @User: Advanced
    AP_GROUPINFO("ESC_RV", 9, AP_DroneCAN, _esc_rv, 0),
    
    AP_GROUPEND
};

// this is the timeout in milliseconds for periodic message types. We
// set this to 1 to minimise resend of stale msgs
#define CAN_PERIODIC_TX_TIMEOUT_MS 2

AP_DroneCAN::AP_DroneCAN(const int driver_index) :
_driver_index(driver_index),
canard_iface(driver_index),
_dna_server(*this, canard_iface, driver_index)
{
    AP_Param::setup_object_defaults(this, var_info);

    for (uint8_t i = 0; i < DRONECAN_SRV_NUMBER; i++) {
        _SRV_conf[i].esc_pending = false;
        _SRV_conf[i].servo_pending = false;
    }

    debug_dronecan(AP_CANManager::LOG_INFO, "AP_DroneCAN constructed\n\r");
}

AP_DroneCAN::~AP_DroneCAN()
{
}

AP_DroneCAN *AP_DroneCAN::get_dronecan(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) != AP_CAN::Protocol::DroneCAN) {
        return nullptr;
    }
    return static_cast<AP_DroneCAN*>(AP::can().get_driver(driver_index));
}

bool AP_DroneCAN::add_interface(AP_HAL::CANIface* can_iface)
{
    if (!canard_iface.add_interface(can_iface)) {
        debug_dronecan(AP_CANManager::LOG_ERROR, "DroneCAN: can't add DroneCAN interface\n\r");
        return false;   
    }
    return true;
}

void AP_DroneCAN::init(uint8_t driver_index, bool enable_filters)
{
    if (driver_index != _driver_index) {
        debug_dronecan(AP_CANManager::LOG_ERROR, "DroneCAN: init called with wrong driver_index");
        return;
    }
    if (_initialized) {
        debug_dronecan(AP_CANManager::LOG_ERROR, "DroneCAN: init called more than once\n\r");
        return;
    }

    node_info_rsp.name.len = hal.util->snprintf((char*)node_info_rsp.name.data, sizeof(node_info_rsp.name.data), "org.ardupilot:%u", driver_index);

    node_info_rsp.software_version.major = AP_DRONECAN_SW_VERS_MAJOR;
    node_info_rsp.software_version.minor = AP_DRONECAN_SW_VERS_MINOR;
    node_info_rsp.hardware_version.major = AP_DRONECAN_HW_VERS_MAJOR;
    node_info_rsp.hardware_version.minor = AP_DRONECAN_HW_VERS_MINOR;

#if HAL_CANFD_SUPPORTED
    if (option_is_set(Options::CANFD_ENABLED)) {
        canard_iface.set_canfd(true);
    }
#endif

    uint8_t uid_len = sizeof(uavcan_protocol_HardwareVersion::unique_id);
    uint8_t unique_id[sizeof(uavcan_protocol_HardwareVersion::unique_id)];

    mem_pool = new uint32_t[_pool_size/sizeof(uint32_t)];
    if (mem_pool == nullptr) {
        debug_dronecan(AP_CANManager::LOG_ERROR, "DroneCAN: Failed to allocate memory pool\n\r");
        return;
    }
    canard_iface.init(mem_pool, (_pool_size/sizeof(uint32_t))*sizeof(uint32_t), _dronecan_node);

    if (!hal.util->get_system_id_unformatted(unique_id, uid_len)) {
        return;
    }
    unique_id[uid_len - 1] += _dronecan_node;
    memcpy(node_info_rsp.hardware_version.unique_id, unique_id, uid_len);

    //Start Servers
    if (!_dna_server.init(unique_id, uid_len, _dronecan_node)) {
        debug_dronecan(AP_CANManager::LOG_ERROR, "DroneCAN: Failed to start DNA Server\n\r");
        return;
    }

    // Roundup all subscribers from supported drivers
#if AP_GPS_DRONECAN_ENABLED
    AP_GPS_DroneCAN::subscribe_msgs(this);
#endif
#if AP_COMPASS_DRONECAN_ENABLED
    AP_Compass_DroneCAN::subscribe_msgs(this);
#endif
#if AP_BARO_DRONECAN_ENABLED
    AP_Baro_DroneCAN::subscribe_msgs(this);
#endif
    AP_BattMonitor_DroneCAN::subscribe_msgs(this);
#if AP_AIRSPEED_DRONECAN_ENABLED
    AP_Airspeed_DroneCAN::subscribe_msgs(this);
#endif
#if AP_OPTICALFLOW_HEREFLOW_ENABLED
    AP_OpticalFlow_HereFlow::subscribe_msgs(this);
#endif
#if AP_RANGEFINDER_DRONECAN_ENABLED
    AP_RangeFinder_DroneCAN::subscribe_msgs(this);
#endif
#if AP_RCPROTOCOL_DRONECAN_ENABLED
    AP_RCProtocol_DroneCAN::subscribe_msgs(this);
#endif
#if AP_EFI_DRONECAN_ENABLED
    AP_EFI_DroneCAN::subscribe_msgs(this);
#endif

#if AP_PROXIMITY_DRONECAN_ENABLED
    AP_Proximity_DroneCAN::subscribe_msgs(this);
#endif
#if HAL_MOUNT_XACTI_ENABLED
    AP_Mount_Xacti::subscribe_msgs(this);
#endif

    act_out_array.set_timeout_ms(5);
    act_out_array.set_priority(CANARD_TRANSFER_PRIORITY_HIGH);

    esc_raw.set_timeout_ms(2);
    // esc_raw is one higher than high priority to ensure that it is given higher priority over act_out_array
    esc_raw.set_priority(CANARD_TRANSFER_PRIORITY_HIGH - 1);

#if AP_DRONECAN_HOBBYWING_ESC_SUPPORT
    esc_hobbywing_raw.set_timeout_ms(2);
    esc_hobbywing_raw.set_priority(CANARD_TRANSFER_PRIORITY_HIGH);
#endif
    
    himark_out.set_timeout_ms(2);
    himark_out.set_priority(CANARD_TRANSFER_PRIORITY_HIGH);

    rgb_led.set_timeout_ms(20);
    rgb_led.set_priority(CANARD_TRANSFER_PRIORITY_LOW);

    buzzer.set_timeout_ms(20);
    buzzer.set_priority(CANARD_TRANSFER_PRIORITY_LOW);

    safety_state.set_timeout_ms(20);
    safety_state.set_priority(CANARD_TRANSFER_PRIORITY_LOW);

    arming_status.set_timeout_ms(20);
    arming_status.set_priority(CANARD_TRANSFER_PRIORITY_LOW);

#if AP_DRONECAN_SEND_GPS
    gnss_fix2.set_timeout_ms(20);
    gnss_fix2.set_priority(CANARD_TRANSFER_PRIORITY_LOW);

    gnss_auxiliary.set_timeout_ms(20);
    gnss_auxiliary.set_priority(CANARD_TRANSFER_PRIORITY_LOW);

    gnss_heading.set_timeout_ms(20);
    gnss_heading.set_priority(CANARD_TRANSFER_PRIORITY_LOW);

    gnss_status.set_timeout_ms(20);
    gnss_status.set_priority(CANARD_TRANSFER_PRIORITY_LOW);
#endif

    rtcm_stream.set_timeout_ms(20);
    rtcm_stream.set_priority(CANARD_TRANSFER_PRIORITY_LOW);

    notify_state.set_timeout_ms(20);
    notify_state.set_priority(CANARD_TRANSFER_PRIORITY_LOW);

#if HAL_MOUNT_XACTI_ENABLED
    xacti_copter_att_status.set_timeout_ms(20);
    xacti_copter_att_status.set_priority(CANARD_TRANSFER_PRIORITY_LOW);
    xacti_gimbal_control_data.set_timeout_ms(20);
    xacti_gimbal_control_data.set_priority(CANARD_TRANSFER_PRIORITY_LOW);
    xacti_gnss_status.set_timeout_ms(20);
    xacti_gnss_status.set_priority(CANARD_TRANSFER_PRIORITY_LOW);
#endif

    param_save_client.set_timeout_ms(20);
    param_save_client.set_priority(CANARD_TRANSFER_PRIORITY_LOW);

    param_get_set_client.set_timeout_ms(20);
    param_get_set_client.set_priority(CANARD_TRANSFER_PRIORITY_LOW);

    node_status.set_priority(CANARD_TRANSFER_PRIORITY_LOWEST);
    node_status.set_timeout_ms(1000);

    protocol_stats.set_priority(CANARD_TRANSFER_PRIORITY_LOWEST);
    protocol_stats.set_timeout_ms(3000);

    can_stats.set_priority(CANARD_TRANSFER_PRIORITY_LOWEST);
    can_stats.set_timeout_ms(3000);

    rgb_led.set_timeout_ms(20);
    rgb_led.set_priority(CANARD_TRANSFER_PRIORITY_LOW);
    
    node_info_server.set_timeout_ms(20);

    // setup node status
    node_status_msg.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status_msg.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    node_status_msg.sub_mode = 0;

    // Spin node for device discovery
    for (uint8_t i = 0; i < 5; i++) {
        send_node_status();
        canard_iface.process(1000);
    }

    hal.util->snprintf(_thread_name, sizeof(_thread_name), "dronecan_%u", driver_index);

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_DroneCAN::loop, void), _thread_name, DRONECAN_STACK_SIZE, AP_HAL::Scheduler::PRIORITY_CAN, 0)) {
        debug_dronecan(AP_CANManager::LOG_ERROR, "DroneCAN: couldn't create thread\n\r");
        return;
    }

    _initialized = true;
    debug_dronecan(AP_CANManager::LOG_INFO, "DroneCAN: init done\n\r");
}

void AP_DroneCAN::loop(void)
{
    while (true) {
        if (!_initialized) {
            hal.scheduler->delay_microseconds(1000);
            continue;
        }

        canard_iface.process(1);

        safety_state_send();
        notify_state_send();
        check_parameter_callback_timeout();
        send_parameter_request();
        send_parameter_save_request();
        send_node_status();
        _dna_server.verify_nodes();

#if AP_DRONECAN_SEND_GPS
        if (option_is_set(AP_DroneCAN::Options::SEND_GNSS) && !AP_GPS_DroneCAN::instance_exists(this)) {
            // send if enabled and this interface/driver is not used by the AP_GPS driver
            gnss_send_fix();
            gnss_send_yaw();
        }
#endif
        logging();
#if AP_DRONECAN_HOBBYWING_ESC_SUPPORT
        hobbywing_ESC_update();
#endif
        if (_SRV_armed_mask != 0) {
            // we have active servos
            uint32_t now = AP_HAL::micros();
            const uint32_t servo_period_us = 1000000UL / unsigned(_servo_rate_hz.get());
            if (now - _SRV_last_send_us >= servo_period_us) {
                _SRV_last_send_us = now;
                if (option_is_set(Options::USE_HIMARK_SERVO)) {
                    SRV_send_himark();
                } else {
                    SRV_send_actuator();
                }
                for (uint8_t i = 0; i < DRONECAN_SRV_NUMBER; i++) {
                    _SRV_conf[i].servo_pending = false;
                }
            }
        }
    }
}

#if AP_DRONECAN_HOBBYWING_ESC_SUPPORT
void AP_DroneCAN::hobbywing_ESC_update(void)
{
    uint32_t now = AP_HAL::millis();
    if (now - hobbywing.last_GetId_send_ms >= 1000U) {
        hobbywing.last_GetId_send_ms = now;
        com_hobbywing_esc_GetEscID msg;
        msg.payload.len = 1;
        msg.payload.data[0] = 0;
        esc_hobbywing_GetEscID.broadcast(msg);
    }
}

/*
  handle hobbywing GetEscID reply. This gets us the mapping between CAN NodeID and throttle channel
 */
void AP_DroneCAN::handle_hobbywing_GetEscID(const CanardRxTransfer& transfer, const com_hobbywing_esc_GetEscID& msg)
{
    if (msg.payload.len == 2 &&
        msg.payload.data[0] == transfer.source_node_id) {
        // throttle channel is 2nd payload byte
        const uint8_t thr_channel = msg.payload.data[1];
        if (thr_channel > 0 && thr_channel <= HOBBYWING_MAX_ESC) {
            hobbywing.thr_chan[thr_channel-1] = transfer.source_node_id;
        }
    }
}

/*
  find the ESC index given a CAN node ID
 */
bool AP_DroneCAN::hobbywing_find_esc_index(uint8_t node_id, uint8_t &esc_index) const
{
    for (uint8_t i=0; i<HOBBYWING_MAX_ESC; i++) {
        if (hobbywing.thr_chan[i] == node_id) {
            const uint8_t esc_offset = constrain_int16(_esc_offset.get(), 0, DRONECAN_SRV_NUMBER);
            esc_index = i + esc_offset;
            return true;
        }
    }
    return false;
}

/*
  handle hobbywing StatusMsg1 reply
 */
void AP_DroneCAN::handle_hobbywing_StatusMsg1(const CanardRxTransfer& transfer, const com_hobbywing_esc_StatusMsg1& msg)
{
    uint8_t esc_index;
    if (hobbywing_find_esc_index(transfer.source_node_id, esc_index)) {
        update_rpm(esc_index, msg.rpm);
    }
}

/*
  handle hobbywing StatusMsg2 reply
 */
void AP_DroneCAN::handle_hobbywing_StatusMsg2(const CanardRxTransfer& transfer, const com_hobbywing_esc_StatusMsg2& msg)
{
    uint8_t esc_index;
    if (hobbywing_find_esc_index(transfer.source_node_id, esc_index)) {
        TelemetryData t {
            .temperature_cdeg = int16_t(msg.temperature*100),
            .voltage = msg.input_voltage*0.1f,
            .current = msg.current*0.1f,
        };
        update_telem_data(esc_index, t,
                          AP_ESC_Telem_Backend::TelemetryType::CURRENT|
                          AP_ESC_Telem_Backend::TelemetryType::VOLTAGE|
                          AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE);
    }

}
#endif // AP_DRONECAN_HOBBYWING_ESC_SUPPORT

void AP_DroneCAN::send_node_status(void)
{
    const uint32_t now = AP_HAL::millis();
    if (now - _node_status_last_send_ms < 1000) {
        return;
    }
    _node_status_last_send_ms = now;
    node_status_msg.uptime_sec = now / 1000;
    node_status.broadcast(node_status_msg);

    if (option_is_set(Options::ENABLE_STATS)) {
        // also send protocol and can stats
        protocol_stats.broadcast(canard_iface.protocol_stats);

        // get can stats
        for (uint8_t i=0; i<canard_iface.num_ifaces; i++) {
            if (canard_iface.ifaces[i] == nullptr) {
                continue;
            }
            auto* iface = hal.can[0]; 
            for (uint8_t j=0; j<HAL_NUM_CAN_IFACES; j++) {
                if (hal.can[j] == canard_iface.ifaces[i]) {
                    iface = hal.can[j];
                    break;
                }
            }
            auto* bus_stats = iface->get_statistics();
            if (bus_stats == nullptr) {
                continue;
            }
            dronecan_protocol_CanStats can_stats_msg;
            can_stats_msg.interface = i;
            can_stats_msg.tx_requests = bus_stats->tx_requests;
            can_stats_msg.tx_rejected = bus_stats->tx_rejected;
            can_stats_msg.tx_overflow = bus_stats->tx_overflow;
            can_stats_msg.tx_success = bus_stats->tx_success;
            can_stats_msg.tx_timedout = bus_stats->tx_timedout;
            can_stats_msg.tx_abort = bus_stats->tx_abort;
            can_stats_msg.rx_received = bus_stats->rx_received;
            can_stats_msg.rx_overflow = bus_stats->rx_overflow;
            can_stats_msg.rx_errors = bus_stats->rx_errors;
            can_stats_msg.busoff_errors = bus_stats->num_busoff_err;
            can_stats.broadcast(can_stats_msg);
        }
    }
}

void AP_DroneCAN::handle_node_info_request(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoRequest& req)
{
    node_info_rsp.status = node_status_msg;
    node_info_rsp.status.uptime_sec = AP_HAL::millis() / 1000;

    node_info_server.respond(transfer, node_info_rsp);
}

int16_t AP_DroneCAN::scale_esc_output(uint8_t idx){
    static const int16_t cmd_max = ((1<<13)-1);
    float scaled = 0;

    //Check if this channel has a reversible ESC. If it does, we can send negative commands.
    if ((((uint32_t) 1) << idx) & _esc_rv) {
        scaled = cmd_max * (hal.rcout->scale_esc_to_unity(_SRV_conf[idx].pulse));
    } else {
        scaled = cmd_max * (hal.rcout->scale_esc_to_unity(_SRV_conf[idx].pulse) + 1.0) / 2.0;
        scaled = constrain_float(scaled, 0, cmd_max);
    }

    return static_cast<int16_t>(scaled);
}

///// SRV output /////

void AP_DroneCAN::SRV_send_actuator(void)
{
    uint8_t starting_servo = 0;
    bool repeat_send;

    WITH_SEMAPHORE(SRV_sem);

    do {
        repeat_send = false;
        uavcan_equipment_actuator_ArrayCommand msg;

        uint8_t i;
        // DroneCAN can hold maximum of 15 commands in one frame
        for (i = 0; starting_servo < DRONECAN_SRV_NUMBER && i < 15; starting_servo++) {
            uavcan_equipment_actuator_Command cmd;

            /*
             * Servo output uses a range of 1000-2000 PWM for scaling.
             * This converts output PWM from [1000:2000] range to [-1:1] range that
             * is passed to servo as unitless type via DroneCAN.
             * This approach allows for MIN/TRIM/MAX values to be used fully on
             * autopilot side and for servo it should have the setup to provide maximum
             * physically possible throws at [-1:1] limits.
             */

            if (_SRV_conf[starting_servo].servo_pending && ((((uint32_t) 1) << starting_servo) & _SRV_armed_mask)) {
                cmd.actuator_id = starting_servo + 1;

                if (option_is_set(Options::USE_ACTUATOR_PWM)) {
                    cmd.command_type = UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_PWM;
                    cmd.command_value = _SRV_conf[starting_servo].pulse;
                } else {
                    cmd.command_type = UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_UNITLESS;
                    cmd.command_value = constrain_float(((float) _SRV_conf[starting_servo].pulse - 1000.0) / 500.0 - 1.0, -1.0, 1.0);
                }

                msg.commands.data[i] = cmd;

                i++;
            }
        }
        msg.commands.len = i;
        if (i > 0) {
            if (act_out_array.broadcast(msg) > 0) {
                _srv_send_count++;
            } else {
                _fail_send_count++;
            }

            if (i == 15) {
                repeat_send = true;
            }
        }
    } while (repeat_send);
}

/*
  Himark servo output. This uses com.himark.servo.ServoCmd packets
 */
void AP_DroneCAN::SRV_send_himark(void)
{
    WITH_SEMAPHORE(SRV_sem);

    // ServoCmd can hold maximum of 17 commands. First find the highest pending servo < 17
    int8_t highest_to_send = -1;
    for (int8_t i = 16; i >= 0; i--) {
        if (_SRV_conf[i].servo_pending && ((1U<<i) & _SRV_armed_mask) != 0) {
            highest_to_send = i;
            break;
        }
    }
    if (highest_to_send == -1) {
        // nothing to send
        return;
    }
    com_himark_servo_ServoCmd msg {};

    for (uint8_t i = 0; i <= highest_to_send; i++) {
        if ((1U<<i) & _SRV_armed_mask) {
            const uint16_t pulse = constrain_int16(_SRV_conf[i].pulse - 1000, 0, 1000);
            msg.cmd.data[i] = pulse;
        }
    }
    msg.cmd.len = highest_to_send+1;

    himark_out.broadcast(msg);
}

void AP_DroneCAN::SRV_send_esc(void)
{
    uavcan_equipment_esc_RawCommand esc_msg;

    uint8_t active_esc_num = 0, max_esc_num = 0;
    uint8_t k = 0;

    // esc offset allows for efficient packing of higher ESC numbers in RawCommand
    const uint8_t esc_offset = constrain_int16(_esc_offset.get(), 0, DRONECAN_SRV_NUMBER);

    // find out how many esc we have enabled and if they are active at all
    for (uint8_t i = esc_offset; i < DRONECAN_SRV_NUMBER; i++) {
        if ((((uint32_t) 1) << i) & _ESC_armed_mask) {
            max_esc_num = i + 1;
            if (_SRV_conf[i].esc_pending) {
                active_esc_num++;
            }
        }
    }

    // if at least one is active (update) we need to send to all
    if (active_esc_num > 0) {
        k = 0;

        for (uint8_t i = esc_offset; i < max_esc_num && k < 20; i++) {
            if ((((uint32_t) 1) << i) & _ESC_armed_mask) {
                esc_msg.cmd.data[k] = scale_esc_output(i);
            } else {
                esc_msg.cmd.data[k] = static_cast<unsigned>(0);
            }

            k++;
        }
        esc_msg.cmd.len = k;

        if (esc_raw.broadcast(esc_msg)) {
            _esc_send_count++;
        } else {
            _fail_send_count++;
        }
        // immediately push data to CAN bus
        canard_iface.processTx(true);
    }

    for (uint8_t i = 0; i < DRONECAN_SRV_NUMBER; i++) {
        _SRV_conf[i].esc_pending = false;
    }
}

#if AP_DRONECAN_HOBBYWING_ESC_SUPPORT
/*
  support for Hobbywing DroneCAN ESCs
 */
void AP_DroneCAN::SRV_send_esc_hobbywing(void)
{
    com_hobbywing_esc_RawCommand esc_msg;

    uint8_t active_esc_num = 0, max_esc_num = 0;
    uint8_t k = 0;

    // esc offset allows for efficient packing of higher ESC numbers in RawCommand
    const uint8_t esc_offset = constrain_int16(_esc_offset.get(), 0, DRONECAN_SRV_NUMBER);

    // find out how many esc we have enabled and if they are active at all
    for (uint8_t i = esc_offset; i < DRONECAN_SRV_NUMBER; i++) {
        if ((((uint32_t) 1) << i) & _ESC_armed_mask) {
            max_esc_num = i + 1;
            if (_SRV_conf[i].esc_pending) {
                active_esc_num++;
            }
        }
    }

    // if at least one is active (update) we need to send to all
    if (active_esc_num > 0) {
        k = 0;

        for (uint8_t i = esc_offset; i < max_esc_num && k < 20; i++) {
            if ((((uint32_t) 1) << i) & _ESC_armed_mask) {
                esc_msg.command.data[k] = scale_esc_output(i);
            } else {
                esc_msg.command.data[k] = static_cast<unsigned>(0);
            }

            k++;
        }
        esc_msg.command.len = k;

        if (esc_hobbywing_raw.broadcast(esc_msg)) {
            _esc_send_count++;
        } else {
            _fail_send_count++;
        }
        // immediately push data to CAN bus
        canard_iface.processTx(true);
    }
}
#endif // AP_DRONECAN_HOBBYWING_ESC_SUPPORT

void AP_DroneCAN::SRV_push_servos()
{
    WITH_SEMAPHORE(SRV_sem);

    for (uint8_t i = 0; i < DRONECAN_SRV_NUMBER; i++) {
        // Check if this channels has any function assigned
        if (SRV_Channels::channel_function(i) >= SRV_Channel::k_none) {
            _SRV_conf[i].pulse = SRV_Channels::srv_channel(i)->get_output_pwm();
            _SRV_conf[i].esc_pending = true;
            _SRV_conf[i].servo_pending = true;
        }
    }

    uint32_t servo_armed_mask = _servo_bm;
    uint32_t esc_armed_mask = _esc_bm;
    const bool safety_off = hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED;
    if (!safety_off) {
        AP_BoardConfig *boardconfig = AP_BoardConfig::get_singleton();
        if (boardconfig != nullptr) {
            const uint32_t safety_mask = boardconfig->get_safety_mask();
            servo_armed_mask &= safety_mask;
            esc_armed_mask &= safety_mask;
        } else {
            servo_armed_mask = 0;
            esc_armed_mask = 0;
        }
    }
    _SRV_armed_mask = servo_armed_mask;
    _ESC_armed_mask = esc_armed_mask;

    if (_ESC_armed_mask != 0) {
        // push ESCs as fast as we can
#if AP_DRONECAN_HOBBYWING_ESC_SUPPORT
        if (option_is_set(Options::USE_HOBBYWING_ESC)) {
            SRV_send_esc_hobbywing();
        } else
#endif
        {
            SRV_send_esc();
        }
    }
}

// notify state send
void AP_DroneCAN::notify_state_send()
{
    uint32_t now = AP_HAL::millis();

    if (_notify_state_hz == 0 || (now - _last_notify_state_ms) < uint32_t(1000 / _notify_state_hz)) {
        return;
    }

    ardupilot_indication_NotifyState msg;
    msg.vehicle_state = 0;
    if (AP_Notify::flags.initialising) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_INITIALISING;
    }
    if (AP_Notify::flags.armed) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_ARMED;
    }
    if (AP_Notify::flags.flying) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_FLYING;
    }
    if (AP_Notify::flags.compass_cal_running) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_MAGCAL_RUN;
    }
    if (AP_Notify::flags.ekf_bad) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_EKF_BAD;
    }
    if (AP_Notify::flags.esc_calibration) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_ESC_CALIBRATION;
    }
    if (AP_Notify::flags.failsafe_battery) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_FAILSAFE_BATT;
    }
    if (AP_Notify::flags.failsafe_gcs) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_FAILSAFE_GCS;
    }
    if (AP_Notify::flags.failsafe_radio) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_FAILSAFE_RADIO;
    }
    if (AP_Notify::flags.firmware_update) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_FW_UPDATE;
    }
    if (AP_Notify::flags.gps_fusion) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_GPS_FUSION;
    }
    if (AP_Notify::flags.gps_glitching) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_GPS_GLITCH;
    }
    if (AP_Notify::flags.have_pos_abs) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_POS_ABS_AVAIL;
    }
    if (AP_Notify::flags.leak_detected) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_LEAK_DET;
    }
    if (AP_Notify::flags.parachute_release) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_CHUTE_RELEASED;
    }
    if (AP_Notify::flags.powering_off) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_POWERING_OFF;
    }
    if (AP_Notify::flags.pre_arm_check) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_PREARM;
    }
    if (AP_Notify::flags.pre_arm_gps_check) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_PREARM_GPS;
    }
    if (AP_Notify::flags.save_trim) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_SAVE_TRIM;
    }
    if (AP_Notify::flags.vehicle_lost) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_LOST;
    }
    if (AP_Notify::flags.video_recording) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_VIDEO_RECORDING;
    }
    if (AP_Notify::flags.waiting_for_throw) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_THROW_READY;
    }

#ifndef HAL_BUILD_AP_PERIPH
    const AP_Vehicle* vehicle = AP::vehicle();
    if (vehicle != nullptr) {
        if (vehicle->is_landing()) {
            msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_IS_LANDING;
        }
        if (vehicle->is_taking_off()) {
            msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_IS_TAKING_OFF;
        }
    }
#endif // HAL_BUILD_AP_PERIPH

    msg.aux_data_type = ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_YAW_EARTH_CENTIDEGREES;
    uint16_t yaw_cd = (uint16_t)(360.0f - degrees(AP::ahrs().get_yaw()))*100.0f;
    const uint8_t *data = (uint8_t *)&yaw_cd;
    for (uint8_t i=0; i<2; i++) {
        msg.aux_data.data[i] = data[i];
    }
    msg.aux_data.len = 2;
    notify_state.broadcast(msg);
    _last_notify_state_ms = AP_HAL::millis();
}

#if AP_DRONECAN_SEND_GPS
void AP_DroneCAN::gnss_send_fix()
{
    const AP_GPS &gps = AP::gps();

    const uint32_t gps_lib_time_ms = gps.last_message_time_ms();
    if (_gnss.last_gps_lib_fix_ms == gps_lib_time_ms) {
        return;
    }
    _gnss.last_gps_lib_fix_ms = gps_lib_time_ms;

    /*
        send Fix2 packet
    */

    uavcan_equipment_gnss_Fix2 pkt {};
    const Location &loc = gps.location();
    const Vector3f &vel = gps.velocity();

    pkt.timestamp.usec = AP_HAL::micros64();
    pkt.gnss_timestamp.usec = gps.time_epoch_usec();
    if (pkt.gnss_timestamp.usec == 0) {
        pkt.gnss_time_standard = UAVCAN_EQUIPMENT_GNSS_FIX2_GNSS_TIME_STANDARD_NONE;
    } else {
        pkt.gnss_time_standard = UAVCAN_EQUIPMENT_GNSS_FIX2_GNSS_TIME_STANDARD_UTC;
    }
    pkt.longitude_deg_1e8 = uint64_t(loc.lng) * 10ULL;
    pkt.latitude_deg_1e8 = uint64_t(loc.lat) * 10ULL;
    pkt.height_ellipsoid_mm = loc.alt * 10;
    pkt.height_msl_mm = loc.alt * 10;
    for (uint8_t i=0; i<3; i++) {
        pkt.ned_velocity[i] = vel[i];
    }
    pkt.sats_used = gps.num_sats();
    switch (gps.status()) {
    case AP_GPS::GPS_Status::NO_GPS:
    case AP_GPS::GPS_Status::NO_FIX:
        pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_NO_FIX;
        pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_SINGLE;
        pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_DGPS_OTHER;
        break;
    case AP_GPS::GPS_Status::GPS_OK_FIX_2D:
        pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_2D_FIX;
        pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_SINGLE;
        pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_DGPS_OTHER;
        break;
    case AP_GPS::GPS_Status::GPS_OK_FIX_3D:
        pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX;
        pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_SINGLE;
        pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_DGPS_OTHER;
        break;
    case AP_GPS::GPS_Status::GPS_OK_FIX_3D_DGPS:
        pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX;
        pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_DGPS;
        pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_DGPS_SBAS;
        break;
    case AP_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FLOAT:
        pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX;
        pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK;
        pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FLOAT;
        break;
    case AP_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FIXED:
        pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX;
        pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK;
        pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FIXED;
        break;
    }

    pkt.covariance.len = 6;
    float hacc;
    if (gps.horizontal_accuracy(hacc)) {
        pkt.covariance.data[0] = pkt.covariance.data[1] = sq(hacc);
    }
    float vacc;
    if (gps.vertical_accuracy(vacc)) {
        pkt.covariance.data[2] = sq(vacc);
    }
    float sacc;
    if (gps.speed_accuracy(sacc)) {
        const float vc3 = sq(sacc);
        pkt.covariance.data[3] = pkt.covariance.data[4] = pkt.covariance.data[5] = vc3;
    }

    gnss_fix2.broadcast(pkt);



    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _gnss.last_send_status_ms >= 1000) {
        _gnss.last_send_status_ms = now_ms;

        /*
        send aux packet
        */
        uavcan_equipment_gnss_Auxiliary pkt_auxiliary {};
        pkt_auxiliary.hdop = gps.get_hdop() * 0.01;
        pkt_auxiliary.vdop = gps.get_vdop() * 0.01;

        gnss_auxiliary.broadcast(pkt_auxiliary);


        /*
        send Status packet
        */
        ardupilot_gnss_Status pkt_status {};
        pkt_status.healthy = gps.is_healthy();
        if (gps.logging_present() && gps.logging_enabled() && !gps.logging_failed()) {
            pkt_status.status |= ARDUPILOT_GNSS_STATUS_STATUS_LOGGING;
        }
        uint8_t idx; // unused
        if (pkt_status.healthy && !gps.first_unconfigured_gps(idx)) {
            pkt_status.status |= ARDUPILOT_GNSS_STATUS_STATUS_ARMABLE;
        }

        uint32_t error_codes;
        if (gps.get_error_codes(error_codes)) {
            pkt_status.error_codes = error_codes;
        }

        gnss_status.broadcast(pkt_status);
    }
}

void AP_DroneCAN::gnss_send_yaw()
{
    const AP_GPS &gps = AP::gps();

    if (!gps.have_gps_yaw()) {
        return;
    }

    float yaw_deg, yaw_acc_deg;
    uint32_t yaw_time_ms;
    if (!gps.gps_yaw_deg(yaw_deg, yaw_acc_deg, yaw_time_ms) && yaw_time_ms != _gnss.last_lib_yaw_time_ms) {
        return;
    }

    _gnss.last_lib_yaw_time_ms = yaw_time_ms;

    ardupilot_gnss_Heading pkt_heading {};
    pkt_heading.heading_valid = true;
    pkt_heading.heading_accuracy_valid = is_positive(yaw_acc_deg);
    pkt_heading.heading_rad = radians(yaw_deg);
    pkt_heading.heading_accuracy_rad = radians(yaw_acc_deg);

    gnss_heading.broadcast(pkt_heading);
}
#endif // AP_DRONECAN_SEND_GPS

// SafetyState send
void AP_DroneCAN::safety_state_send()
{
    uint32_t now = AP_HAL::millis();
    if (now - _last_safety_state_ms < 500) {
        // update at 2Hz
        return;
    }
    _last_safety_state_ms = now;

    { // handle SafetyState
        ardupilot_indication_SafetyState safety_msg;
        auto state = hal.util->safety_switch_state();
        if (_SRV_armed_mask != 0 || _ESC_armed_mask != 0) {
            // if we are outputting any servos or ESCs due to
            // BRD_SAFETY_MASK then we need to advertise safety as
            // off, this changes LEDs to indicate unsafe and allows
            // AP_Periph ESCs and servos to run
            state = AP_HAL::Util::SAFETY_ARMED;
        }
        switch (state) {
        case AP_HAL::Util::SAFETY_ARMED:
            safety_msg.status = ARDUPILOT_INDICATION_SAFETYSTATE_STATUS_SAFETY_OFF;
            safety_state.broadcast(safety_msg);
            break;
        case AP_HAL::Util::SAFETY_DISARMED:
            safety_msg.status = ARDUPILOT_INDICATION_SAFETYSTATE_STATUS_SAFETY_ON;
            safety_state.broadcast(safety_msg);
            break;
        default:
            // nothing to send
            break;
        }
    }

    { // handle ArmingStatus
        uavcan_equipment_safety_ArmingStatus arming_msg;
        arming_msg.status = hal.util->get_soft_armed() ? UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_STATUS_FULLY_ARMED :
                                                      UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_STATUS_DISARMED;
        arming_status.broadcast(arming_msg);
    }
}

/*
  handle Button message
 */
void AP_DroneCAN::handle_button(const CanardRxTransfer& transfer, const ardupilot_indication_Button& msg)
{
    switch (msg.button) {
    case ARDUPILOT_INDICATION_BUTTON_BUTTON_SAFETY: {
        AP_BoardConfig *brdconfig = AP_BoardConfig::get_singleton();
        if (brdconfig && brdconfig->safety_button_handle_pressed(msg.press_time)) {
            AP_HAL::Util::safety_state state = hal.util->safety_switch_state();
            if (state == AP_HAL::Util::SAFETY_ARMED) {
                hal.rcout->force_safety_on();
            } else {
                hal.rcout->force_safety_off();
            }
        }
        break;
    }
    }
}

/*
  handle traffic report
 */
void AP_DroneCAN::handle_traffic_report(const CanardRxTransfer& transfer, const ardupilot_equipment_trafficmonitor_TrafficReport& msg)
{
#if HAL_ADSB_ENABLED
    AP_ADSB *adsb = AP::ADSB();
    if (!adsb || !adsb->enabled()) {
        // ADSB not enabled
        return;
    }

    AP_ADSB::adsb_vehicle_t vehicle;
    mavlink_adsb_vehicle_t &pkt = vehicle.info;

    pkt.ICAO_address = msg.icao_address;
    pkt.tslc = msg.tslc;
    pkt.lat = msg.latitude_deg_1e7;
    pkt.lon = msg.longitude_deg_1e7;
    pkt.altitude = msg.alt_m * 1000;
    pkt.heading = degrees(msg.heading) * 100;
    pkt.hor_velocity = norm(msg.velocity[0], msg.velocity[1]) * 100;
    pkt.ver_velocity = -msg.velocity[2] * 100;
    pkt.squawk = msg.squawk;
    for (uint8_t i=0; i<9; i++) {
        pkt.callsign[i] = msg.callsign[i];
    }
    pkt.emitter_type = msg.traffic_type;

    if (msg.alt_type == ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_ALT_TYPE_PRESSURE_AMSL) {
        pkt.flags |= ADSB_FLAGS_VALID_ALTITUDE;
        pkt.altitude_type = ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
    } else if (msg.alt_type == ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_ALT_TYPE_WGS84) {
        pkt.flags |= ADSB_FLAGS_VALID_ALTITUDE;
        pkt.altitude_type = ADSB_ALTITUDE_TYPE_GEOMETRIC;
    }

    if (msg.lat_lon_valid) {
        pkt.flags |= ADSB_FLAGS_VALID_COORDS;
    }
    if (msg.heading_valid) {
        pkt.flags |= ADSB_FLAGS_VALID_HEADING;
    }
    if (msg.velocity_valid) {
        pkt.flags |= ADSB_FLAGS_VALID_VELOCITY;
    }
    if (msg.callsign_valid) {
        pkt.flags |= ADSB_FLAGS_VALID_CALLSIGN;
    }
    if (msg.ident_valid) {
        pkt.flags |= ADSB_FLAGS_VALID_SQUAWK;
    }
    if (msg.simulated_report) {
        pkt.flags |= ADSB_FLAGS_SIMULATED;
    }
    if (msg.vertical_velocity_valid) {
        pkt.flags |= ADSB_FLAGS_VERTICAL_VELOCITY_VALID;
    }
    if (msg.baro_valid) {
        pkt.flags |= ADSB_FLAGS_BARO_VALID;
    }

    vehicle.last_update_ms = AP_HAL::millis() - (vehicle.info.tslc * 1000);
    adsb->handle_adsb_vehicle(vehicle);
#endif
}

/*
  handle actuator status message
 */
void AP_DroneCAN::handle_actuator_status(const CanardRxTransfer& transfer, const uavcan_equipment_actuator_Status& msg)
{
    // log as CSRV message
    AP::logger().Write_ServoStatus(AP_HAL::micros64(),
                                   msg.actuator_id,
                                   msg.position,
                                   msg.force,
                                   msg.speed,
                                   msg.power_rating_pct,
                                   0, 0, 0, 0, 0, 0);
}

/*
  handle himark ServoInfo message
 */
void AP_DroneCAN::handle_himark_servoinfo(const CanardRxTransfer& transfer, const com_himark_servo_ServoInfo &msg)
{
    // log as CSRV message
    AP::logger().Write_ServoStatus(AP_HAL::micros64(),
                                   msg.servo_id,
                                   msg.pos_sensor*0.01,
                                   0,
                                   0,
                                   0,
                                   msg.pos_cmd*0.01,
                                   msg.voltage*0.01,
                                   msg.current*0.01,
                                   msg.motor_temp*0.2-40,
                                   msg.pcb_temp*0.2-40,
                                   msg.error_status);
}

#if AP_DRONECAN_VOLZ_FEEDBACK_ENABLED
void AP_DroneCAN::handle_actuator_status_Volz(const CanardRxTransfer& transfer, const com_volz_servo_ActuatorStatus& msg)
{
    AP::logger().WriteStreaming(
        "CVOL",
        "TimeUS,Id,Pos,Cur,V,Pow,T",
        "s#dAv%O",
        "F-00000",
        "QBfffBh",
        AP_HAL::micros64(),
        msg.actuator_id,
        ToDeg(msg.actual_position),
        msg.current * 0.025f,
        msg.voltage * 0.2f,
        msg.motor_pwm * (100.0/255.0),
        int16_t(msg.motor_temperature) - 50);
}
#endif

/*
  handle ESC status message
 */
void AP_DroneCAN::handle_ESC_status(const CanardRxTransfer& transfer, const uavcan_equipment_esc_Status& msg)
{
#if HAL_WITH_ESC_TELEM
    const uint8_t esc_offset = constrain_int16(_esc_offset.get(), 0, DRONECAN_SRV_NUMBER);
    const uint8_t esc_index = msg.esc_index + esc_offset;

    if (!is_esc_data_index_valid(esc_index)) {
        return;
    }

    TelemetryData t {
        .temperature_cdeg = int16_t((KELVIN_TO_C(msg.temperature)) * 100),
        .voltage = msg.voltage,
        .current = msg.current,
    };

    update_rpm(esc_index, msg.rpm, msg.error_count);
    update_telem_data(esc_index, t,
        (isnan(msg.current) ? 0 : AP_ESC_Telem_Backend::TelemetryType::CURRENT)
            | (isnan(msg.voltage) ? 0 : AP_ESC_Telem_Backend::TelemetryType::VOLTAGE)
            | (isnan(msg.temperature) ? 0 : AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE));
#endif
}

bool AP_DroneCAN::is_esc_data_index_valid(const uint8_t index) {
    if (index > DRONECAN_SRV_NUMBER) {
        // printf("DroneCAN: invalid esc index: %d. max index allowed: %d\n\r", index, DRONECAN_SRV_NUMBER);
        return false;
    }
    return true;
}

/*
  handle LogMessage debug
 */
void AP_DroneCAN::handle_debug(const CanardRxTransfer& transfer, const uavcan_protocol_debug_LogMessage& msg)
{
#if HAL_LOGGING_ENABLED
    if (AP::can().get_log_level() != AP_CANManager::LOG_NONE) {
        // log to onboard log and mavlink
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CAN[%u] %s", transfer.source_node_id, msg.text.data);
    } else {
        // only log to onboard log
        AP::logger().Write_MessageF("CAN[%u] %s", transfer.source_node_id, msg.text.data);
    }
#endif
}

/*
 check for parameter get/set response timeout
*/
void AP_DroneCAN::check_parameter_callback_timeout()
{
    WITH_SEMAPHORE(_param_sem);

    // return immediately if not waiting for get/set parameter response
    if (param_request_sent_ms == 0) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - param_request_sent_ms > AP_DRONECAN_GETSET_TIMEOUT_MS) {
        param_request_sent_ms = 0;
        param_int_cb = nullptr;
        param_float_cb = nullptr;
    }
}

/*
  send any queued request to get/set parameter
  called from loop
*/
void AP_DroneCAN::send_parameter_request()
{
    WITH_SEMAPHORE(_param_sem);
    if (param_request_sent) {
        return;
    }
    param_get_set_client.request(param_request_node_id, param_getset_req);
    param_request_sent = true;
}

/*
  set named float parameter on node
*/
bool AP_DroneCAN::set_parameter_on_node(uint8_t node_id, const char *name, float value, ParamGetSetFloatCb *cb)
{
    WITH_SEMAPHORE(_param_sem);

    // fail if waiting for any previous get/set request
    if (param_int_cb != nullptr ||
        param_float_cb != nullptr) {
        return false;
    }
    param_getset_req.index = 0;
    param_getset_req.name.len = strncpy_noterm((char*)param_getset_req.name.data, name, sizeof(param_getset_req.name.data)-1);
    param_getset_req.value.real_value = value;
    param_getset_req.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
    param_float_cb = cb;
    param_request_sent = false;
    param_request_sent_ms = AP_HAL::millis();
    param_request_node_id = node_id;
    return true;
}

/*
  set named integer parameter on node
*/
bool AP_DroneCAN::set_parameter_on_node(uint8_t node_id, const char *name, int32_t value, ParamGetSetIntCb *cb)
{
    WITH_SEMAPHORE(_param_sem);

    // fail if waiting for any previous get/set request
    if (param_int_cb != nullptr ||
        param_float_cb != nullptr) {
        return false;
    }
    param_getset_req.index = 0;
    param_getset_req.name.len = strncpy_noterm((char*)param_getset_req.name.data, name, sizeof(param_getset_req.name.data)-1);
    param_getset_req.value.integer_value = value;
    param_getset_req.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
    param_int_cb = cb;
    param_request_sent = false;
    param_request_sent_ms = AP_HAL::millis();
    param_request_node_id = node_id;
    return true;
}

/*
  get named float parameter on node
*/
bool AP_DroneCAN::get_parameter_on_node(uint8_t node_id, const char *name, ParamGetSetFloatCb *cb)
{
    WITH_SEMAPHORE(_param_sem);

    // fail if waiting for any previous get/set request
    if (param_int_cb != nullptr ||
        param_float_cb != nullptr) {
        return false;
    }
    param_getset_req.index = 0;
    param_getset_req.name.len = strncpy_noterm((char*)param_getset_req.name.data, name, sizeof(param_getset_req.name.data));
    param_getset_req.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY;
    param_float_cb = cb;
    param_request_sent = false;
    param_request_sent_ms = AP_HAL::millis();
    param_request_node_id = node_id;
    return true;
}

/*
  get named integer parameter on node
*/
bool AP_DroneCAN::get_parameter_on_node(uint8_t node_id, const char *name, ParamGetSetIntCb *cb)
{
    WITH_SEMAPHORE(_param_sem);

    // fail if waiting for any previous get/set request
    if (param_int_cb != nullptr ||
        param_float_cb != nullptr) {
        return false;
    }
    param_getset_req.index = 0;
    param_getset_req.name.len = strncpy_noterm((char*)param_getset_req.name.data, name, sizeof(param_getset_req.name.data));
    param_getset_req.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY;
    param_int_cb = cb;
    param_request_sent = false;
    param_request_sent_ms = AP_HAL::millis();
    param_request_node_id = node_id;
    return true;
}

void AP_DroneCAN::handle_param_get_set_response(const CanardRxTransfer& transfer, const uavcan_protocol_param_GetSetResponse& rsp)
{
    WITH_SEMAPHORE(_param_sem);
    if (!param_int_cb &&
        !param_float_cb) {
        return;
    }
    if ((rsp.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE) && param_int_cb) {
        int32_t val = rsp.value.integer_value;
        if ((*param_int_cb)(this, transfer.source_node_id, (const char*)rsp.name.data, val)) {
            // we want the parameter to be set with val
            param_getset_req.index = 0;
            memcpy(param_getset_req.name.data, rsp.name.data, rsp.name.len);
            param_getset_req.value.integer_value = val;
            param_getset_req.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
            param_request_sent = false;
            param_request_sent_ms = AP_HAL::millis();
            param_request_node_id = transfer.source_node_id;
            return;
        }
    } else if ((rsp.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE) && param_float_cb) {
        float val = rsp.value.real_value;
        if ((*param_float_cb)(this, transfer.source_node_id, (const char*)rsp.name.data, val)) {
            // we want the parameter to be set with val
            param_getset_req.index = 0;
            memcpy(param_getset_req.name.data, rsp.name.data, rsp.name.len);
            param_getset_req.value.real_value = val;
            param_getset_req.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
            param_request_sent = false;
            param_request_sent_ms = AP_HAL::millis();
            param_request_node_id = transfer.source_node_id;
            return;
        }
    }
    param_request_sent_ms = 0;
    param_int_cb = nullptr;
    param_float_cb = nullptr;
}


void AP_DroneCAN::send_parameter_save_request()
{
    WITH_SEMAPHORE(_param_save_sem);
    if (param_save_request_sent) {
        return;
    }
    param_save_client.request(param_save_request_node_id, param_save_req);
    param_save_request_sent = true;
}

bool AP_DroneCAN::save_parameters_on_node(uint8_t node_id, ParamSaveCb *cb)
{
    WITH_SEMAPHORE(_param_save_sem);
    if (save_param_cb != nullptr) {
        //busy
        return false;
    }

    param_save_req.opcode = UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE;
    param_save_request_sent = false;
    param_save_request_node_id = node_id;
    save_param_cb = cb;
    return true;
}

// handle parameter save request response
void AP_DroneCAN::handle_param_save_response(const CanardRxTransfer& transfer, const uavcan_protocol_param_ExecuteOpcodeResponse& rsp)
{
    WITH_SEMAPHORE(_param_save_sem);
    if (!save_param_cb) {
        return;
    }
    (*save_param_cb)(this, transfer.source_node_id, rsp.ok);
    save_param_cb = nullptr;
}

// Send Reboot command
// Note: Do not call this from outside DroneCAN thread context,
// THIS IS NOT A THREAD SAFE API!
void AP_DroneCAN::send_reboot_request(uint8_t node_id)
{
    uavcan_protocol_RestartNodeRequest request;
    request.magic_number = UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_MAGIC_NUMBER;
    restart_node_client.request(node_id, request);
}

// check if a option is set and if it is then reset it to 0.
// return true if it was set
bool AP_DroneCAN::check_and_reset_option(Options option)
{
    bool ret = option_is_set(option);
    if (ret) {
        _options.set_and_save(int16_t(_options.get() & ~uint16_t(option)));
    }
    return ret;
}

// handle prearm check
bool AP_DroneCAN::prearm_check(char* fail_msg, uint8_t fail_msg_len) const
{
    // forward this to DNA_Server
    return _dna_server.prearm_check(fail_msg, fail_msg_len);
}

/*
  periodic logging
 */
void AP_DroneCAN::logging(void)
{
#if HAL_LOGGING_ENABLED
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_log_ms < 1000) {
        return;
    }
    last_log_ms = now_ms;
    if (HAL_NUM_CAN_IFACES <= _driver_index) {
        // no interface?
        return;
    }
    const auto *iface = hal.can[_driver_index];
    if (iface == nullptr) {
        return;
    }
    const auto *stats = iface->get_statistics();
    if (stats == nullptr) {
        // statistics not implemented on this interface
        return;
    }
    const auto &s = *stats;
    AP::logger().WriteStreaming("CANS",
                                "TimeUS,I,T,Trq,Trej,Tov,Tto,Tab,R,Rov,Rer,Bo,Etx,Stx,Ftx",
                                "s#-------------",
                                "F--------------",
                                "QBIIIIIIIIIIIII",
                                AP_HAL::micros64(),
                                _driver_index,
                                s.tx_success,
                                s.tx_requests,
                                s.tx_rejected,
                                s.tx_overflow,
                                s.tx_timedout,
                                s.tx_abort,
                                s.rx_received,
                                s.rx_overflow,
                                s.rx_errors,
                                s.num_busoff_err,
                                _esc_send_count,
                                _srv_send_count,
                                _fail_send_count);
#endif // HAL_LOGGING_ENABLED
}

#endif // HAL_NUM_CAN_IFACES
