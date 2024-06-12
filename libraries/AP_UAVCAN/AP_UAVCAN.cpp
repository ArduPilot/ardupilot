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

#if HAL_ENABLE_LIBUAVCAN_DRIVERS
#include "AP_UAVCAN.h"
#include <GCS_MAVLink/GCS.h>

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_CANManager/AP_CANManager.h>

#include <uavcan/transport/can_acceptance_filter_configurator.hpp>

#include <uavcan/equipment/actuator/ArrayCommand.hpp>
#include <uavcan/equipment/actuator/Command.hpp>
#include <uavcan/equipment/actuator/Status.hpp>

#include <uavcan/equipment/esc/RawCommand.hpp>
#include <uavcan/equipment/esc/Status.hpp>
#include <uavcan/equipment/indication/LightsCommand.hpp>
#include <uavcan/equipment/indication/SingleLightCommand.hpp>
#include <uavcan/equipment/indication/BeepCommand.hpp>
#include <uavcan/equipment/indication/RGB565.hpp>
#include <uavcan/equipment/safety/ArmingStatus.hpp>
#include <ardupilot/indication/SafetyState.hpp>
#include <ardupilot/indication/Button.hpp>
#include <ardupilot/indication/NotifyState.hpp>
#include <ardupilot/equipment/trafficmonitor/TrafficReport.hpp>
#if AP_DRONECAN_SEND_GPS
#include <uavcan/equipment/gnss/Fix2.hpp>
#include <uavcan/equipment/gnss/Auxiliary.hpp>
#include <ardupilot/gnss/Heading.hpp>
#include <ardupilot/gnss/Status.hpp>
#endif
#include <uavcan/equipment/gnss/RTCMStream.hpp>
#include <uavcan/protocol/debug/LogMessage.hpp>

#if AP_DRONECAN_HIMARK_SERVO_ENABLED
#include <com/himark/servo/ServoCmd.hpp>
#include <com/himark/servo/ServoInfo.hpp>
#endif

#if AP_DRONECAN_HOBBYWING_ESC_ENABLED
#include <com/hobbywing/esc/GetEscID.hpp>
#include <com/hobbywing/esc/StatusMsg1.hpp>
#include <com/hobbywing/esc/StatusMsg2.hpp>
#include <com/hobbywing/esc/RawCommand.hpp>
#endif

#include <AP_Arming/AP_Arming.h>
#include <AP_Baro/AP_Baro_UAVCAN.h>
#include <AP_RangeFinder/AP_RangeFinder_UAVCAN.h>
#include <AP_EFI/AP_EFI_DroneCAN.h>
#include <AP_GPS/AP_GPS_UAVCAN.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_BattMonitor/AP_BattMonitor_UAVCAN.h>
#include <AP_Compass/AP_Compass_UAVCAN.h>
#include <AP_Airspeed/AP_Airspeed_UAVCAN.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_OpticalFlow/AP_OpticalFlow_HereFlow.h>
#include <AP_ADSB/AP_ADSB.h>
#include "AP_UAVCAN_DNA_Server.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_OpenDroneID/AP_OpenDroneID.h>
#include "AP_UAVCAN_pool.h"
#include <AP_Proximity/AP_Proximity_DroneCAN.h>

#define LED_DELAY_US 50000

extern const AP_HAL::HAL& hal;

// setup default pool size
#ifndef UAVCAN_NODE_POOL_SIZE
#if HAL_CANFD_SUPPORTED
#define UAVCAN_NODE_POOL_SIZE 16384
#else
#define UAVCAN_NODE_POOL_SIZE 8192
#endif
#endif

#if HAL_CANFD_SUPPORTED
#define UAVCAN_STACK_SIZE     8192
#else
#define UAVCAN_STACK_SIZE     4096
#endif

#ifndef AP_DRONECAN_VOLZ_FEEDBACK_ENABLED
#define AP_DRONECAN_VOLZ_FEEDBACK_ENABLED 0
#endif

#if AP_DRONECAN_VOLZ_FEEDBACK_ENABLED
#include <com/volz/servo/ActuatorStatus.hpp>
#endif

#define debug_uavcan(level_debug, fmt, args...) do { AP::can().log_text(level_debug, "UAVCAN", fmt, ##args); } while (0)

// Translation of all messages from UAVCAN structures into AP structures is done
// in AP_UAVCAN and not in corresponding drivers.
// The overhead of including definitions of DSDL is very high and it is best to
// concentrate in one place.

// table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_UAVCAN::var_info[] = {
    // @Param: NODE
    // @DisplayName: UAVCAN node that is used for this network
    // @Description: UAVCAN node should be set implicitly
    // @Range: 1 250
    // @User: Advanced
    AP_GROUPINFO("NODE", 1, AP_UAVCAN, _uavcan_node, 10),

    // @Param: SRV_BM
    // @DisplayName: Output channels to be transmitted as servo over UAVCAN
    // @Description: Bitmask with one set for channel to be transmitted as a servo command over UAVCAN
    // @Bitmask: 0: Servo 1, 1: Servo 2, 2: Servo 3, 3: Servo 4, 4: Servo 5, 5: Servo 6, 6: Servo 7, 7: Servo 8, 8: Servo 9, 9: Servo 10, 10: Servo 11, 11: Servo 12, 12: Servo 13, 13: Servo 14, 14: Servo 15, 15: Servo 16, 16: Servo 17, 17: Servo 18, 18: Servo 19, 19: Servo 20, 20: Servo 21, 21: Servo 22, 22: Servo 23, 23: Servo 24, 24: Servo 25, 25: Servo 26, 26: Servo 27, 27: Servo 28, 28: Servo 29, 29: Servo 30, 30: Servo 31, 31: Servo 32

    // @User: Advanced
    AP_GROUPINFO("SRV_BM", 2, AP_UAVCAN, _servo_bm, 0),

    // @Param: ESC_BM
    // @DisplayName: Output channels to be transmitted as ESC over UAVCAN
    // @Description: Bitmask with one set for channel to be transmitted as a ESC command over UAVCAN
    // @Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32
    // @User: Advanced
    AP_GROUPINFO("ESC_BM", 3, AP_UAVCAN, _esc_bm, 0),

    // @Param: SRV_RT
    // @DisplayName: Servo output rate
    // @Description: Maximum transmit rate for servo outputs
    // @Range: 1 200
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("SRV_RT", 4, AP_UAVCAN, _servo_rate_hz, 50),

    // @Param: OPTION
    // @DisplayName: UAVCAN options
    // @Description: Option flags
    // @Bitmask: 0:ClearDNADatabase,1:IgnoreDNANodeConflicts,2:EnableCanfd,3:IgnoreDNANodeUnhealthy,4:SendServoAsPWM,5:SendGNSS,6:UseHimarkServo,7:UseHobbyWingESC
    // @User: Advanced
    AP_GROUPINFO("OPTION", 5, AP_UAVCAN, _options, 0),
    
    // @Param: NTF_RT
    // @DisplayName: Notify State rate
    // @Description: Maximum transmit rate for Notify State Message
    // @Range: 1 200
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("NTF_RT", 6, AP_UAVCAN, _notify_state_hz, 20),

    // @Param: ESC_OF
    // @DisplayName: ESC Output channels offset
    // @Description: Offset for ESC numbering in DroneCAN ESC RawCommand messages. This allows for more efficient packing of ESC command messages. If your ESCs are on servo functions 5 to 8 and you set this parameter to 4 then the ESC RawCommand will be sent with the first 4 slots filled. This can be used for more efficint usage of CAN bandwidth
    // @Range: 0 18
    // @User: Advanced
    AP_GROUPINFO("ESC_OF", 7, AP_UAVCAN, _esc_offset, 0),

    // @Param: POOL
    // @DisplayName: CAN pool size
    // @Description: Amount of memory in bytes to allocate for the DroneCAN memory pool. More memory is needed for higher CAN bus loads
    // @Range: 1024 16384
    // @User: Advanced
    AP_GROUPINFO("POOL", 8, AP_UAVCAN, _pool_size, UAVCAN_NODE_POOL_SIZE),
    
    AP_GROUPEND
};

// this is the timeout in milliseconds for periodic message types. We
// set this to 1 to minimise resend of stale msgs
#define CAN_PERIODIC_TX_TIMEOUT_MS 2

// publisher interfaces
static uavcan::Publisher<uavcan::equipment::actuator::ArrayCommand>* act_out_array[HAL_MAX_CAN_PROTOCOL_DRIVERS];
static uavcan::Publisher<uavcan::equipment::esc::RawCommand>* esc_raw[HAL_MAX_CAN_PROTOCOL_DRIVERS];
static uavcan::Publisher<uavcan::equipment::indication::LightsCommand>* rgb_led[HAL_MAX_CAN_PROTOCOL_DRIVERS];
static uavcan::Publisher<uavcan::equipment::indication::BeepCommand>* buzzer[HAL_MAX_CAN_PROTOCOL_DRIVERS];
static uavcan::Publisher<ardupilot::indication::SafetyState>* safety_state[HAL_MAX_CAN_PROTOCOL_DRIVERS];
static uavcan::Publisher<uavcan::equipment::safety::ArmingStatus>* arming_status[HAL_MAX_CAN_PROTOCOL_DRIVERS];
#if AP_DRONECAN_SEND_GPS
static uavcan::Publisher<uavcan::equipment::gnss::Fix2>* gnss_fix2[HAL_MAX_CAN_PROTOCOL_DRIVERS];
static uavcan::Publisher<uavcan::equipment::gnss::Auxiliary>* gnss_auxiliary[HAL_MAX_CAN_PROTOCOL_DRIVERS];
static uavcan::Publisher<ardupilot::gnss::Heading>* gnss_heading[HAL_MAX_CAN_PROTOCOL_DRIVERS];
static uavcan::Publisher<ardupilot::gnss::Status>* gnss_status[HAL_MAX_CAN_PROTOCOL_DRIVERS];
#endif
static uavcan::Publisher<uavcan::equipment::gnss::RTCMStream>* rtcm_stream[HAL_MAX_CAN_PROTOCOL_DRIVERS];
static uavcan::Publisher<ardupilot::indication::NotifyState>* notify_state[HAL_MAX_CAN_PROTOCOL_DRIVERS];

#if AP_DRONECAN_HOBBYWING_ESC_ENABLED
static uavcan::Publisher<com::hobbywing::esc::RawCommand>* esc_hobbywing_raw[HAL_MAX_CAN_PROTOCOL_DRIVERS];
static uavcan::Publisher<com::hobbywing::esc::GetEscID>* esc_hobbywing_GetEscID[HAL_MAX_CAN_PROTOCOL_DRIVERS];

UC_REGISTRY_BINDER(HobbywingESCIDCb, com::hobbywing::esc::GetEscID);
static uavcan::Subscriber<com::hobbywing::esc::GetEscID, HobbywingESCIDCb> *hobbywing_GetEscId_listener[HAL_MAX_CAN_PROTOCOL_DRIVERS];
UC_REGISTRY_BINDER(HobbywingStatus1Cb, com::hobbywing::esc::StatusMsg1);
static uavcan::Subscriber<com::hobbywing::esc::StatusMsg1, HobbywingStatus1Cb> *hobbywing_Status1_listener[HAL_MAX_CAN_PROTOCOL_DRIVERS];
UC_REGISTRY_BINDER(HobbywingStatus2Cb, com::hobbywing::esc::StatusMsg2);
static uavcan::Subscriber<com::hobbywing::esc::StatusMsg2, HobbywingStatus2Cb> *hobbywing_Status2_listener[HAL_MAX_CAN_PROTOCOL_DRIVERS];
#endif // AP_DRONECAN_HOBBYWING_ESC_ENABLED

// Clients
UC_CLIENT_CALL_REGISTRY_BINDER(ParamGetSetCb, uavcan::protocol::param::GetSet);
static uavcan::ServiceClient<uavcan::protocol::param::GetSet, ParamGetSetCb>* param_get_set_client[HAL_MAX_CAN_PROTOCOL_DRIVERS];
static uavcan::protocol::param::GetSet::Request param_getset_req[HAL_MAX_CAN_PROTOCOL_DRIVERS];

UC_CLIENT_CALL_REGISTRY_BINDER(ParamExecuteOpcodeCb, uavcan::protocol::param::ExecuteOpcode);
static uavcan::ServiceClient<uavcan::protocol::param::ExecuteOpcode, ParamExecuteOpcodeCb>* param_execute_opcode_client[HAL_MAX_CAN_PROTOCOL_DRIVERS];
static uavcan::protocol::param::ExecuteOpcode::Request param_save_req[HAL_MAX_CAN_PROTOCOL_DRIVERS];


// subscribers

// handler SafteyButton
UC_REGISTRY_BINDER(ButtonCb, ardupilot::indication::Button);
static uavcan::Subscriber<ardupilot::indication::Button, ButtonCb> *safety_button_listener[HAL_MAX_CAN_PROTOCOL_DRIVERS];

// handler TrafficReport
UC_REGISTRY_BINDER(TrafficReportCb, ardupilot::equipment::trafficmonitor::TrafficReport);
static uavcan::Subscriber<ardupilot::equipment::trafficmonitor::TrafficReport, TrafficReportCb> *traffic_report_listener[HAL_MAX_CAN_PROTOCOL_DRIVERS];

// handler actuator status
UC_REGISTRY_BINDER(ActuatorStatusCb, uavcan::equipment::actuator::Status);
static uavcan::Subscriber<uavcan::equipment::actuator::Status, ActuatorStatusCb> *actuator_status_listener[HAL_MAX_CAN_PROTOCOL_DRIVERS];

#if AP_DRONECAN_VOLZ_FEEDBACK_ENABLED
UC_REGISTRY_BINDER(ActuatorStatusVolzCb, com::volz::servo::ActuatorStatus);
static uavcan::Subscriber<com::volz::servo::ActuatorStatus, ActuatorStatusVolzCb> *actuator_status_Volz_listener[HAL_MAX_CAN_PROTOCOL_DRIVERS];
#endif

// handler ESC status
UC_REGISTRY_BINDER(ESCStatusCb, uavcan::equipment::esc::Status);
static uavcan::Subscriber<uavcan::equipment::esc::Status, ESCStatusCb> *esc_status_listener[HAL_MAX_CAN_PROTOCOL_DRIVERS];

#if AP_DRONECAN_HIMARK_SERVO_ENABLED
static uavcan::Publisher<com::himark::servo::ServoCmd>* himark_out[HAL_MAX_CAN_PROTOCOL_DRIVERS];
UC_REGISTRY_BINDER(HimarkServoInfoCb, com::himark::servo::ServoInfo);
static uavcan::Subscriber<com::himark::servo::ServoInfo, HimarkServoInfoCb> *himark_servoinfo_listener[HAL_MAX_CAN_PROTOCOL_DRIVERS];
#endif

// handler DEBUG
UC_REGISTRY_BINDER(DebugCb, uavcan::protocol::debug::LogMessage);
static uavcan::Subscriber<uavcan::protocol::debug::LogMessage, DebugCb> *debug_listener[HAL_MAX_CAN_PROTOCOL_DRIVERS];

AP_UAVCAN::AP_UAVCAN()
{
    AP_Param::setup_object_defaults(this, var_info);

    for (uint8_t i = 0; i < UAVCAN_SRV_NUMBER; i++) {
        _SRV_conf[i].esc_pending = false;
        _SRV_conf[i].servo_pending = false;
    }

    AP_Param::load_object_from_eeprom(this, var_info);

    debug_uavcan(AP_CANManager::LOG_INFO, "AP_UAVCAN constructed\n\r");
}

AP_UAVCAN::~AP_UAVCAN()
{
}

AP_UAVCAN *AP_UAVCAN::get_uavcan(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) != AP_CANManager::Driver_Type_UAVCAN) {
        return nullptr;
    }
    return static_cast<AP_UAVCAN*>(AP::can().get_driver(driver_index));
}

bool AP_UAVCAN::add_interface(AP_HAL::CANIface* can_iface) {

    if (_iface_mgr == nullptr) {
        _iface_mgr = new uavcan::CanIfaceMgr();
    }

    if (_iface_mgr == nullptr) {
        debug_uavcan(AP_CANManager::LOG_ERROR, "UAVCAN: can't create UAVCAN interface manager\n\r");
        return false;
    }

    if (!_iface_mgr->add_interface(can_iface)) {
        debug_uavcan(AP_CANManager::LOG_ERROR, "UAVCAN: can't add UAVCAN interface\n\r");
        return false;   
    }
    return true;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic error "-Wframe-larger-than=2200"
void AP_UAVCAN::init(uint8_t driver_index, bool enable_filters)
{
    _driver_index = driver_index;

    if (_initialized) {
        debug_uavcan(AP_CANManager::LOG_ERROR, "UAVCAN: init called more than once\n\r");
        return;
    }

    if (_iface_mgr == nullptr) {
        debug_uavcan(AP_CANManager::LOG_ERROR, "UAVCAN: can't get UAVCAN interface driver\n\r");
        return;
    }

    _allocator = new AP_PoolAllocator(_pool_size);

    if (_allocator == nullptr || !_allocator->init()) {
        debug_uavcan(AP_CANManager::LOG_ERROR, "UAVCAN: couldn't allocate node pool\n");
        return;
    }

    _node = new uavcan::Node<0>(*_iface_mgr, uavcan::SystemClock::instance(), *_allocator);

    if (_node == nullptr) {
        debug_uavcan(AP_CANManager::LOG_ERROR, "UAVCAN: couldn't allocate node\n\r");
        return;
    }

    if (_node->isStarted()) {
        debug_uavcan(AP_CANManager::LOG_ERROR, "UAVCAN: node was already started?\n\r");
        return;
    }
    {
        uavcan::NodeID self_node_id(_uavcan_node);
        _node->setNodeID(self_node_id);

        char ndname[20];
        snprintf(ndname, sizeof(ndname), "org.ardupilot:%u", driver_index);

        uavcan::NodeStatusProvider::NodeName name(ndname);
        _node->setName(name);
    }
    {
        uavcan::protocol::SoftwareVersion sw_version; // Standard type uavcan.protocol.SoftwareVersion
        sw_version.major = AP_UAVCAN_SW_VERS_MAJOR;
        sw_version.minor = AP_UAVCAN_SW_VERS_MINOR;
        _node->setSoftwareVersion(sw_version);

        uavcan::protocol::HardwareVersion hw_version; // Standard type uavcan.protocol.HardwareVersion

        hw_version.major = AP_UAVCAN_HW_VERS_MAJOR;
        hw_version.minor = AP_UAVCAN_HW_VERS_MINOR;

        const uint8_t uid_buf_len = hw_version.unique_id.capacity();
        uint8_t uid_len = uid_buf_len;
        uint8_t unique_id[uid_buf_len];


        if (hal.util->get_system_id_unformatted(unique_id, uid_len)) {
            //This is because we are maintaining a common Server Record for all UAVCAN Instances.
            //In case the node IDs are different, and unique id same, it will create
            //conflict in the Server Record.
            unique_id[uid_len - 1] += _uavcan_node;
            uavcan::copy(unique_id, unique_id + uid_len, hw_version.unique_id.begin());
        }
        _node->setHardwareVersion(hw_version);
    }

#if UAVCAN_SUPPORT_CANFD
    if (option_is_set(Options::CANFD_ENABLED)) {
        _node->enableCanFd();
    }
#endif

    int start_res = _node->start();
    if (start_res < 0) {
        debug_uavcan(AP_CANManager::LOG_ERROR, "UAVCAN: node start problem, error %d\n\r", start_res);
        return;
    }

    _dna_server = new AP_UAVCAN_DNA_Server(this, StorageAccess(StorageManager::StorageCANDNA));
    if (_dna_server == nullptr) {
        debug_uavcan(AP_CANManager::LOG_ERROR, "UAVCAN: couldn't allocate DNA server\n\r");
        return;
    }

    //Start Servers
    if (!_dna_server->init()) {
        debug_uavcan(AP_CANManager::LOG_ERROR, "UAVCAN: Failed to start DNA Server\n\r");
        return;
    }

    // Roundup all subscribers from supported drivers
    AP_UAVCAN_DNA_Server::subscribe_msgs(this);
    AP_GPS_UAVCAN::subscribe_msgs(this);
#if AP_COMPASS_UAVCAN_ENABLED
    AP_Compass_UAVCAN::subscribe_msgs(this);
#endif
#if AP_BARO_UAVCAN_ENABLED
    AP_Baro_UAVCAN::subscribe_msgs(this);
#endif
    AP_BattMonitor_UAVCAN::subscribe_msgs(this);
#if AP_AIRSPEED_UAVCAN_ENABLED
    AP_Airspeed_UAVCAN::subscribe_msgs(this);
#endif
#if AP_OPTICALFLOW_HEREFLOW_ENABLED
    AP_OpticalFlow_HereFlow::subscribe_msgs(this);
#endif
#if AP_RANGEFINDER_UAVCAN_ENABLED
    AP_RangeFinder_UAVCAN::subscribe_msgs(this);
#endif
#if AP_EFI_DRONECAN_ENABLED
    AP_EFI_DroneCAN::subscribe_msgs(this);
#endif

#if AP_PROXIMITY_DRONECAN_ENABLED
    AP_Proximity_DroneCAN::subscribe_msgs(this);
#endif

    act_out_array[driver_index] = new uavcan::Publisher<uavcan::equipment::actuator::ArrayCommand>(*_node);
    act_out_array[driver_index]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(2));
    act_out_array[driver_index]->setPriority(uavcan::TransferPriority::OneLowerThanHighest);

#if AP_DRONECAN_HIMARK_SERVO_ENABLED
    himark_enabled = option_is_set(Options::USE_HIMARK_SERVO);
    if (himark_enabled) {
        himark_out[driver_index] = new uavcan::Publisher<com::himark::servo::ServoCmd>(*_node);
        himark_out[driver_index]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(2));
        himark_out[driver_index]->setPriority(uavcan::TransferPriority::OneLowerThanHighest);

        himark_servoinfo_listener[driver_index] = new uavcan::Subscriber<com::himark::servo::ServoInfo, HimarkServoInfoCb>(*_node);
        if (himark_servoinfo_listener[driver_index]) {
            himark_servoinfo_listener[driver_index]->start(HimarkServoInfoCb(this, &handle_himark_servoinfo));
        }
    }
#endif // AP_DRONECAN_HIMARK_SERVO_ENABLED
    
    esc_raw[driver_index] = new uavcan::Publisher<uavcan::equipment::esc::RawCommand>(*_node);
    esc_raw[driver_index]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(2));
    esc_raw[driver_index]->setPriority(uavcan::TransferPriority::OneLowerThanHighest);

#if AP_DRONECAN_HOBBYWING_ESC_ENABLED
    hobbywing.enabled = option_is_set(Options::USE_HOBBYWING_ESC);
    if (hobbywing.enabled) {
        esc_hobbywing_GetEscID[driver_index] = new uavcan::Publisher<com::hobbywing::esc::GetEscID>(*_node);
        esc_hobbywing_GetEscID[driver_index]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
        esc_hobbywing_GetEscID[driver_index]->setPriority(uavcan::TransferPriority::OneHigherThanLowest);

        esc_hobbywing_raw[driver_index] = new uavcan::Publisher<com::hobbywing::esc::RawCommand>(*_node);
        esc_hobbywing_raw[driver_index]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(2));
        esc_hobbywing_raw[driver_index]->setPriority(uavcan::TransferPriority::OneLowerThanHighest);

        hobbywing_GetEscId_listener[driver_index] = new uavcan::Subscriber<com::hobbywing::esc::GetEscID, HobbywingESCIDCb>(*_node);
        if (hobbywing_GetEscId_listener[driver_index]) {
            hobbywing_GetEscId_listener[driver_index]->start(HobbywingESCIDCb(this, &handle_hobbywing_GetEscID));
        }

        hobbywing_Status1_listener[driver_index] = new uavcan::Subscriber<com::hobbywing::esc::StatusMsg1, HobbywingStatus1Cb>(*_node);
        if (hobbywing_Status1_listener[driver_index]) {
            hobbywing_Status1_listener[driver_index]->start(HobbywingStatus1Cb(this, &handle_hobbywing_StatusMsg1));
        }

        hobbywing_Status2_listener[driver_index] = new uavcan::Subscriber<com::hobbywing::esc::StatusMsg2, HobbywingStatus2Cb>(*_node);
        if (hobbywing_Status2_listener[driver_index]) {
            hobbywing_Status2_listener[driver_index]->start(HobbywingStatus2Cb(this, &handle_hobbywing_StatusMsg2));
        }
    }
#endif // AP_DRONECAN_HOBBYWING_ESC_ENABLED
    
    rgb_led[driver_index] = new uavcan::Publisher<uavcan::equipment::indication::LightsCommand>(*_node);
    rgb_led[driver_index]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
    rgb_led[driver_index]->setPriority(uavcan::TransferPriority::OneHigherThanLowest);

    buzzer[driver_index] = new uavcan::Publisher<uavcan::equipment::indication::BeepCommand>(*_node);
    buzzer[driver_index]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
    buzzer[driver_index]->setPriority(uavcan::TransferPriority::OneHigherThanLowest);

    safety_state[driver_index] = new uavcan::Publisher<ardupilot::indication::SafetyState>(*_node);
    safety_state[driver_index]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
    safety_state[driver_index]->setPriority(uavcan::TransferPriority::OneHigherThanLowest);

    arming_status[driver_index] = new uavcan::Publisher<uavcan::equipment::safety::ArmingStatus>(*_node);
    arming_status[driver_index]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
    arming_status[driver_index]->setPriority(uavcan::TransferPriority::OneHigherThanLowest);

#if AP_DRONECAN_SEND_GPS
    gnss_fix2[driver_index] = new uavcan::Publisher<uavcan::equipment::gnss::Fix2>(*_node);
    gnss_fix2[driver_index]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
    gnss_fix2[driver_index]->setPriority(uavcan::TransferPriority::OneLowerThanHighest);

    gnss_auxiliary[driver_index] = new uavcan::Publisher<uavcan::equipment::gnss::Auxiliary>(*_node);
    gnss_auxiliary[driver_index]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
    gnss_auxiliary[driver_index]->setPriority(uavcan::TransferPriority::OneLowerThanHighest);

    gnss_heading[driver_index] = new uavcan::Publisher<ardupilot::gnss::Heading>(*_node);
    gnss_heading[driver_index]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
    gnss_heading[driver_index]->setPriority(uavcan::TransferPriority::OneLowerThanHighest);

    gnss_status[driver_index] = new uavcan::Publisher<ardupilot::gnss::Status>(*_node);
    gnss_status[driver_index]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
    gnss_status[driver_index]->setPriority(uavcan::TransferPriority::OneLowerThanHighest);
#endif

    rtcm_stream[driver_index] = new uavcan::Publisher<uavcan::equipment::gnss::RTCMStream>(*_node);
    rtcm_stream[driver_index]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
    rtcm_stream[driver_index]->setPriority(uavcan::TransferPriority::OneHigherThanLowest);

    notify_state[driver_index] = new uavcan::Publisher<ardupilot::indication::NotifyState>(*_node);
    notify_state[driver_index]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
    notify_state[driver_index]->setPriority(uavcan::TransferPriority::OneHigherThanLowest);

    param_get_set_client[driver_index] = new uavcan::ServiceClient<uavcan::protocol::param::GetSet, ParamGetSetCb>(*_node, ParamGetSetCb(this, &AP_UAVCAN::handle_param_get_set_response));

    param_execute_opcode_client[driver_index] = new uavcan::ServiceClient<uavcan::protocol::param::ExecuteOpcode, ParamExecuteOpcodeCb>(*_node, ParamExecuteOpcodeCb(this, &AP_UAVCAN::handle_param_save_response));

    safety_button_listener[driver_index] = new uavcan::Subscriber<ardupilot::indication::Button, ButtonCb>(*_node);
    if (safety_button_listener[driver_index]) {
        safety_button_listener[driver_index]->start(ButtonCb(this, &handle_button));
    }

    traffic_report_listener[driver_index] = new uavcan::Subscriber<ardupilot::equipment::trafficmonitor::TrafficReport, TrafficReportCb>(*_node);
    if (traffic_report_listener[driver_index]) {
        traffic_report_listener[driver_index]->start(TrafficReportCb(this, &handle_traffic_report));
    }

    actuator_status_listener[driver_index] = new uavcan::Subscriber<uavcan::equipment::actuator::Status, ActuatorStatusCb>(*_node);
    if (actuator_status_listener[driver_index]) {
        actuator_status_listener[driver_index]->start(ActuatorStatusCb(this, &handle_actuator_status));
    }

#if AP_DRONECAN_VOLZ_FEEDBACK_ENABLED
    actuator_status_Volz_listener[driver_index] = new uavcan::Subscriber<com::volz::servo::ActuatorStatus, ActuatorStatusVolzCb>(*_node);
    if (actuator_status_Volz_listener[driver_index]) {
        actuator_status_Volz_listener[driver_index]->start(ActuatorStatusVolzCb(this, &handle_actuator_status_Volz));
    }
#endif

    esc_status_listener[driver_index] = new uavcan::Subscriber<uavcan::equipment::esc::Status, ESCStatusCb>(*_node);
    if (esc_status_listener[driver_index]) {
        esc_status_listener[driver_index]->start(ESCStatusCb(this, &handle_ESC_status));
    }

    debug_listener[driver_index] = new uavcan::Subscriber<uavcan::protocol::debug::LogMessage, DebugCb>(*_node);
    if (debug_listener[driver_index]) {
        debug_listener[driver_index]->start(DebugCb(this, &handle_debug));
    }
    
    _led_conf.devices_count = 0;

    /*
     * Informing other nodes that we're ready to work.
     * Default mode is INITIALIZING.
     */
    _node->setModeOperational();

    // Spin node for device discovery
    _node->spin(uavcan::MonotonicDuration::fromMSec(5000));

    snprintf(_thread_name, sizeof(_thread_name), "uavcan_%u", driver_index);

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_UAVCAN::loop, void), _thread_name, UAVCAN_STACK_SIZE, AP_HAL::Scheduler::PRIORITY_CAN, 0)) {
        _node->setModeOfflineAndPublish();
        debug_uavcan(AP_CANManager::LOG_ERROR, "UAVCAN: couldn't create thread\n\r");
        return;
    }

    _initialized = true;
    debug_uavcan(AP_CANManager::LOG_INFO, "UAVCAN: init done\n\r");
}
#pragma GCC diagnostic pop

void AP_UAVCAN::loop(void)
{
    while (true) {
        if (!_initialized) {
            hal.scheduler->delay_microseconds(1000);
            continue;
        }

        const int error = _node->spin(uavcan::MonotonicDuration::fromMSec(1));

        if (error < 0) {
            hal.scheduler->delay_microseconds(100);
            continue;
        }

        if (_SRV_armed) {
            bool sent_servos = false;

            if (_servo_bm > 0) {
                // if we have any Servos in bitmask
                uint32_t now = AP_HAL::native_micros();
                const uint32_t servo_period_us = 1000000UL / unsigned(_servo_rate_hz.get());
                if (now - _SRV_last_send_us >= servo_period_us) {
                    _SRV_last_send_us = now;
#if AP_DRONECAN_HIMARK_SERVO_ENABLED
                    if (himark_enabled) {
                        SRV_send_himark();
                    } else
#endif
                    {
                        SRV_send_actuator();
                    }
                    sent_servos = true;
                    for (uint8_t i = 0; i < UAVCAN_SRV_NUMBER; i++) {
                        _SRV_conf[i].servo_pending = false;
                    }
                }
            }

            // if we have any ESC's in bitmask
            if (_esc_bm > 0 && !sent_servos) {
#if AP_DRONECAN_HOBBYWING_ESC_ENABLED
                if (hobbywing.enabled) {
                    SRV_send_esc_hobbywing();
                } else
#endif
                {
                    SRV_send_esc();
                }
            }

            for (uint8_t i = 0; i < UAVCAN_SRV_NUMBER; i++) {
                _SRV_conf[i].esc_pending = false;
            }
        }

        led_out_send();
        buzzer_send();
        rtcm_stream_send();
        safety_state_send();
        notify_state_send();
        send_parameter_request();
        send_parameter_save_request();
        _dna_server->verify_nodes();
#if AP_OPENDRONEID_ENABLED
        AP::opendroneid().dronecan_send(this);
#endif

#if AP_DRONECAN_SEND_GPS
        if (option_is_set(AP_UAVCAN::Options::SEND_GNSS) && !AP_GPS_UAVCAN::instance_exists(this)) {
            // send if enabled and this interface/driver is not used by the AP_GPS driver
            gnss_send_fix();
            gnss_send_yaw();
        }
#endif

        logging();
#if AP_DRONECAN_HOBBYWING_ESC_ENABLED
        hobbywing_ESC_update();
#endif
    }
}


///// SRV output /////

void AP_UAVCAN::SRV_send_actuator(void)
{
    uint8_t starting_servo = 0;
    bool repeat_send;

    WITH_SEMAPHORE(SRV_sem);

    do {
        repeat_send = false;
        uavcan::equipment::actuator::ArrayCommand msg;

        uint8_t i;
        // UAVCAN can hold maximum of 15 commands in one frame
        for (i = 0; starting_servo < UAVCAN_SRV_NUMBER && i < 15; starting_servo++) {
            uavcan::equipment::actuator::Command cmd;

            /*
             * Servo output uses a range of 1000-2000 PWM for scaling.
             * This converts output PWM from [1000:2000] range to [-1:1] range that
             * is passed to servo as unitless type via UAVCAN.
             * This approach allows for MIN/TRIM/MAX values to be used fully on
             * autopilot side and for servo it should have the setup to provide maximum
             * physically possible throws at [-1:1] limits.
             */

            if (_SRV_conf[starting_servo].servo_pending && ((((uint32_t) 1) << starting_servo) & _servo_bm)) {
                cmd.actuator_id = starting_servo + 1;

                if (option_is_set(Options::USE_ACTUATOR_PWM)) {
                    cmd.command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_PWM;
                    cmd.command_value = _SRV_conf[starting_servo].pulse;
                } else {
                    cmd.command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_UNITLESS;
                    cmd.command_value = constrain_float(((float) _SRV_conf[starting_servo].pulse - 1000.0) / 500.0 - 1.0, -1.0, 1.0);
                }

                msg.commands.push_back(cmd);

                i++;
            }
        }

        if (i > 0) {
            if (act_out_array[_driver_index]->broadcast(msg) > 0) {
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

#if AP_DRONECAN_HIMARK_SERVO_ENABLED
/*
  Himark servo output. This uses com.himark.servo.ServoCmd packets
 */
void AP_UAVCAN::SRV_send_himark(void)
{
    WITH_SEMAPHORE(SRV_sem);

    // ServoCmd can hold maximum of 17 commands. First find the highest pending servo < 17
    int8_t highest_to_send = -1;
    for (int8_t i = 16; i >= 0; i--) {
        if (_SRV_conf[i].servo_pending && ((1U<<i) & _servo_bm) != 0) {
            highest_to_send = i;
            break;
        }
    }
    if (highest_to_send == -1) {
        // nothing to send
        return;
    }
    com::himark::servo::ServoCmd msg {};

    for (uint8_t i = 0; i <= highest_to_send; i++) {
        const uint16_t pulse = constrain_int16(_SRV_conf[i].pulse - 1000, 0, 1000);
        msg.cmd.push_back(pulse);
    }
    himark_out[_driver_index]->broadcast(msg);
}
#endif // AP_DRONECAN_HIMARK_SERVO_ENABLED


void AP_UAVCAN::SRV_send_esc(void)
{
    static const int cmd_max = uavcan::equipment::esc::RawCommand::FieldTypes::cmd::RawValueType::max();
    uavcan::equipment::esc::RawCommand esc_msg;

    uint8_t active_esc_num = 0, max_esc_num = 0;
    uint8_t k = 0;

    WITH_SEMAPHORE(SRV_sem);

    // esc offset allows for efficient packing of higher ESC numbers in RawCommand
    const uint8_t esc_offset = constrain_int16(_esc_offset.get(), 0, UAVCAN_SRV_NUMBER);

    // find out how many esc we have enabled and if they are active at all
    for (uint8_t i = esc_offset; i < UAVCAN_SRV_NUMBER; i++) {
        if ((((uint32_t) 1) << i) & _esc_bm) {
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
            if ((((uint32_t) 1) << i) & _esc_bm) {
                // TODO: ESC negative scaling for reverse thrust and reverse rotation
                float scaled = cmd_max * (hal.rcout->scale_esc_to_unity(_SRV_conf[i].pulse) + 1.0) / 2.0;

                scaled = constrain_float(scaled, 0, cmd_max);

                esc_msg.cmd.push_back(static_cast<int>(scaled));
            } else {
                esc_msg.cmd.push_back(static_cast<unsigned>(0));
            }

            k++;
        }

        if (esc_raw[_driver_index]->broadcast(esc_msg) > 0) {
            _esc_send_count++;
        } else {
            _fail_send_count++;
        }
    }
}

#if AP_DRONECAN_HOBBYWING_ESC_ENABLED
/*
  send HobbyWing version of ESC RawCommand
 */
void AP_UAVCAN::SRV_send_esc_hobbywing(void)
{
    static const int cmd_max = 8191;
    com::hobbywing::esc::RawCommand esc_msg;

    uint8_t active_esc_num = 0, max_esc_num = 0;
    uint8_t k = 0;

    WITH_SEMAPHORE(SRV_sem);

    // esc offset allows for efficient packing of higher ESC numbers in RawCommand
    const uint8_t esc_offset = constrain_int16(_esc_offset.get(), 0, UAVCAN_SRV_NUMBER);

    // find out how many esc we have enabled and if they are active at all
    for (uint8_t i = esc_offset; i < UAVCAN_SRV_NUMBER; i++) {
        if ((((uint32_t) 1) << i) & _esc_bm) {
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
            if ((((uint32_t) 1) << i) & _esc_bm) {
                float scaled = cmd_max * (hal.rcout->scale_esc_to_unity(_SRV_conf[i].pulse) + 1.0) / 2.0;

                scaled = constrain_float(scaled, 0, cmd_max);

                esc_msg.command.push_back(static_cast<int>(scaled));
            } else {
                esc_msg.command.push_back(static_cast<unsigned>(0));
            }

            k++;
        }

        if (esc_hobbywing_raw[_driver_index]->broadcast(esc_msg) > 0) {
            _esc_send_count++;
        } else {
            _fail_send_count++;
        }
    }
}
#endif // AP_DRONECAN_HOBBYWING_ESC_ENABLED

void AP_UAVCAN::SRV_push_servos()
{
    WITH_SEMAPHORE(SRV_sem);

    for (uint8_t i = 0; i < UAVCAN_SRV_NUMBER; i++) {
        // Check if this channels has any function assigned
        if (SRV_Channels::channel_function(i) >= SRV_Channel::k_none) {
            _SRV_conf[i].pulse = SRV_Channels::srv_channel(i)->get_output_pwm();
            _SRV_conf[i].esc_pending = true;
            _SRV_conf[i].servo_pending = true;
        }
    }

    _SRV_armed = hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED;
}


///// LED /////

void AP_UAVCAN::led_out_send()
{
    uint64_t now = AP_HAL::native_micros64();

    if ((now - _led_conf.last_update) < LED_DELAY_US) {
        return;
    }

    uavcan::equipment::indication::LightsCommand msg;
    {
        WITH_SEMAPHORE(_led_out_sem);

        if (_led_conf.devices_count == 0) {
            return;
        }

        uavcan::equipment::indication::SingleLightCommand cmd;

        for (uint8_t i = 0; i < _led_conf.devices_count; i++) {
            cmd.light_id =_led_conf.devices[i].led_index;
            cmd.color.red = _led_conf.devices[i].red >> 3;
            cmd.color.green = _led_conf.devices[i].green >> 2;
            cmd.color.blue = _led_conf.devices[i].blue >> 3;

            msg.commands.push_back(cmd);
        }
    }

    rgb_led[_driver_index]->broadcast(msg);
    _led_conf.last_update = now;
}

bool AP_UAVCAN::led_write(uint8_t led_index, uint8_t red, uint8_t green, uint8_t blue)
{
    if (_led_conf.devices_count >= AP_UAVCAN_MAX_LED_DEVICES) {
        return false;
    }

    WITH_SEMAPHORE(_led_out_sem);

    // check if a device instance exists. if so, break so the instance index is remembered
    uint8_t instance = 0;
    for (; instance < _led_conf.devices_count; instance++) {
        if (_led_conf.devices[instance].led_index == led_index) {
            break;
        }
    }

    // load into the correct instance.
    // if an existing instance was found in above for loop search,
    // then instance value is < _led_conf.devices_count.
    // otherwise a new one was just found so we increment the count.
    // Either way, the correct instance is the current value of instance
    _led_conf.devices[instance].led_index = led_index;
    _led_conf.devices[instance].red = red;
    _led_conf.devices[instance].green = green;
    _led_conf.devices[instance].blue = blue;

    if (instance == _led_conf.devices_count) {
        _led_conf.devices_count++;
    }

    return true;
}

// buzzer send
void AP_UAVCAN::buzzer_send()
{
    uavcan::equipment::indication::BeepCommand msg;
    WITH_SEMAPHORE(_buzzer.sem);
    uint8_t mask = (1U << _driver_index);
    if ((_buzzer.pending_mask & mask) == 0) {
        return;
    }
    _buzzer.pending_mask &= ~mask;
    msg.frequency = _buzzer.frequency;
    msg.duration = _buzzer.duration;
    buzzer[_driver_index]->broadcast(msg);
}

// buzzer support
void AP_UAVCAN::set_buzzer_tone(float frequency, float duration_s)
{
    WITH_SEMAPHORE(_buzzer.sem);
    _buzzer.frequency = frequency;
    _buzzer.duration = duration_s;
    _buzzer.pending_mask = 0xFF;
}

// notify state send
void AP_UAVCAN::notify_state_send()
{
    uint32_t now = AP_HAL::native_millis();

    if (_notify_state_hz == 0 || (now - _last_notify_state_ms) < uint32_t(1000 / _notify_state_hz)) {
        return;
    }

    ardupilot::indication::NotifyState msg;
    msg.vehicle_state = 0;
    if (AP_Notify::flags.initialising) {
        msg.vehicle_state |= 1 << ardupilot::indication::NotifyState::VEHICLE_STATE_INITIALISING;
    }
    if (AP_Notify::flags.armed) {
        msg.vehicle_state |= 1 << ardupilot::indication::NotifyState::VEHICLE_STATE_ARMED;
    }
    if (AP_Notify::flags.flying) {
        msg.vehicle_state |= 1 << ardupilot::indication::NotifyState::VEHICLE_STATE_FLYING;
    }
    if (AP_Notify::flags.compass_cal_running) {
        msg.vehicle_state |= 1 << ardupilot::indication::NotifyState::VEHICLE_STATE_MAGCAL_RUN;
    }
    if (AP_Notify::flags.ekf_bad) {
        msg.vehicle_state |= 1 << ardupilot::indication::NotifyState::VEHICLE_STATE_EKF_BAD;
    }
    if (AP_Notify::flags.esc_calibration) {
        msg.vehicle_state |= 1 << ardupilot::indication::NotifyState::VEHICLE_STATE_ESC_CALIBRATION;
    }
    if (AP_Notify::flags.failsafe_battery) {
        msg.vehicle_state |= 1 << ardupilot::indication::NotifyState::VEHICLE_STATE_FAILSAFE_BATT;
    }
    if (AP_Notify::flags.failsafe_gcs) {
        msg.vehicle_state |= 1 << ardupilot::indication::NotifyState::VEHICLE_STATE_FAILSAFE_GCS;
    }
    if (AP_Notify::flags.failsafe_radio) {
        msg.vehicle_state |= 1 << ardupilot::indication::NotifyState::VEHICLE_STATE_FAILSAFE_RADIO;
    }
    if (AP_Notify::flags.firmware_update) {
        msg.vehicle_state |= 1 << ardupilot::indication::NotifyState::VEHICLE_STATE_FW_UPDATE;
    }
    if (AP_Notify::flags.gps_fusion) {
        msg.vehicle_state |= 1 << ardupilot::indication::NotifyState::VEHICLE_STATE_GPS_FUSION;
    }
    if (AP_Notify::flags.gps_glitching) {
        msg.vehicle_state |= 1 << ardupilot::indication::NotifyState::VEHICLE_STATE_GPS_GLITCH;
    }
    if (AP_Notify::flags.have_pos_abs) {
        msg.vehicle_state |= 1 << ardupilot::indication::NotifyState::VEHICLE_STATE_POS_ABS_AVAIL;
    }
    if (AP_Notify::flags.leak_detected) {
        msg.vehicle_state |= 1 << ardupilot::indication::NotifyState::VEHICLE_STATE_LEAK_DET;
    }
    if (AP_Notify::flags.parachute_release) {
        msg.vehicle_state |= 1 << ardupilot::indication::NotifyState::VEHICLE_STATE_CHUTE_RELEASED;
    }
    if (AP_Notify::flags.powering_off) {
        msg.vehicle_state |= 1 << ardupilot::indication::NotifyState::VEHICLE_STATE_POWERING_OFF;
    }
    if (AP_Notify::flags.pre_arm_check) {
        msg.vehicle_state |= 1 << ardupilot::indication::NotifyState::VEHICLE_STATE_PREARM;
    }
    if (AP_Notify::flags.pre_arm_gps_check) {
        msg.vehicle_state |= 1 << ardupilot::indication::NotifyState::VEHICLE_STATE_PREARM_GPS;
    }
    if (AP_Notify::flags.save_trim) {
        msg.vehicle_state |= 1 << ardupilot::indication::NotifyState::VEHICLE_STATE_SAVE_TRIM;
    }
    if (AP_Notify::flags.vehicle_lost) {
        msg.vehicle_state |= 1 << ardupilot::indication::NotifyState::VEHICLE_STATE_LOST;
    }
    if (AP_Notify::flags.video_recording) {
        msg.vehicle_state |= 1 << ardupilot::indication::NotifyState::VEHICLE_STATE_VIDEO_RECORDING;
    }
    if (AP_Notify::flags.waiting_for_throw) {
        msg.vehicle_state |= 1 << ardupilot::indication::NotifyState::VEHICLE_STATE_THROW_READY;
    }

    msg.aux_data_type = ardupilot::indication::NotifyState::VEHICLE_YAW_EARTH_CENTIDEGREES;
    uint16_t yaw_cd = (uint16_t)(360.0f - degrees(AP::ahrs().get_yaw()))*100.0f;
    const uint8_t *data = (uint8_t *)&yaw_cd;
    for (uint8_t i=0; i<2; i++) {
        msg.aux_data.push_back(data[i]);
    }
    notify_state[_driver_index]->broadcast(msg);
    _last_notify_state_ms = AP_HAL::native_millis();
}

#if AP_DRONECAN_SEND_GPS
void AP_UAVCAN::gnss_send_fix()
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

    uavcan::equipment::gnss::Fix2 pkt {};
    const Location &loc = gps.location();
    const Vector3f &vel = gps.velocity();

    pkt.timestamp.usec = AP_HAL::native_micros64();
    pkt.gnss_timestamp.usec = gps.time_epoch_usec();
    if (pkt.gnss_timestamp.usec == 0) {
        pkt.gnss_time_standard = uavcan::equipment::gnss::Fix2::GNSS_TIME_STANDARD_NONE;
    } else {
        pkt.gnss_time_standard = uavcan::equipment::gnss::Fix2::GNSS_TIME_STANDARD_UTC;
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
        pkt.status = uavcan::equipment::gnss::Fix2::STATUS_NO_FIX;
        pkt.mode = uavcan::equipment::gnss::Fix2::MODE_SINGLE;
        pkt.sub_mode = uavcan::equipment::gnss::Fix2::SUB_MODE_DGPS_OTHER;
        break;
    case AP_GPS::GPS_Status::GPS_OK_FIX_2D:
        pkt.status = uavcan::equipment::gnss::Fix2::STATUS_2D_FIX;
        pkt.mode = uavcan::equipment::gnss::Fix2::MODE_SINGLE;
        pkt.sub_mode = uavcan::equipment::gnss::Fix2::SUB_MODE_DGPS_OTHER;
        break;
    case AP_GPS::GPS_Status::GPS_OK_FIX_3D:
        pkt.status = uavcan::equipment::gnss::Fix2::STATUS_3D_FIX;
        pkt.mode = uavcan::equipment::gnss::Fix2::MODE_SINGLE;
        pkt.sub_mode = uavcan::equipment::gnss::Fix2::SUB_MODE_DGPS_OTHER;
        break;
    case AP_GPS::GPS_Status::GPS_OK_FIX_3D_DGPS:
        pkt.status = uavcan::equipment::gnss::Fix2::STATUS_3D_FIX;
        pkt.mode = uavcan::equipment::gnss::Fix2::MODE_DGPS;
        pkt.sub_mode = uavcan::equipment::gnss::Fix2::SUB_MODE_DGPS_SBAS;
        break;
    case AP_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FLOAT:
        pkt.status = uavcan::equipment::gnss::Fix2::STATUS_3D_FIX;
        pkt.mode = uavcan::equipment::gnss::Fix2::MODE_RTK;
        pkt.sub_mode = uavcan::equipment::gnss::Fix2::SUB_MODE_RTK_FLOAT;
        break;
    case AP_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FIXED:
        pkt.status = uavcan::equipment::gnss::Fix2::STATUS_3D_FIX;
        pkt.mode = uavcan::equipment::gnss::Fix2::MODE_RTK;
        pkt.sub_mode = uavcan::equipment::gnss::Fix2::SUB_MODE_RTK_FIXED;
        break;
    }

    pkt.covariance.resize(6);
    float hacc;
    if (gps.horizontal_accuracy(hacc)) {
        pkt.covariance[0] = pkt.covariance[1] = sq(hacc);
    }
    float vacc;
    if (gps.vertical_accuracy(vacc)) {
        pkt.covariance[2] = sq(vacc);
    }
    float sacc;
    if (gps.speed_accuracy(sacc)) {
        const float vc3 = sq(sacc);
        pkt.covariance[3] = pkt.covariance[4] = pkt.covariance[5] = vc3;
    }

    gnss_fix2[_driver_index]->broadcast(pkt);



    const uint32_t now_ms = AP_HAL::native_millis();
    if (now_ms - _gnss.last_send_status_ms >= 1000) {
        _gnss.last_send_status_ms = now_ms;

        /*
        send aux packet
        */
        uavcan::equipment::gnss::Auxiliary pkt_auxiliary {};
        pkt_auxiliary.hdop = gps.get_hdop() * 0.01;
        pkt_auxiliary.vdop = gps.get_vdop() * 0.01;

        gnss_auxiliary[_driver_index]->broadcast(pkt_auxiliary);


        /*
        send Status packet
        */
        ardupilot::gnss::Status pkt_status {};
        pkt_status.healthy = gps.is_healthy();
        if (gps.logging_present() && gps.logging_enabled() && !gps.logging_failed()) {
            pkt_status.status |= ardupilot::gnss::Status::STATUS_LOGGING;
        }
        uint8_t idx; // unused
        if (pkt_status.healthy && !gps.first_unconfigured_gps(idx)) {
            pkt_status.status |= ardupilot::gnss::Status::STATUS_ARMABLE;
        }

        uint32_t error_codes;
        if (gps.get_error_codes(error_codes)) {
            pkt_status.error_codes = error_codes;
        }

        gnss_status[_driver_index]->broadcast(pkt_status);
    }
}

void AP_UAVCAN::gnss_send_yaw()
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

    ardupilot::gnss::Heading pkt_heading {};
    pkt_heading.heading_valid = true;
    pkt_heading.heading_accuracy_valid = is_positive(yaw_acc_deg);
    pkt_heading.heading_rad = radians(yaw_deg);
    pkt_heading.heading_accuracy_rad = radians(yaw_acc_deg);

    gnss_heading[_driver_index]->broadcast(pkt_heading);
}
#endif // AP_DRONECAN_SEND_GPS


void AP_UAVCAN::rtcm_stream_send()
{
    WITH_SEMAPHORE(_rtcm_stream.sem);
    if (_rtcm_stream.buf == nullptr ||
        _rtcm_stream.buf->available() == 0) {
        // nothing to send
        return;
    }
    uint32_t now = AP_HAL::native_millis();
    if (now - _rtcm_stream.last_send_ms < 20) {
        // don't send more than 50 per second
        return;
    }
    _rtcm_stream.last_send_ms = now;
    uavcan::equipment::gnss::RTCMStream msg;
    uint32_t len = _rtcm_stream.buf->available();
    if (len > 128) {
        len = 128;
    }
    msg.protocol_id = uavcan::equipment::gnss::RTCMStream::PROTOCOL_ID_RTCM3;
    for (uint8_t i=0; i<len; i++) {
        uint8_t b;
        if (!_rtcm_stream.buf->read_byte(&b)) {
            return;
        }
        msg.data.push_back(b);
    }
    rtcm_stream[_driver_index]->broadcast(msg);
}

// SafetyState send
void AP_UAVCAN::safety_state_send()
{
    uint32_t now = AP_HAL::native_millis();
    if (now - _last_safety_state_ms < 500) {
        // update at 2Hz
        return;
    }
    _last_safety_state_ms = now;

    { // handle SafetyState
        ardupilot::indication::SafetyState safety_msg;
        switch (hal.util->safety_switch_state()) {
        case AP_HAL::Util::SAFETY_ARMED:
            safety_msg.status = ardupilot::indication::SafetyState::STATUS_SAFETY_OFF;
            break;
        case AP_HAL::Util::SAFETY_DISARMED:
            safety_msg.status = ardupilot::indication::SafetyState::STATUS_SAFETY_ON;
            break;
        default:
            // nothing to send
            break;
        }
        safety_state[_driver_index]->broadcast(safety_msg);
    }

    { // handle ArmingStatus
        uavcan::equipment::safety::ArmingStatus arming_msg;
        arming_msg.status = hal.util->get_soft_armed() ? uavcan::equipment::safety::ArmingStatus::STATUS_FULLY_ARMED :
                                                      uavcan::equipment::safety::ArmingStatus::STATUS_DISARMED;
        arming_status[_driver_index]->broadcast(arming_msg);
    }
}

/*
 send RTCMStream packet on all active UAVCAN drivers
*/
void AP_UAVCAN::send_RTCMStream(const uint8_t *data, uint32_t len)
{
    WITH_SEMAPHORE(_rtcm_stream.sem);
    if (_rtcm_stream.buf == nullptr) {
        // give enough space for a full round from a NTRIP server with all
        // constellations
        _rtcm_stream.buf = new ByteBuffer(2400);
    }
    if (_rtcm_stream.buf == nullptr) {
        return;
    }
    _rtcm_stream.buf->write(data, len);
}

/*
  handle Button message
 */
void AP_UAVCAN::handle_button(AP_UAVCAN* ap_uavcan, uint8_t node_id, const ButtonCb &cb)
{
    switch (cb.msg->button) {
    case ardupilot::indication::Button::BUTTON_SAFETY: {
        AP_BoardConfig *brdconfig = AP_BoardConfig::get_singleton();
        if (brdconfig && brdconfig->safety_button_handle_pressed(cb.msg->press_time)) {
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
void AP_UAVCAN::handle_traffic_report(AP_UAVCAN* ap_uavcan, uint8_t node_id, const TrafficReportCb &cb)
{
#if HAL_ADSB_ENABLED
    AP_ADSB *adsb = AP::ADSB();
    if (!adsb || !adsb->enabled()) {
        // ADSB not enabled
        return;
    }

    const ardupilot::equipment::trafficmonitor::TrafficReport &msg = cb.msg[0];
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

    if (msg.alt_type == ardupilot::equipment::trafficmonitor::TrafficReport::ALT_TYPE_PRESSURE_AMSL) {
        pkt.flags |= ADSB_FLAGS_VALID_ALTITUDE;
        pkt.altitude_type = ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
    } else if (msg.alt_type == ardupilot::equipment::trafficmonitor::TrafficReport::ALT_TYPE_WGS84) {
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

    vehicle.last_update_ms = AP_HAL::native_millis() - (vehicle.info.tslc * 1000);
    adsb->handle_adsb_vehicle(vehicle);
#endif
}

/*
  handle actuator status message
 */
void AP_UAVCAN::handle_actuator_status(AP_UAVCAN* ap_uavcan, uint8_t node_id, const ActuatorStatusCb &cb)
{
    // log as CSRV message
    AP::logger().Write_ServoStatus(AP_HAL::native_micros64(),
                                   cb.msg->actuator_id,
                                   cb.msg->position,
                                   cb.msg->force,
                                   cb.msg->speed,
                                   cb.msg->power_rating_pct,
                                   0, 0, 0, 0, 0, 0);
}

#if AP_DRONECAN_HIMARK_SERVO_ENABLED
/*
  handle himark ServoInfo message
 */
void AP_UAVCAN::handle_himark_servoinfo(AP_UAVCAN* ap_uavcan, uint8_t node_id, const HimarkServoInfoCb &cb)
{
    // log as CSRV message
    AP::logger().Write_ServoStatus(AP_HAL::native_micros64(),
                                   cb.msg->servo_id,
                                   cb.msg->pos_sensor*0.01,
                                   0,
                                   0,
                                   0,
                                   cb.msg->pos_cmd*0.01,
                                   cb.msg->voltage*0.01,
                                   cb.msg->current*0.01,
                                   cb.msg->motor_temp*0.2-40,
                                   cb.msg->pcb_temp*0.2-40,
                                   cb.msg->error_status);
}
#endif // AP_DRONECAN_HIMARK_SERVO_ENABLED

#if AP_DRONECAN_VOLZ_FEEDBACK_ENABLED
void AP_UAVCAN::handle_actuator_status_Volz(AP_UAVCAN* ap_uavcan, uint8_t node_id, const ActuatorStatusVolzCb &cb)
{
    AP::logger().WriteStreaming(
        "CVOL",
        "TimeUS,Id,Pos,Cur,V,Pow,T",
        "s#dAv%O",
        "F-00000",
        "QBfffBh",
        AP_HAL::native_micros64(),
        cb.msg->actuator_id,
        ToDeg(cb.msg->actual_position),
        cb.msg->current * 0.025f,
        cb.msg->voltage * 0.2f,
        cb.msg->motor_pwm * (100.0/255.0),
        int16_t(cb.msg->motor_temperature) - 50);
}
#endif

/*
  handle ESC status message
 */
void AP_UAVCAN::handle_ESC_status(AP_UAVCAN* ap_uavcan, uint8_t node_id, const ESCStatusCb &cb)
{
#if HAL_WITH_ESC_TELEM
    const uint8_t esc_offset = constrain_int16(ap_uavcan->_esc_offset.get(), 0, UAVCAN_SRV_NUMBER);
    const uint8_t esc_index = cb.msg->esc_index + esc_offset;

    if (!is_esc_data_index_valid(esc_index)) {
        return;
    }

    TelemetryData t {
        .temperature_cdeg = int16_t((KELVIN_TO_C(cb.msg->temperature)) * 100),
        .voltage = cb.msg->voltage,
        .current = cb.msg->current,
    };

    ap_uavcan->update_rpm(esc_index, cb.msg->rpm, cb.msg->error_count);
    ap_uavcan->update_telem_data(esc_index, t,
        (isnan(cb.msg->current) ? 0 : AP_ESC_Telem_Backend::TelemetryType::CURRENT)
            | (isnan(cb.msg->voltage) ? 0 : AP_ESC_Telem_Backend::TelemetryType::VOLTAGE)
            | (isnan(cb.msg->temperature) ? 0 : AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE));
#endif
}

bool AP_UAVCAN::is_esc_data_index_valid(const uint8_t index) {
    if (index > UAVCAN_SRV_NUMBER) {
        // printf("UAVCAN: invalid esc index: %d. max index allowed: %d\n\r", index, UAVCAN_SRV_NUMBER);
        return false;
    }
    return true;
}

/*
  handle LogMessage debug
 */
void AP_UAVCAN::handle_debug(AP_UAVCAN* ap_uavcan, uint8_t node_id, const DebugCb &cb)
{
#if HAL_LOGGING_ENABLED
    const auto &msg = *cb.msg;
    if (AP::can().get_log_level() != AP_CANManager::LOG_NONE) {
        // log to onboard log and mavlink
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CAN[%u] %s", node_id, msg.text.c_str());
    } else {
        // only log to onboard log
        AP::logger().Write_MessageF("CAN[%u] %s", node_id, msg.text.c_str());
    }
#endif
}

void AP_UAVCAN::send_parameter_request()
{
    WITH_SEMAPHORE(_param_sem);
    if (param_request_sent) {
        return;
    }
    param_get_set_client[_driver_index]->call(param_request_node_id, param_getset_req[_driver_index]);
    param_request_sent = true;
}

bool AP_UAVCAN::set_parameter_on_node(uint8_t node_id, const char *name, float value, ParamGetSetFloatCb *cb)
{
    WITH_SEMAPHORE(_param_sem);
    if (param_int_cb != nullptr ||
        param_float_cb != nullptr) {
        //busy
        return false;
    }
    param_getset_req[_driver_index].index = 0;
    param_getset_req[_driver_index].name = name;
    param_getset_req[_driver_index].value.to<uavcan::protocol::param::Value::Tag::real_value>() = value;
    param_float_cb = cb;
    param_request_sent = false;
    param_request_node_id = node_id;
    return true;
}

bool AP_UAVCAN::set_parameter_on_node(uint8_t node_id, const char *name, int32_t value, ParamGetSetIntCb *cb)
{
    WITH_SEMAPHORE(_param_sem);
    if (param_int_cb != nullptr ||
        param_float_cb != nullptr) {
        //busy
        return false;
    }
    param_getset_req[_driver_index].index = 0;
    param_getset_req[_driver_index].name = name;
    param_getset_req[_driver_index].value.to<uavcan::protocol::param::Value::Tag::integer_value>() = value;
    param_int_cb = cb;
    param_request_sent = false;
    param_request_node_id = node_id;
    return true;
}

bool AP_UAVCAN::get_parameter_on_node(uint8_t node_id, const char *name, ParamGetSetFloatCb *cb)
{
    WITH_SEMAPHORE(_param_sem);
    if (param_int_cb != nullptr ||
        param_float_cb != nullptr) {
        //busy
        return false;
    }
    param_getset_req[_driver_index].index = 0;
    param_getset_req[_driver_index].name = name;
    param_getset_req[_driver_index].value.to<uavcan::protocol::param::Value::Tag::empty>();
    param_float_cb = cb;
    param_request_sent = false;
    param_request_node_id = node_id;
    return true;
}

bool AP_UAVCAN::get_parameter_on_node(uint8_t node_id, const char *name, ParamGetSetIntCb *cb)
{
    WITH_SEMAPHORE(_param_sem);
    if (param_int_cb != nullptr ||
        param_float_cb != nullptr) {
        //busy
        return false;
    }
    param_getset_req[_driver_index].index = 0;
    param_getset_req[_driver_index].name = name;
    param_getset_req[_driver_index].value.to<uavcan::protocol::param::Value::Tag::empty>();
    param_int_cb = cb;
    param_request_sent = false;
    param_request_node_id = node_id;
    return true;
}

void AP_UAVCAN::handle_param_get_set_response(AP_UAVCAN* ap_uavcan, uint8_t node_id, const ParamGetSetCb &cb)
{
    WITH_SEMAPHORE(ap_uavcan->_param_sem);
    if (!ap_uavcan->param_int_cb &&
        !ap_uavcan->param_float_cb) {
        return;
    }
    uavcan::protocol::param::GetSet::Response rsp = cb.rsp->getResponse();
    if (rsp.value.is(uavcan::protocol::param::Value::Tag::integer_value) && ap_uavcan->param_int_cb) {
        int32_t val = rsp.value.to<uavcan::protocol::param::Value::Tag::integer_value>();
        if ((*ap_uavcan->param_int_cb)(ap_uavcan, node_id, rsp.name.c_str(), val)) {
            // we want the parameter to be set with val
            param_getset_req[ap_uavcan->_driver_index].index = 0;
            param_getset_req[ap_uavcan->_driver_index].name = rsp.name;
            param_getset_req[ap_uavcan->_driver_index].value.to<uavcan::protocol::param::Value::Tag::integer_value>() = val;
            ap_uavcan->param_int_cb = ap_uavcan->param_int_cb;
            ap_uavcan->param_request_sent = false;
            ap_uavcan->param_request_node_id = node_id;
            return;
        }
    } else if (rsp.value.is(uavcan::protocol::param::Value::Tag::real_value) && ap_uavcan->param_float_cb) {
        float val = rsp.value.to<uavcan::protocol::param::Value::Tag::real_value>();
        if ((*ap_uavcan->param_float_cb)(ap_uavcan, node_id, rsp.name.c_str(), val)) {
            // we want the parameter to be set with val
            param_getset_req[ap_uavcan->_driver_index].index = 0;
            param_getset_req[ap_uavcan->_driver_index].name = rsp.name;
            param_getset_req[ap_uavcan->_driver_index].value.to<uavcan::protocol::param::Value::Tag::real_value>() = val;
            ap_uavcan->param_float_cb = ap_uavcan->param_float_cb;
            ap_uavcan->param_request_sent = false;
            ap_uavcan->param_request_node_id = node_id;
            return;
        }
    }
    ap_uavcan->param_int_cb = nullptr;
    ap_uavcan->param_float_cb = nullptr;
}


void AP_UAVCAN::send_parameter_save_request()
{
    WITH_SEMAPHORE(_param_save_sem);
    if (param_save_request_sent) {
        return;
    }
    param_execute_opcode_client[_driver_index]->call(param_save_request_node_id, param_save_req[_driver_index]);
    param_save_request_sent = true;
}

bool AP_UAVCAN::save_parameters_on_node(uint8_t node_id, ParamSaveCb *cb)
{
    WITH_SEMAPHORE(_param_save_sem);
    if (save_param_cb != nullptr) {
        //busy
        return false;
    }

    param_save_req[_driver_index].opcode = uavcan::protocol::param::ExecuteOpcode::Request::OPCODE_SAVE;
    param_save_request_sent = false;
    param_save_request_node_id = node_id;
    save_param_cb = cb;
    return true;
}

// handle parameter save request response
void AP_UAVCAN::handle_param_save_response(AP_UAVCAN* ap_uavcan, uint8_t node_id, const ParamExecuteOpcodeCb &cb)
{
    WITH_SEMAPHORE(ap_uavcan->_param_save_sem);
    if (!ap_uavcan->save_param_cb) {
        return;
    }
    uavcan::protocol::param::ExecuteOpcode::Response rsp = cb.rsp->getResponse();
    (*ap_uavcan->save_param_cb)(ap_uavcan, node_id, rsp.ok);
    ap_uavcan->save_param_cb = nullptr;
}

// Send Reboot command
// Note: Do not call this from outside UAVCAN thread context,
// THIS IS NOT A THREAD SAFE API!
void AP_UAVCAN::send_reboot_request(uint8_t node_id)
{
    if (_node == nullptr) {
        return;
    }
    uavcan::protocol::RestartNode::Request request;
    request.magic_number = uavcan::protocol::RestartNode::Request::MAGIC_NUMBER;
    uavcan::ServiceClient<uavcan::protocol::RestartNode> client(*_node);
    client.setCallback([](const uavcan::ServiceCallResult<uavcan::protocol::RestartNode>& call_result){});

    client.call(node_id, request);
}

// check if a option is set and if it is then reset it to 0.
// return true if it was set
bool AP_UAVCAN::check_and_reset_option(Options option)
{
    bool ret = option_is_set(option);
    if (ret) {
        _options.set_and_save(int16_t(_options.get() & ~uint16_t(option)));
    }
    return ret;
}

// handle prearm check
bool AP_UAVCAN::prearm_check(char* fail_msg, uint8_t fail_msg_len) const
{
    // forward this to DNA_Server
    return _dna_server->prearm_check(fail_msg, fail_msg_len);
}

/*
  periodic logging
 */
void AP_UAVCAN::logging(void)
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

#if AP_DRONECAN_HOBBYWING_ESC_ENABLED
/*
  update ESC node mapping
 */
void AP_UAVCAN::hobbywing_ESC_update(void)
{
    if (!hobbywing.enabled || hal.util->get_soft_armed()) {
        return;
    }
    uint32_t now = AP_HAL::millis();
    if (now - hobbywing.last_GetId_send_ms >= 1000U) {
        com::hobbywing::esc::GetEscID msg;
        hobbywing.last_GetId_send_ms = now;
        msg.payload.push_back(0);
        esc_hobbywing_GetEscID[_driver_index]->broadcast(msg);
    }
}

/*
  handle hobbywing GetEscID reply. This gets us the mapping between CAN NodeID and throttle channel
 */
void AP_UAVCAN::handle_hobbywing_GetEscID(AP_UAVCAN* ap_uavcan, uint8_t node_id, const HobbywingESCIDCb &cb)
{
    const auto &msg = *cb.msg;
    if (msg.payload.size() == 2 &&
        msg.payload[0] == node_id) {
        // throttle channel is 2nd payload byte
        const uint8_t thr_channel = msg.payload[1];
        if (thr_channel > 0 && thr_channel <= HOBBYWING_MAX_ESC) {
            ap_uavcan->hobbywing.thr_chan[thr_channel-1] = node_id;
        }
    }
}

/*
  find the ESC index given a CAN node ID
 */
bool AP_UAVCAN::hobbywing_find_esc_index(uint8_t node_id, uint8_t &esc_index) const
{
    for (uint8_t i=0; i<HOBBYWING_MAX_ESC; i++) {
        if (hobbywing.thr_chan[i] == node_id) {
            const uint8_t esc_offset = constrain_int16(_esc_offset.get(), 0, UAVCAN_SRV_NUMBER);
            esc_index = i + esc_offset;
            return true;
        }
    }
    return false;
}

/*
  handle hobbywing StatusMsg1 reply
 */
void AP_UAVCAN::handle_hobbywing_StatusMsg1(AP_UAVCAN* ap_uavcan, uint8_t node_id, const HobbywingStatus1Cb &cb)
{
    uint8_t esc_index;
    if (ap_uavcan->hobbywing_find_esc_index(node_id, esc_index)) {
        ap_uavcan->update_rpm(esc_index, cb.msg->rpm);
    }
}

/*
  handle hobbywing StatusMsg2 reply
 */
void AP_UAVCAN::handle_hobbywing_StatusMsg2(AP_UAVCAN* ap_uavcan, uint8_t node_id, const HobbywingStatus2Cb &cb)
{
    uint8_t esc_index;
    if (ap_uavcan->hobbywing_find_esc_index(node_id, esc_index)) {
        TelemetryData t {
            .temperature_cdeg = int16_t(cb.msg->temperature*100),
            .voltage = cb.msg->input_voltage*0.1f,
            .current = cb.msg->current*0.1f,
        };
        ap_uavcan->update_telem_data(esc_index, t,
                                     AP_ESC_Telem_Backend::TelemetryType::CURRENT|
                                     AP_ESC_Telem_Backend::TelemetryType::VOLTAGE|
                                     AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE);
    }

}
#endif // AP_DRONECAN_HOBBYWING_ESC_ENABLED

#endif // HAL_NUM_CAN_IFACES
