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
#pragma once

#include <AP_HAL/AP_HAL.h>

#if HAL_ENABLE_DRONECAN_DRIVERS

#include "AP_Canard_iface.h"
#include <AP_CANManager/AP_CANManager.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>
#include <SRV_Channel/SRV_Channel_config.h>
#include <canard/publisher.h>
#include <canard/subscriber.h>
#include <canard/service_client.h>
#include <canard/service_server.h>
#include <stdio.h>
#include "AP_DroneCAN_DNA_Server.h"
#include <canard.h>
#include <dronecan_msgs.h>

#ifndef DRONECAN_SRV_NUMBER
#define DRONECAN_SRV_NUMBER NUM_SERVO_CHANNELS
#endif

#ifndef AP_DRONECAN_SEND_GPS
#define AP_DRONECAN_SEND_GPS (BOARD_FLASH_SIZE > 1024)
#endif

#define AP_DRONECAN_SW_VERS_MAJOR 1
#define AP_DRONECAN_SW_VERS_MINOR 0

#define AP_DRONECAN_HW_VERS_MAJOR 1
#define AP_DRONECAN_HW_VERS_MINOR 0


#ifndef AP_DRONECAN_HOBBYWING_ESC_SUPPORT
#define AP_DRONECAN_HOBBYWING_ESC_SUPPORT (BOARD_FLASH_SIZE>1024)
#endif

// fwd-declare callback classes
class AP_DroneCAN_DNA_Server;

class AP_DroneCAN : public AP_CANDriver, public AP_ESC_Telem_Backend {
    friend class AP_DroneCAN_DNA_Server;
public:
    AP_DroneCAN(const int driver_index);
    ~AP_DroneCAN();

    static const struct AP_Param::GroupInfo var_info[];

    // Return uavcan from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_DroneCAN *get_dronecan(uint8_t driver_index);
    bool prearm_check(char* fail_msg, uint8_t fail_msg_len) const;

    void init(uint8_t driver_index, bool enable_filters) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    uint8_t get_driver_index() const { return _driver_index; }

    FUNCTOR_TYPEDEF(ParamGetSetIntCb, bool, AP_DroneCAN*, const uint8_t, const char*, int32_t &);
    FUNCTOR_TYPEDEF(ParamGetSetFloatCb, bool, AP_DroneCAN*, const uint8_t, const char*, float &);
    FUNCTOR_TYPEDEF(ParamSaveCb, void, AP_DroneCAN*,  const uint8_t, bool);

    void send_node_status();

    ///// SRV output /////
    void SRV_push_servos(void);

    ///// LED /////
    bool led_write(uint8_t led_index, uint8_t red, uint8_t green, uint8_t blue);

    // buzzer
    void set_buzzer_tone(float frequency, float duration_s);

    // Send Reboot command
    // Note: Do not call this from outside UAVCAN thread context,
    // you can call this from dronecan callbacks and handlers.
    // THIS IS NOT A THREAD SAFE API!
    void send_reboot_request(uint8_t node_id);

    // get or set param value
    // returns true on success, false on failure
    // failures occur when waiting on node to respond to previous get or set request
    bool set_parameter_on_node(uint8_t node_id, const char *name, float value, ParamGetSetFloatCb *cb);
    bool set_parameter_on_node(uint8_t node_id, const char *name, int32_t value, ParamGetSetIntCb *cb);
    bool get_parameter_on_node(uint8_t node_id, const char *name, ParamGetSetFloatCb *cb);
    bool get_parameter_on_node(uint8_t node_id, const char *name, ParamGetSetIntCb *cb);

    // Save parameters
    bool save_parameters_on_node(uint8_t node_id, ParamSaveCb *cb);

    // options bitmask
    enum class Options : uint16_t {
        DNA_CLEAR_DATABASE        = (1U<<0),
        DNA_IGNORE_DUPLICATE_NODE = (1U<<1),
        CANFD_ENABLED             = (1U<<2),
        DNA_IGNORE_UNHEALTHY_NODE = (1U<<3),
        USE_ACTUATOR_PWM          = (1U<<4),
        SEND_GNSS                 = (1U<<5),
        USE_HIMARK_SERVO          = (1U<<6),
        USE_HOBBYWING_ESC         = (1U<<7),
        ENABLE_STATS              = (1U<<8),
    };

    // check if a option is set
    bool option_is_set(Options option) const {
        return (uint16_t(_options.get()) & uint16_t(option)) != 0;
    }

    // check if a option is set and if it is then reset it to
    // 0. return true if it was set
    bool check_and_reset_option(Options option);

    CanardInterface& get_canard_iface() { return canard_iface; }

    Canard::Publisher<uavcan_equipment_indication_LightsCommand> rgb_led{canard_iface};
    Canard::Publisher<uavcan_equipment_indication_BeepCommand> buzzer{canard_iface};
    Canard::Publisher<uavcan_equipment_gnss_RTCMStream> rtcm_stream{canard_iface};

    // xacti specific publishers
    Canard::Publisher<com_xacti_CopterAttStatus> xacti_copter_att_status{canard_iface};
    Canard::Publisher<com_xacti_GimbalControlData> xacti_gimbal_control_data{canard_iface};
    Canard::Publisher<com_xacti_GnssStatus> xacti_gnss_status{canard_iface};

private:
    void loop(void);

    ///// SRV output /////
    void SRV_send_actuator();
    void SRV_send_esc();
    void SRV_send_himark();

    //scale servo output appropriately before sending
    int16_t scale_esc_output(uint8_t idx);

    // SafetyState
    void safety_state_send();

    // send notify vehicle state
    void notify_state_send();

    // check for parameter get/set response timeout
    void check_parameter_callback_timeout();

    // send queued parameter get/set request. called from loop
    void send_parameter_request();
    
    // send queued parameter save request. called from loop
    void send_parameter_save_request();

    // periodic logging
    void logging();
    
    // get parameter on a node
    ParamGetSetIntCb *param_int_cb;         // latest get param request callback function (for integers)
    ParamGetSetFloatCb *param_float_cb;     // latest get param request callback function (for floats)
    bool param_request_sent = true;         // true after a param request has been sent, false when queued to be sent
    uint32_t param_request_sent_ms;         // system time that get param request was sent
    HAL_Semaphore _param_sem;               // semaphore protecting this block of variables
    uint8_t param_request_node_id;          // node id of most recent get param request

    // save parameters on a node
    ParamSaveCb *save_param_cb;             // latest save param request callback function
    bool param_save_request_sent = true;    // true after a save param request has been sent, false when queued to be sent
    HAL_Semaphore _param_save_sem;          // semaphore protecting this block of variables
    uint8_t param_save_request_node_id;     // node id of most recent save param request

    // UAVCAN parameters
    AP_Int8 _dronecan_node;
    AP_Int32 _servo_bm;
    AP_Int32 _esc_bm;
    AP_Int8 _esc_offset;
    AP_Int16 _servo_rate_hz;
    AP_Int16 _options;
    AP_Int16 _notify_state_hz;
    AP_Int16 _pool_size;
    AP_Int32 _esc_rv;

    uint32_t *mem_pool;

    AP_DroneCAN_DNA_Server _dna_server;

    uint8_t _driver_index;

    char _thread_name[13];
    bool _initialized;
    ///// SRV output /////
    struct {
        uint16_t pulse;
        bool esc_pending;
        bool servo_pending;
    } _SRV_conf[DRONECAN_SRV_NUMBER];

    uint32_t _esc_send_count;
    uint32_t _srv_send_count;
    uint32_t _fail_send_count;

    uint32_t _SRV_armed_mask; // mask of servo outputs that are active
    uint32_t _ESC_armed_mask; // mask of ESC outputs that are active
    uint32_t _SRV_last_send_us;
    HAL_Semaphore SRV_sem;

    // last log time
    uint32_t last_log_ms;

#if AP_DRONECAN_SEND_GPS
    // send GNSS Fix and yaw, same thing AP_GPS_DroneCAN would receive
    void gnss_send_fix();
    void gnss_send_yaw();
    
    // GNSS Fix and Status
    struct {
        uint32_t last_gps_lib_fix_ms;
        uint32_t last_send_status_ms;
        uint32_t last_lib_yaw_time_ms;
    } _gnss;
#endif

    // node status send
    uint32_t _node_status_last_send_ms;

    // safety status send state
    uint32_t _last_safety_state_ms;

    // notify vehicle state
    uint32_t _last_notify_state_ms;
    uavcan_protocol_NodeStatus node_status_msg;

    CanardInterface canard_iface;

    Canard::Publisher<uavcan_protocol_NodeStatus> node_status{canard_iface};
    Canard::Publisher<dronecan_protocol_CanStats> can_stats{canard_iface};
    Canard::Publisher<dronecan_protocol_Stats> protocol_stats{canard_iface};
    Canard::Publisher<uavcan_equipment_actuator_ArrayCommand> act_out_array{canard_iface};
    Canard::Publisher<uavcan_equipment_esc_RawCommand> esc_raw{canard_iface};
    Canard::Publisher<ardupilot_indication_SafetyState> safety_state{canard_iface};
    Canard::Publisher<uavcan_equipment_safety_ArmingStatus> arming_status{canard_iface};
    Canard::Publisher<ardupilot_indication_NotifyState> notify_state{canard_iface};
    Canard::Publisher<com_himark_servo_ServoCmd> himark_out{canard_iface};

#if AP_DRONECAN_SEND_GPS
    Canard::Publisher<uavcan_equipment_gnss_Fix2> gnss_fix2{canard_iface};
    Canard::Publisher<uavcan_equipment_gnss_Auxiliary> gnss_auxiliary{canard_iface};
    Canard::Publisher<ardupilot_gnss_Heading> gnss_heading{canard_iface};
    Canard::Publisher<ardupilot_gnss_Status> gnss_status{canard_iface};
#endif
    // incoming messages
    Canard::ObjCallback<AP_DroneCAN, ardupilot_indication_Button> safety_button_cb{this, &AP_DroneCAN::handle_button};
    Canard::Subscriber<ardupilot_indication_Button> safety_button_listener{safety_button_cb, _driver_index};

    Canard::ObjCallback<AP_DroneCAN, ardupilot_equipment_trafficmonitor_TrafficReport> traffic_report_cb{this, &AP_DroneCAN::handle_traffic_report};
    Canard::Subscriber<ardupilot_equipment_trafficmonitor_TrafficReport> traffic_report_listener{traffic_report_cb, _driver_index};

    Canard::ObjCallback<AP_DroneCAN, uavcan_equipment_actuator_Status> actuator_status_cb{this, &AP_DroneCAN::handle_actuator_status};
    Canard::Subscriber<uavcan_equipment_actuator_Status> actuator_status_listener{actuator_status_cb, _driver_index};

    Canard::ObjCallback<AP_DroneCAN, uavcan_equipment_esc_Status> esc_status_cb{this, &AP_DroneCAN::handle_ESC_status};
    Canard::Subscriber<uavcan_equipment_esc_Status> esc_status_listener{esc_status_cb, _driver_index};

    Canard::ObjCallback<AP_DroneCAN, uavcan_protocol_debug_LogMessage> debug_cb{this, &AP_DroneCAN::handle_debug};
    Canard::Subscriber<uavcan_protocol_debug_LogMessage> debug_listener{debug_cb, _driver_index};

    // param client
    Canard::ObjCallback<AP_DroneCAN, uavcan_protocol_param_GetSetResponse> param_get_set_res_cb{this, &AP_DroneCAN::handle_param_get_set_response};
    Canard::Client<uavcan_protocol_param_GetSetResponse> param_get_set_client{canard_iface, param_get_set_res_cb};

    Canard::ObjCallback<AP_DroneCAN, uavcan_protocol_param_ExecuteOpcodeResponse> param_save_res_cb{this, &AP_DroneCAN::handle_param_save_response};
    Canard::Client<uavcan_protocol_param_ExecuteOpcodeResponse> param_save_client{canard_iface, param_save_res_cb};

    // reboot client
    void handle_restart_node_response(const CanardRxTransfer& transfer, const uavcan_protocol_RestartNodeResponse& msg) {}
    Canard::ObjCallback<AP_DroneCAN, uavcan_protocol_RestartNodeResponse> restart_node_res_cb{this, &AP_DroneCAN::handle_restart_node_response};
    Canard::Client<uavcan_protocol_RestartNodeResponse> restart_node_client{canard_iface, restart_node_res_cb};

    uavcan_protocol_param_ExecuteOpcodeRequest param_save_req;
    uavcan_protocol_param_GetSetRequest param_getset_req;

    // Node Info Server
    Canard::ObjCallback<AP_DroneCAN, uavcan_protocol_GetNodeInfoRequest> node_info_req_cb{this, &AP_DroneCAN::handle_node_info_request};
    Canard::Server<uavcan_protocol_GetNodeInfoRequest> node_info_server{canard_iface, node_info_req_cb};
    uavcan_protocol_GetNodeInfoResponse node_info_rsp;

#if AP_DRONECAN_HOBBYWING_ESC_SUPPORT
    /*
      Hobbywing ESC support. Note that we need additional meta-data as
      the status messages do not have an ESC ID in them, so we need a
      mapping from node ID
    */
    #define HOBBYWING_MAX_ESC 8
    struct {
        uint32_t last_GetId_send_ms;
        uint8_t thr_chan[HOBBYWING_MAX_ESC];
    } hobbywing;
    void hobbywing_ESC_update();

    void SRV_send_esc_hobbywing();
    Canard::Publisher<com_hobbywing_esc_RawCommand> esc_hobbywing_raw{canard_iface};
    Canard::Publisher<com_hobbywing_esc_GetEscID> esc_hobbywing_GetEscID{canard_iface};
    Canard::ObjCallback<AP_DroneCAN, com_hobbywing_esc_GetEscID> esc_hobbywing_GetEscID_cb{this, &AP_DroneCAN::handle_hobbywing_GetEscID};
    Canard::Subscriber<com_hobbywing_esc_GetEscID> esc_hobbywing_GetEscID_listener{esc_hobbywing_GetEscID_cb, _driver_index};
    Canard::ObjCallback<AP_DroneCAN, com_hobbywing_esc_StatusMsg1> esc_hobbywing_StatusMSG1_cb{this, &AP_DroneCAN::handle_hobbywing_StatusMsg1};
    Canard::Subscriber<com_hobbywing_esc_StatusMsg1> esc_hobbywing_StatusMSG1_listener{esc_hobbywing_StatusMSG1_cb, _driver_index};
    Canard::ObjCallback<AP_DroneCAN, com_hobbywing_esc_StatusMsg2> esc_hobbywing_StatusMSG2_cb{this, &AP_DroneCAN::handle_hobbywing_StatusMsg2};
    Canard::Subscriber<com_hobbywing_esc_StatusMsg2> esc_hobbywing_StatusMSG2_listener{esc_hobbywing_StatusMSG2_cb, _driver_index};
    bool hobbywing_find_esc_index(uint8_t node_id, uint8_t &esc_index) const;
    void handle_hobbywing_GetEscID(const CanardRxTransfer& transfer, const com_hobbywing_esc_GetEscID& msg);
    void handle_hobbywing_StatusMsg1(const CanardRxTransfer& transfer, const com_hobbywing_esc_StatusMsg1& msg);
    void handle_hobbywing_StatusMsg2(const CanardRxTransfer& transfer, const com_hobbywing_esc_StatusMsg2& msg);
#endif // AP_DRONECAN_HOBBYWING_ESC_SUPPORT
    
    // incoming button handling
    void handle_button(const CanardRxTransfer& transfer, const ardupilot_indication_Button& msg);
    void handle_traffic_report(const CanardRxTransfer& transfer, const ardupilot_equipment_trafficmonitor_TrafficReport& msg);
    void handle_actuator_status(const CanardRxTransfer& transfer, const uavcan_equipment_actuator_Status& msg);
    void handle_actuator_status_Volz(const CanardRxTransfer& transfer, const com_volz_servo_ActuatorStatus& msg);
    void handle_ESC_status(const CanardRxTransfer& transfer, const uavcan_equipment_esc_Status& msg);
    void handle_himark_servoinfo(const CanardRxTransfer& transfer, const com_himark_servo_ServoInfo &msg);
    static bool is_esc_data_index_valid(const uint8_t index);
    void handle_debug(const CanardRxTransfer& transfer, const uavcan_protocol_debug_LogMessage& msg);
    void handle_param_get_set_response(const CanardRxTransfer& transfer, const uavcan_protocol_param_GetSetResponse& rsp);
    void handle_param_save_response(const CanardRxTransfer& transfer, const uavcan_protocol_param_ExecuteOpcodeResponse& rsp);
    void handle_node_info_request(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoRequest& req);
};

#endif // #if HAL_ENABLE_DRONECAN_DRIVERS
