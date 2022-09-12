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

#if HAL_ENABLE_LIBUAVCAN_DRIVERS

#include <uavcan/uavcan.hpp>
#include "AP_UAVCAN_IfaceMgr.h"
#include "AP_UAVCAN_Clock.h"
#include <AP_CANManager/AP_CANManager.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>
#include <uavcan/protocol/param/GetSet.hpp>
#include <uavcan/protocol/param/ExecuteOpcode.hpp>
#include <SRV_Channel/SRV_Channel.h>


#ifndef UAVCAN_SRV_NUMBER
#define UAVCAN_SRV_NUMBER NUM_SERVO_CHANNELS
#endif

#define AP_UAVCAN_SW_VERS_MAJOR 1
#define AP_UAVCAN_SW_VERS_MINOR 0

#define AP_UAVCAN_HW_VERS_MAJOR 1
#define AP_UAVCAN_HW_VERS_MINOR 0

#define AP_UAVCAN_MAX_LED_DEVICES 4

// fwd-declare callback classes
class ButtonCb;
class TrafficReportCb;
class ActuatorStatusCb;
class ESCStatusCb;
class DebugCb;
class ParamGetSetCb;
class ParamExecuteOpcodeCb;
class AP_PoolAllocator;
class AP_UAVCAN_DNA_Server;

#if defined(__GNUC__) && (__GNUC__ > 8)
#define DISABLE_W_CAST_FUNCTION_TYPE_PUSH \
    _Pragma("GCC diagnostic push") \
    _Pragma("GCC diagnostic ignored \"-Wcast-function-type\"")
#define DISABLE_W_CAST_FUNCTION_TYPE_POP \
    _Pragma("GCC diagnostic pop")
#else
#define DISABLE_W_CAST_FUNCTION_TYPE_PUSH
#define DISABLE_W_CAST_FUNCTION_TYPE_POP
#endif
#if defined(__GNUC__) && (__GNUC__ >= 11)
#define DISABLE_W_CAST_FUNCTION_TYPE_WITH_VOID (void*)
#else
#define DISABLE_W_CAST_FUNCTION_TYPE_WITH_VOID
#endif

/*
    Frontend Backend-Registry Binder: Whenever a message of said DataType_ from new node is received,
    the Callback will invoke registry to register the node as separate backend.
*/
#define UC_REGISTRY_BINDER(ClassName_, DataType_) \
    class ClassName_ : public AP_UAVCAN::RegistryBinder<DataType_> { \
        typedef void (*CN_Registry)(AP_UAVCAN*, uint8_t, const ClassName_&); \
        public: \
            ClassName_() : RegistryBinder() {} \
            DISABLE_W_CAST_FUNCTION_TYPE_PUSH \
            ClassName_(AP_UAVCAN* uc,  CN_Registry ffunc) : \
                RegistryBinder(uc, (Registry)DISABLE_W_CAST_FUNCTION_TYPE_WITH_VOID ffunc) {} \
            DISABLE_W_CAST_FUNCTION_TYPE_POP \
    }

#define UC_CLIENT_CALL_REGISTRY_BINDER(ClassName_, DataType_) \
    class ClassName_ : public AP_UAVCAN::ClientCallRegistryBinder<DataType_> { \
        typedef void (*CN_Registry)(AP_UAVCAN*, uint8_t, const ClassName_&); \
        public: \
            ClassName_() : ClientCallRegistryBinder() {} \
            DISABLE_W_CAST_FUNCTION_TYPE_PUSH \
            ClassName_(AP_UAVCAN* uc,  CN_Registry ffunc) : \
                ClientCallRegistryBinder(uc, (ClientCallRegistry)DISABLE_W_CAST_FUNCTION_TYPE_WITH_VOID ffunc) {} \
            DISABLE_W_CAST_FUNCTION_TYPE_POP \
    }

class AP_UAVCAN : public AP_CANDriver, public AP_ESC_Telem_Backend {
    friend class AP_UAVCAN_DNA_Server;
public:
    AP_UAVCAN();
    ~AP_UAVCAN();

    static const struct AP_Param::GroupInfo var_info[];

    // Return uavcan from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_UAVCAN *get_uavcan(uint8_t driver_index);
    bool prearm_check(char* fail_msg, uint8_t fail_msg_len) const;

    void init(uint8_t driver_index, bool enable_filters) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    uavcan::Node<0>* get_node() { return _node; }
    uint8_t get_driver_index() const { return _driver_index; }

    FUNCTOR_TYPEDEF(ParamGetSetIntCb, bool, AP_UAVCAN*, const uint8_t, const char*, int32_t &);
    FUNCTOR_TYPEDEF(ParamGetSetFloatCb, bool, AP_UAVCAN*, const uint8_t, const char*, float &);
    FUNCTOR_TYPEDEF(ParamSaveCb, void, AP_UAVCAN*,  const uint8_t, bool);

    ///// SRV output /////
    void SRV_push_servos(void);

    ///// LED /////
    bool led_write(uint8_t led_index, uint8_t red, uint8_t green, uint8_t blue);

    // buzzer
    void set_buzzer_tone(float frequency, float duration_s);

    // send RTCMStream packets
    void send_RTCMStream(const uint8_t *data, uint32_t len);

    // Send Reboot command
    // Note: Do not call this from outside UAVCAN thread context,
    // you can call this from uavcan callbacks and handlers.
    // THIS IS NOT A THREAD SAFE API!
    void send_reboot_request(uint8_t node_id);

    // set param value
    bool set_parameter_on_node(uint8_t node_id, const char *name, float value, ParamGetSetFloatCb *cb);
    bool set_parameter_on_node(uint8_t node_id, const char *name, int32_t value, ParamGetSetIntCb *cb);
    bool get_parameter_on_node(uint8_t node_id, const char *name, ParamGetSetFloatCb *cb);
    bool get_parameter_on_node(uint8_t node_id, const char *name, ParamGetSetIntCb *cb);

    // Save parameters
    bool save_parameters_on_node(uint8_t node_id, ParamSaveCb *cb);

    template <typename DataType_>
    class RegistryBinder {
    protected:
        typedef void (*Registry)(AP_UAVCAN* _ap_uavcan, uint8_t _node_id, const RegistryBinder& _cb);
        AP_UAVCAN* _uc;
        Registry _ffunc;

    public:
        RegistryBinder() :
            _uc(),
            _ffunc(),
            msg() {}

        RegistryBinder(AP_UAVCAN* uc, Registry ffunc) :
            _uc(uc),
            _ffunc(ffunc),
            msg(nullptr) {}

        void operator()(const uavcan::ReceivedDataStructure<DataType_>& _msg) {
            msg = &_msg;
            _ffunc(_uc, _msg.getSrcNodeID().get(), *this);
        }

        const uavcan::ReceivedDataStructure<DataType_> *msg;
    };

    // ClientCallRegistryBinder
    template <typename DataType_>
    class ClientCallRegistryBinder {
    protected:
        typedef void (*ClientCallRegistry)(AP_UAVCAN* _ap_uavcan, uint8_t _node_id, const ClientCallRegistryBinder& _cb);
        AP_UAVCAN* _uc;
        ClientCallRegistry _ffunc;
    public:
        ClientCallRegistryBinder() :
            _uc(),
            _ffunc(),
            rsp() {}

        ClientCallRegistryBinder(AP_UAVCAN* uc, ClientCallRegistry ffunc) :
            _uc(uc),
            _ffunc(ffunc),
            rsp(nullptr) {}

        void operator()(const uavcan::ServiceCallResult<DataType_>& _rsp) {
            rsp = &_rsp;
            _ffunc(_uc, _rsp.getCallID().server_node_id.get(), *this);
        }
        const uavcan::ServiceCallResult<DataType_> *rsp;
    };

    // options bitmask
    enum class Options : uint16_t {
        DNA_CLEAR_DATABASE        = (1U<<0),
        DNA_IGNORE_DUPLICATE_NODE = (1U<<1),
        CANFD_ENABLED             = (1U<<2),
        DNA_IGNORE_UNHEALTHY_NODE = (1U<<3),
        USE_ACTUATOR_PWM          = (1U<<4),
    };

    // check if a option is set
    bool option_is_set(Options option) const {
        return (uint16_t(_options.get()) & uint16_t(option)) != 0;
    }

    // check if a option is set and if it is then reset it to
    // 0. return true if it was set
    bool check_and_reset_option(Options option);

    // This will be needed to implement if UAVCAN is used with multithreading
    // Such cases will be firmware update, etc.
    class RaiiSynchronizer {};

private:
    void loop(void);

    ///// SRV output /////
    void SRV_send_actuator();
    void SRV_send_esc();

    ///// LED /////
    void led_out_send();

    // buzzer
    void buzzer_send();

    // SafetyState
    void safety_state_send();

    // send notify vehicle state
    void notify_state_send();

    // send GNSS injection
    void rtcm_stream_send();

    // send parameter get/set request
    void send_parameter_request();
    
    // send parameter save request
    void send_parameter_save_request();

    // set parameter on a node
    ParamGetSetIntCb *param_int_cb;
    ParamGetSetFloatCb *param_float_cb;
    bool param_request_sent = true;
    HAL_Semaphore _param_sem;
    uint8_t param_request_node_id;

    // save parameters on a node
    ParamSaveCb *save_param_cb;
    bool param_save_request_sent = true;
    HAL_Semaphore _param_save_sem;
    uint8_t param_save_request_node_id;

    // UAVCAN parameters
    AP_Int8 _uavcan_node;
    AP_Int32 _servo_bm;
    AP_Int32 _esc_bm;
    AP_Int8 _esc_offset;
    AP_Int16 _servo_rate_hz;
    AP_Int16 _options;
    AP_Int16 _notify_state_hz;
    AP_Int16 _pool_size;

    AP_PoolAllocator *_allocator;
    AP_UAVCAN_DNA_Server *_dna_server;

    uavcan::Node<0> *_node;

    uint8_t _driver_index;

    uavcan::CanIfaceMgr* _iface_mgr;
    char _thread_name[13];
    bool _initialized;
    ///// SRV output /////
    struct {
        uint16_t pulse;
        bool esc_pending;
        bool servo_pending;
    } _SRV_conf[UAVCAN_SRV_NUMBER];

    uint8_t _SRV_armed;
    uint32_t _SRV_last_send_us;
    HAL_Semaphore SRV_sem;

    ///// LED /////
    struct led_device {
        uint8_t led_index;
        uint8_t red;
        uint8_t green;
        uint8_t blue;
    };

    struct {
        led_device devices[AP_UAVCAN_MAX_LED_DEVICES];
        uint8_t devices_count;
        uint64_t last_update;
    } _led_conf;

    HAL_Semaphore _led_out_sem;

    // buzzer
    struct {
        HAL_Semaphore sem;
        float frequency;
        float duration;
        uint8_t pending_mask; // mask of interfaces to send to
    } _buzzer;

    // GNSS RTCM injection
    struct {
        HAL_Semaphore sem;
        uint32_t last_send_ms;
        ByteBuffer *buf;
    } _rtcm_stream;
    
     // ESC

    static HAL_Semaphore _telem_sem;

    // safety status send state
    uint32_t _last_safety_state_ms;

    // notify vehicle state
    uint32_t _last_notify_state_ms;

    // incoming button handling
    static void handle_button(AP_UAVCAN* ap_uavcan, uint8_t node_id, const ButtonCb &cb);
    static void handle_traffic_report(AP_UAVCAN* ap_uavcan, uint8_t node_id, const TrafficReportCb &cb);
    static void handle_actuator_status(AP_UAVCAN* ap_uavcan, uint8_t node_id, const ActuatorStatusCb &cb);
    static void handle_ESC_status(AP_UAVCAN* ap_uavcan, uint8_t node_id, const ESCStatusCb &cb);
    static bool is_esc_data_index_valid(const uint8_t index);
    static void handle_debug(AP_UAVCAN* ap_uavcan, uint8_t node_id, const DebugCb &cb);
    static void handle_param_get_set_response(AP_UAVCAN* ap_uavcan, uint8_t node_id, const ParamGetSetCb &cb);
    static void handle_param_save_response(AP_UAVCAN* ap_uavcan, uint8_t node_id, const ParamExecuteOpcodeCb &cb);
};

#endif // #if HAL_ENABLE_LIBUAVCAN_DRIVERS
