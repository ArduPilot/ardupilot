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
#ifndef AP_UAVCAN_H_
#define AP_UAVCAN_H_

#include <uavcan/uavcan.hpp>
#include "AP_UAVCAN_DNA_Server.h"

#include <AP_HAL/CAN.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>

#include <uavcan/helpers/heap_based_pool_allocator.hpp>

#ifndef UAVCAN_NODE_POOL_SIZE
#define UAVCAN_NODE_POOL_SIZE 8192
#endif

#ifndef UAVCAN_NODE_POOL_BLOCK_SIZE
#define UAVCAN_NODE_POOL_BLOCK_SIZE 64
#endif

#ifndef UAVCAN_SRV_NUMBER
#define UAVCAN_SRV_NUMBER 18
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

/*
    Frontend Backend-Registry Binder: Whenever a message of said DataType_ from new node is received,
    the Callback will invoke registery to register the node as separate backend.
*/
#define UC_REGISTRY_BINDER(ClassName_, DataType_) \
	class ClassName_ : public AP_UAVCAN::RegistryBinder<DataType_> { \
        typedef void (*CN_Registry)(AP_UAVCAN*, uint8_t, const ClassName_&); \
	    public: \
	        ClassName_() : RegistryBinder() {} \
	        ClassName_(AP_UAVCAN* uc,  CN_Registry ffunc) : \
				RegistryBinder(uc, (Registry)ffunc) {} \
	}

class AP_UAVCAN : public AP_HAL::CANProtocol {
public:
    AP_UAVCAN();
    ~AP_UAVCAN();

    static const struct AP_Param::GroupInfo var_info[];

    // Return uavcan from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_UAVCAN *get_uavcan(uint8_t driver_index);

    void init(uint8_t driver_index, bool enable_filters) override;

    uavcan::Node<0>* get_node() { return _node; }
    uint8_t get_driver_index() { return _driver_index; }


    ///// SRV output /////
    void SRV_push_servos(void);

    ///// LED /////
    bool led_write(uint8_t led_index, uint8_t red, uint8_t green, uint8_t blue);

    // buzzer
    void set_buzzer_tone(float frequency, float duration_s);

    // send RTCMStream packets
    void send_RTCMStream(const uint8_t *data, uint32_t len);

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

private:
    class SystemClock: public uavcan::ISystemClock, uavcan::Noncopyable {
    public:
        SystemClock() = default;

        void adjustUtc(uavcan::UtcDuration adjustment) override {
            utc_adjustment_usec = adjustment.toUSec();
        }

        uavcan::MonotonicTime getMonotonic() const override {
            return uavcan::MonotonicTime::fromUSec(AP_HAL::micros64());
        }

        uavcan::UtcTime getUtc() const override {
            return uavcan::UtcTime::fromUSec(AP_HAL::micros64() + utc_adjustment_usec);
        }

        static SystemClock& instance() {
            static SystemClock inst;
            return inst;
        }

    private:
        int64_t utc_adjustment_usec;
    };

    // This will be needed to implement if UAVCAN is used with multithreading
    // Such cases will be firmware update, etc.
    class RaiiSynchronizer {};

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

    // send GNSS injection
    void rtcm_stream_send();

    uavcan::PoolAllocator<UAVCAN_NODE_POOL_SIZE, UAVCAN_NODE_POOL_BLOCK_SIZE, AP_UAVCAN::RaiiSynchronizer> _node_allocator;

    // UAVCAN parameters
    AP_Int8 _uavcan_node;
    AP_Int32 _servo_bm;
    AP_Int32 _esc_bm;
    AP_Int16 _servo_rate_hz;

    uavcan::Node<0> *_node;

    uint8_t _driver_index;
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
    
    // safety status send state
    uint32_t _last_safety_state_ms;

    // safety button handling
    static void handle_button(AP_UAVCAN* ap_uavcan, uint8_t node_id, const ButtonCb &cb);
    static void handle_traffic_report(AP_UAVCAN* ap_uavcan, uint8_t node_id, const TrafficReportCb &cb);
    static void handle_actuator_status(AP_UAVCAN* ap_uavcan, uint8_t node_id, const ActuatorStatusCb &cb);
    static void handle_ESC_status(AP_UAVCAN* ap_uavcan, uint8_t node_id, const ESCStatusCb &cb);
};

#endif /* AP_UAVCAN_H_ */
