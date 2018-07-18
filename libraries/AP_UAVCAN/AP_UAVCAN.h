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

#include <AP_HAL/CAN.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>

#include <uavcan/helpers/heap_based_pool_allocator.hpp>

#ifndef UAVCAN_NODE_POOL_SIZE
#define UAVCAN_NODE_POOL_SIZE 8192
#endif

#ifndef UAVCAN_NODE_POOL_BLOCK_SIZE
#define UAVCAN_NODE_POOL_BLOCK_SIZE 256
#endif

#ifndef UAVCAN_SRV_NUMBER
#define UAVCAN_SRV_NUMBER 18
#endif

#define AP_UAVCAN_SW_VERS_MAJOR 1
#define AP_UAVCAN_SW_VERS_MINOR 0

#define AP_UAVCAN_HW_VERS_MAJOR 1
#define AP_UAVCAN_HW_VERS_MINOR 0

#define AP_UAVCAN_MAX_LED_DEVICES 4

class AP_UAVCAN : public AP_HAL::CANProtocol {
public:
    AP_UAVCAN();
    ~AP_UAVCAN();

    static const struct AP_Param::GroupInfo var_info[];

    // Return uavcan from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_UAVCAN *get_uavcan(uint8_t driver_index);

    void init(uint8_t driver_index) override;


    ///// SRV output /////
    void SRV_push_servos(void);

    ///// LED /////
    bool led_write(uint8_t led_index, uint8_t red, uint8_t green, uint8_t blue);

    uavcan::Node<0>* get_node() { return _node; }
    uint8_t get_driver_num() { return _driver_num; }
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


    // UAVCAN parameters
    AP_Int8 _uavcan_node;
    AP_Int32 _servo_bm;
    AP_Int32 _esc_bm;
    AP_Int16 _servo_rate_hz;


    uavcan::Node<0> *_node;
    uavcan::HeapBasedPoolAllocator<UAVCAN_NODE_POOL_BLOCK_SIZE, AP_UAVCAN::RaiiSynchronizer> _node_allocator;
    uint8_t _driver_index;
    char _thread_name[9];
    bool _initialized;


    ///// SRV output /////
    struct {
        uint16_t pulse;
        bool esc_pending;
        bool servo_pending;
    } _SRV_conf[UAVCAN_SRV_NUMBER];

    uint8_t _SRV_armed;
    uint32_t _SRV_last_send_us;
    AP_HAL::Semaphore *SRV_sem;

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

    AP_HAL::Semaphore *_led_out_sem;
    /*
        Frontend Backend-Registry Binder: Whenever a message of said DataType_ from new node is received,
        The Callback will invoke frontend to register the node as separate backend.

        Note: Generic Subscriptions should not be done using this mechanism, i.e. messages for which
        you don't want Frontend to init a new Backend when they come from different Nodes.
        Use uavcan::MethodBinder or util/Functor template instead and initialise the subscriber in the Backend Constructor.
    */
    template <typename DataType_, typename Backend_, typename Frontend_>
    class FrontendRegistryBinder {
        AP_UAVCAN* _uc;
        void (Backend_::*_bfunc)(const uavcan::ReceivedDataStructure<DataType_> &);
        Backend_* (Frontend_::*_ffunc)(AP_UAVCAN*, uint8_t);
        Frontend_* _frontend;

    public:
        FrontendRegistryBinder()
            : _uc(),
              _bfunc(),
              _ffunc(),
              _frontend() {}
        FrontendRegistryBinder(AP_UAVCAN* uc, void (Backend_::*bfunc)(const uavcan::ReceivedDataStructure<DataType_> &),
                               Backend_* (Frontend_::*ffunc)(AP_UAVCAN*, uint8_t), Frontend_ *frontend):
            _uc(uc),
            _bfunc(bfunc),
            _ffunc(ffunc),
            _frontend(frontend) {}

        void operator()(const uavcan::ReceivedDataStructure<DataType_> & msg)
        {
            Backend_* bk = (_frontend->*_ffunc)(_uc, msg.getSrcNodeID().get());
            if (bk != nullptr) {
                (bk->*_bfunc)(msg);
            }
        }
    };
};

#endif /* AP_UAVCAN_H_ */
