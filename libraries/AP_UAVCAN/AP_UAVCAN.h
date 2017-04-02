/*
 * AP_UAVCAN.h
 *
 *      Author: Eugene Shamaev
 */
#ifndef AP_UAVCAN_H_
#define AP_UAVCAN_H_

#include <uavcan/uavcan.hpp>

#include <AP_HAL/CAN.h>
#include <AP_HAL/Semaphores.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Param/AP_Param.h>

#include <AP_GPS/GPS_Backend.h>
#include <AP_Baro/AP_Baro_Backend.h>
#include <AP_Compass/AP_Compass.h>

#include <uavcan/helpers/heap_based_pool_allocator.hpp>

#ifndef UAVCAN_NODE_POOL_SIZE
#define UAVCAN_NODE_POOL_SIZE 8192
#endif

#ifndef UAVCAN_NODE_POOL_BLOCK_SIZE
#define UAVCAN_NODE_POOL_BLOCK_SIZE 256
#endif

#ifndef UAVCAN_RCO_NUMBER
#define UAVCAN_RCO_NUMBER 18
#endif

#define AP_UAVCAN_MAX_LISTENERS 4
#define AP_UAVCAN_MAX_GPS_NODES 4
#define AP_UAVCAN_MAX_MAG_NODES 4
#define AP_UAVCAN_MAX_BARO_NODES 4

#define AP_UAVCAN_SW_VERS_MAJOR 1
#define AP_UAVCAN_SW_VERS_MINOR 0

#define AP_UAVCAN_HW_VERS_MAJOR 1
#define AP_UAVCAN_HW_VERS_MINOR 0

class AP_UAVCAN {
public:
    AP_UAVCAN();
    ~AP_UAVCAN();

    static const struct AP_Param::GroupInfo var_info[];

    // this function will register the listening class on a first free channel or on the specified channel
    // if preferred_channel = 0 then free channel will be searched for
    // if preferred_channel > 0 then listener will be added to specific channel
    // return value is the number of assigned channel or 0 if fault
    // channel numbering starts from 1
    uint8_t register_gps_listener(AP_GPS_Backend* new_listener,
                                  uint8_t preferred_channel);
    // Removes specified listener from all nodes
    void remove_gps_listener(AP_GPS_Backend* rem_listener);

    // Returns pointer to GPS state connected with specified node.
    // If node is not found and there are free space, locate a new one
    AP_GPS::GPS_State *find_gps_node(uint8_t node);

    // Updates all listeners of specified node
    void update_gps_state(uint8_t node);

    struct Baro_Info {
        float pressure;
        float pressure_variance;
        float temperature;
        float temperature_variance;
    };

    uint8_t register_baro_listener(AP_Baro_Backend* new_listener,
                                   uint8_t preferred_channel);
    void remove_baro_listener(AP_Baro_Backend* rem_listener);
    Baro_Info *find_baro_node(uint8_t node);
    void update_baro_state(uint8_t node);

    struct Mag_Info {
        Vector3f mag_vector;
    };

    uint8_t register_mag_listener(AP_Compass_Backend* new_listener,
                                  uint8_t preferred_channel);
    void remove_mag_listener(AP_Compass_Backend* rem_listener);
    Mag_Info *find_mag_node(uint8_t node);
    void update_mag_state(uint8_t node);

    // synchronization for RC output
    bool rc_out_sem_take();
    void rc_out_sem_give();

private:
    // ------------------------- GPS
    // 255 - means free node
    uint8_t _gps_nodes[AP_UAVCAN_MAX_GPS_NODES];
    // Counter of how many listeners are connected to this source
    uint8_t _gps_node_taken[AP_UAVCAN_MAX_GPS_NODES];
    // GPS data of the sources
    AP_GPS::GPS_State _gps_node_state[AP_UAVCAN_MAX_GPS_NODES];

    // 255 - means no connection
    uint8_t _gps_listener_to_node[AP_UAVCAN_MAX_LISTENERS];
    // Listeners with callbacks to be updated
    AP_GPS_Backend* _gps_listeners[AP_UAVCAN_MAX_LISTENERS];

    // ------------------------- BARO
    uint8_t _baro_nodes[AP_UAVCAN_MAX_BARO_NODES];
    uint8_t _baro_node_taken[AP_UAVCAN_MAX_BARO_NODES];
    Baro_Info _baro_node_state[AP_UAVCAN_MAX_BARO_NODES];
    uint8_t _baro_listener_to_node[AP_UAVCAN_MAX_LISTENERS];
    AP_Baro_Backend* _baro_listeners[AP_UAVCAN_MAX_LISTENERS];

    // ------------------------- MAG
    uint8_t _mag_nodes[AP_UAVCAN_MAX_MAG_NODES];
    uint8_t _mag_node_taken[AP_UAVCAN_MAX_MAG_NODES];
    Mag_Info _mag_node_state[AP_UAVCAN_MAX_MAG_NODES];
    uint8_t _mag_listener_to_node[AP_UAVCAN_MAX_LISTENERS];
    AP_Compass_Backend* _mag_listeners[AP_UAVCAN_MAX_LISTENERS];

    struct {
        uint16_t pulse;
        uint16_t safety_pulse;
        uint16_t failsafe_pulse;
        bool active;
    } _rco_conf[UAVCAN_RCO_NUMBER];

    bool _initialized;
    uint8_t _rco_armed;
    uint8_t _rco_safety;

    AP_HAL::Semaphore *_rc_out_sem;

    class SystemClock: public uavcan::ISystemClock, uavcan::Noncopyable {
        SystemClock()
        {
        }

        uavcan::UtcDuration utc_adjustment;
        virtual void adjustUtc(uavcan::UtcDuration adjustment)
        {
            utc_adjustment = adjustment;
        }

    public:
        virtual uavcan::MonotonicTime getMonotonic() const
        {
            uavcan::uint64_t usec = 0;
            usec = AP_HAL::micros64();
            return uavcan::MonotonicTime::fromUSec(usec);
        }
        virtual uavcan::UtcTime getUtc() const
        {
            uavcan::UtcTime utc;
            uavcan::uint64_t usec = 0;
            usec = AP_HAL::micros64();
            utc.fromUSec(usec);
            utc += utc_adjustment;
            return utc;
        }

        static SystemClock& instance()
        {
            static SystemClock inst;
            return inst;
        }
    };

    uavcan::Node<0> *_node = nullptr;

    uavcan::ISystemClock& get_system_clock();
    uavcan::ICanDriver* get_can_driver();
    uavcan::Node<0>* get_node();

    // This will be needed to implement if UAVCAN is used with multithreading
    // Such cases will be firmware update, etc.
    class RaiiSynchronizer {
    public:
        RaiiSynchronizer()
        {
        }

        ~RaiiSynchronizer()
        {
        }
    };

    uavcan::HeapBasedPoolAllocator<UAVCAN_NODE_POOL_BLOCK_SIZE, AP_UAVCAN::RaiiSynchronizer> _node_allocator;

    AP_Int8 _uavcan_node;

public:
    void do_cyclic(void);
    bool try_init(void);

    void rco_set_safety_pwm(uint32_t chmask, uint16_t pulse_len);
    void rco_set_failsafe_pwm(uint32_t chmask, uint16_t pulse_len);
    void rco_force_safety_on(void);
    void rco_force_safety_off(void);
    void rco_arm_actuators(bool arm);
    void rco_write(uint16_t pulse_len, uint8_t ch);
};

#endif /* AP_UAVCAN_H_ */
