/*
  simple UAVCAN network sniffer as an ArduPilot firmware
 */
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && HAL_WITH_UAVCAN

#include <uavcan/uavcan.hpp>

#include <AP_HAL/CAN.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL_ChibiOS/CAN.h>

#include <uavcan/helpers/heap_based_pool_allocator.hpp>
#include <uavcan/equipment/indication/RGB565.hpp>

#include <uavcan/equipment/gnss/Fix.hpp>
#include <uavcan/equipment/gnss/Auxiliary.hpp>

#include <uavcan/equipment/ahrs/MagneticFieldStrength.hpp>
#include <uavcan/equipment/ahrs/MagneticFieldStrength2.hpp>
#include <uavcan/equipment/air_data/StaticPressure.hpp>
#include <uavcan/equipment/air_data/StaticTemperature.hpp>

#include <uavcan/equipment/actuator/ArrayCommand.hpp>
#include <uavcan/equipment/actuator/Command.hpp>
#include <uavcan/equipment/actuator/Status.hpp>

#include <uavcan/equipment/esc/RawCommand.hpp>
#include <uavcan/equipment/indication/LightsCommand.hpp>
#include <uavcan/equipment/indication/SingleLightCommand.hpp>
#include <uavcan/equipment/indication/RGB565.hpp>

#include <uavcan/equipment/power/BatteryInfo.hpp>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define UAVCAN_NODE_POOL_SIZE 8192
#define UAVCAN_NODE_POOL_BLOCK_SIZE 256

#define debug_uavcan(level, fmt, args...) do { hal.console->printf(fmt, ##args); } while (0)

class UAVCAN_sniffer {
public:
    UAVCAN_sniffer();
    ~UAVCAN_sniffer();

    void init(void);
    void loop(void);
    void print_stats(void);
    
private:
    AP_HAL::Semaphore *_led_out_sem;

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

    uavcan::Node<0> *_node;

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

    uavcan::HeapBasedPoolAllocator<UAVCAN_NODE_POOL_BLOCK_SIZE, UAVCAN_sniffer::RaiiSynchronizer> _node_allocator;

    AP_HAL::CANManager* _parent_can_mgr;

    void set_parent_can_mgr(AP_HAL::CANManager* parent_can_mgr)
    {
        _parent_can_mgr = parent_can_mgr;
    }
};

static struct {
    const char *msg_name;
    uint32_t count;
} counters[100];

static void count_msg(const char *name)
{
    for (uint16_t i=0; i<ARRAY_SIZE_SIMPLE(counters); i++) {
        if (counters[i].msg_name == name) {
            counters[i].count++;
            break;
        }
        if (counters[i].msg_name == nullptr) {
            counters[i].msg_name = name;
            counters[i].count++;
            break;
        }
    }
}

#define MSG_CB(mtype, cbname)                                                  \
    static void cb_ ## cbname(const uavcan::ReceivedDataStructure<mtype>& msg) { count_msg(msg.getDataTypeFullName()); }

MSG_CB(uavcan::protocol::NodeStatus, NodeStatus)
MSG_CB(uavcan::equipment::gnss::Fix, Fix)
MSG_CB(uavcan::equipment::ahrs::MagneticFieldStrength, MagneticFieldStrength)
MSG_CB(uavcan::equipment::air_data::StaticPressure, StaticPressure)
MSG_CB(uavcan::equipment::air_data::StaticTemperature, StaticTemperature)
MSG_CB(uavcan::equipment::gnss::Auxiliary, Auxiliary)
MSG_CB(uavcan::equipment::actuator::ArrayCommand, ArrayCommand)
MSG_CB(uavcan::equipment::esc::RawCommand, RawCommand)

void UAVCAN_sniffer::init(void)
{
    uint8_t inum = 0;
    const_cast <AP_HAL::HAL&> (hal).can_mgr[inum] = new ChibiOS::CANManager;
    hal.can_mgr[0]->begin(1000000, inum);

    set_parent_can_mgr(hal.can_mgr[inum]);
    
    if (!_parent_can_mgr->is_initialized()) {
        hal.console->printf("Can not initialised\n");
        return;
    }

    auto *node = get_node();

    uavcan::NodeID self_node_id(9);
    node->setNodeID(self_node_id);
    
    char ndname[20];
    snprintf(ndname, sizeof(ndname), "org.ardupilot:%u", inum);
            
    uavcan::NodeStatusProvider::NodeName name(ndname);
    node->setName(name);
            
    uavcan::protocol::SoftwareVersion sw_version; // Standard type uavcan.protocol.SoftwareVersion
    sw_version.major = AP_UAVCAN_SW_VERS_MAJOR;
    sw_version.minor = AP_UAVCAN_SW_VERS_MINOR;
    node->setSoftwareVersion(sw_version);
    
    uavcan::protocol::HardwareVersion hw_version; // Standard type uavcan.protocol.HardwareVersion
    
    hw_version.major = AP_UAVCAN_HW_VERS_MAJOR;
    hw_version.minor = AP_UAVCAN_HW_VERS_MINOR;
    node->setHardwareVersion(hw_version);
    
    const int node_start_res = node->start();
    if (node_start_res < 0) {
        debug_uavcan(1, "UAVCAN: node start problem\n\r");
    }

#define START_CB(mtype, cbname) (new uavcan::Subscriber<mtype>(*node))->start(cb_ ## cbname)
    
    START_CB(uavcan::protocol::NodeStatus, NodeStatus);
    START_CB(uavcan::equipment::gnss::Fix, Fix);
    START_CB(uavcan::equipment::ahrs::MagneticFieldStrength, MagneticFieldStrength);
    START_CB(uavcan::equipment::air_data::StaticPressure, StaticPressure);
    START_CB(uavcan::equipment::air_data::StaticTemperature, StaticTemperature);
    START_CB(uavcan::equipment::gnss::Auxiliary, Auxiliary);
    START_CB(uavcan::equipment::actuator::ArrayCommand, ArrayCommand);
    START_CB(uavcan::equipment::esc::RawCommand, RawCommand);
    
    /*
     * Informing other nodes that we're ready to work.
     * Default mode is INITIALIZING.
     */
    node->setModeOperational();
    
    debug_uavcan(1, "UAVCAN: init done\n\r");
}

uavcan::Node<0> *UAVCAN_sniffer::get_node()
{
    if (_node == nullptr && get_can_driver() != nullptr) {
        _node = new uavcan::Node<0>(*get_can_driver(), get_system_clock(), _node_allocator);
    }

    return _node;
}

uavcan::ICanDriver * UAVCAN_sniffer::get_can_driver()
{
    if (_parent_can_mgr != nullptr) {
        if (_parent_can_mgr->is_initialized() == false) {
            return nullptr;
        } else {
            return _parent_can_mgr->get_driver();
        }
    }
    return nullptr;
}

uavcan::ISystemClock & UAVCAN_sniffer::get_system_clock()
{
    return SystemClock::instance();
}

void UAVCAN_sniffer::loop(void)
{
    auto *node = get_node();
    node->spin(uavcan::MonotonicDuration::fromMSec(1));
}

void UAVCAN_sniffer::print_stats(void)
{
    for (uint16_t i=0;i<100;i++) {
        if (counters[i].msg_name == nullptr) {
            break;
        }
        hal.console->printf("%s: %u\n", counters[i].msg_name, unsigned(counters[i].count));
        counters[i].count = 0;
    }
    hal.console->printf("\n");
}

static UAVCAN_sniffer sniffer;

UAVCAN_sniffer::UAVCAN_sniffer() :
    _node_allocator(UAVCAN_NODE_POOL_SIZE, UAVCAN_NODE_POOL_SIZE)
{}

UAVCAN_sniffer::~UAVCAN_sniffer()
{
}

void setup(void)
{
    hal.scheduler->delay(2000);
    hal.console->printf("Starting UAVCAN sniffer\n");
    sniffer.init();
    
}

void loop(void)
{
    sniffer.loop();
    static uint32_t last_print_ms;
    uint32_t now = AP_HAL::millis();
    if (now - last_print_ms >= 1000) {
        last_print_ms = now;
        sniffer.print_stats();
    }

    // auto-reboot for --upload
    if (hal.console->available() > 50) {
        hal.console->printf("rebooting\n");
        while (hal.console->available()) {
            hal.console->read();
        }
        hal.scheduler->reboot(false);
    }
    while (hal.console->available()) {
        hal.console->read();
    }
}

AP_HAL_MAIN();

#else

#include <stdio.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static void loop() { }
static void setup()
{
    printf("Board not currently supported\n");
}

AP_HAL_MAIN();
#endif
