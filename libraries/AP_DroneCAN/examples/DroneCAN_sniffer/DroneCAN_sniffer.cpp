/*
  simple DroneCAN network sniffer as an ArduPilot firmware
 */
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#if HAL_ENABLE_DRONECAN_DRIVERS

#include <AP_HAL/Semaphores.h>

#include <AP_DroneCAN/AP_DroneCAN.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <AP_HAL_Linux/CANSocketIface.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <AP_HAL_SITL/CANSocketIface.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <hal.h>
#include <AP_HAL_ChibiOS/CANIface.h>
#endif

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define DRONECAN_NODE_POOL_SIZE 8192

static uint8_t node_memory_pool[DRONECAN_NODE_POOL_SIZE];

#define debug_dronecan(fmt, args...) do { hal.console->printf(fmt, ##args); } while (0)

class DroneCAN_sniffer {
public:
    DroneCAN_sniffer();
    ~DroneCAN_sniffer();

    void init(void);
    void loop(void);
    void print_stats(void);

private:
    uint8_t driver_index = 0;

    AP_CANManager can_mgr;
};

static struct {
    const char *msg_name;
    uint32_t count;
    uint64_t last_time_us;
    uint32_t avg_period_us;
    uint32_t max_period_us;
    uint32_t min_period_us;
} counters[100];

static void count_msg(const char *name)
{
    for (uint16_t i=0; i<ARRAY_SIZE(counters); i++) {
        if (counters[i].msg_name == name) {
            counters[i].count++;
            uint64_t now = AP_HAL::micros64();
            uint32_t period_us = now - counters[i].last_time_us;
            counters[i].last_time_us = now;
            if (counters[i].avg_period_us == 0) {
                counters[i].avg_period_us = period_us;
            } else {
                counters[i].avg_period_us = (counters[i].avg_period_us * (counters[i].count - 1) + period_us) / counters[i].count;
            }
            if (counters[i].max_period_us < period_us) {
                counters[i].max_period_us = period_us;
            }
            if (counters[i].min_period_us == 0 || counters[i].min_period_us > period_us) {
                counters[i].min_period_us = period_us;
            }
            break;
        }
        if (counters[i].msg_name == nullptr) {
            counters[i].msg_name = name;
            counters[i].count++;
            counters[i].last_time_us = AP_HAL::micros64();
            break;
        }
    }
}

#define MSG_CB(mtype, cbname)                                                  \
    static void cb_ ## cbname(const CanardRxTransfer &transfer, const mtype& msg) { count_msg(#mtype); }

MSG_CB(uavcan_protocol_NodeStatus, NodeStatus)
MSG_CB(uavcan_equipment_gnss_Fix2, Fix2)
MSG_CB(uavcan_equipment_gnss_Auxiliary, Auxiliary)
MSG_CB(uavcan_equipment_ahrs_MagneticFieldStrength, MagneticFieldStrength)
MSG_CB(uavcan_equipment_ahrs_MagneticFieldStrength2, MagneticFieldStrength2);
MSG_CB(uavcan_equipment_air_data_StaticPressure, StaticPressure)
MSG_CB(uavcan_equipment_air_data_StaticTemperature, StaticTemperature)
MSG_CB(uavcan_equipment_power_BatteryInfo, BatteryInfo);
MSG_CB(uavcan_equipment_actuator_ArrayCommand, ArrayCommand)
MSG_CB(uavcan_equipment_esc_RawCommand, RawCommand)
MSG_CB(uavcan_equipment_indication_LightsCommand, LightsCommand);
MSG_CB(com_hex_equipment_flow_Measurement, Measurement);


uavcan_protocol_NodeStatus node_status;
uavcan_protocol_GetNodeInfoResponse node_info;
CanardInterface *_uavcan_iface_mgr;
Canard::Publisher<uavcan_protocol_NodeStatus> *node_status_pub;
Canard::Server<uavcan_protocol_GetNodeInfoRequest> *node_info_srv;

static void cb_GetNodeInfoRequest(const CanardRxTransfer &transfer, const uavcan_protocol_GetNodeInfoRequest& msg)
{
    if (node_info_srv == nullptr) {
        return;
    }
    node_info_srv->respond(transfer, node_info);
}

void DroneCAN_sniffer::init(void)
{
    const_cast <AP_HAL::HAL&> (hal).can[driver_index] = new HAL_CANIface(driver_index);
    
    if (hal.can[driver_index] == nullptr) {
        AP_HAL::panic("Couldn't allocate CANManager, something is very wrong");
    }

    hal.can[driver_index]->init(1000000, AP_HAL::CANIface::NormalMode);

    if (!hal.can[driver_index]->is_initialized()) {
        debug_dronecan("Can not initialised\n");
        return;
    }
    _uavcan_iface_mgr = new CanardInterface{driver_index};

    if (_uavcan_iface_mgr == nullptr) {
        return;
    }

    if (!_uavcan_iface_mgr->add_interface(hal.can[driver_index])) {
        debug_dronecan("Failed to add iface");
        return;
    }

    _uavcan_iface_mgr->init(node_memory_pool, sizeof(node_memory_pool), 9);

    node_status_pub = new Canard::Publisher<uavcan_protocol_NodeStatus>{*_uavcan_iface_mgr};
    if (node_status_pub == nullptr) {
        return;
    }

    node_info_srv = new Canard::Server<uavcan_protocol_GetNodeInfoRequest>{*_uavcan_iface_mgr, *Canard::allocate_static_callback(cb_GetNodeInfoRequest)};
    if (node_info_srv == nullptr) {
        return;
    }

    node_info.name.len = snprintf((char*)node_info.name.data, sizeof(node_info.name.data), "org.ardupilot:%u", driver_index);

    node_info.software_version.major = AP_DRONECAN_SW_VERS_MAJOR;
    node_info.software_version.minor = AP_DRONECAN_SW_VERS_MINOR;

    node_info.hardware_version.major = AP_DRONECAN_HW_VERS_MAJOR;
    node_info.hardware_version.minor = AP_DRONECAN_HW_VERS_MINOR;

#define START_CB(mtype, cbname) Canard::allocate_sub_static_callback(cb_ ## cbname,driver_index)

    START_CB(uavcan_protocol_NodeStatus, NodeStatus);
    START_CB(uavcan_equipment_gnss_Fix2, Fix2);
    START_CB(uavcan_equipment_gnss_Auxiliary, Auxiliary);
    START_CB(uavcan_equipment_ahrs_MagneticFieldStrength, MagneticFieldStrength);
    START_CB(uavcan_equipment_ahrs_MagneticFieldStrength2, MagneticFieldStrength2);
    START_CB(uavcan_equipment_air_data_StaticPressure, StaticPressure);
    START_CB(uavcan_equipment_air_data_StaticTemperature, StaticTemperature);
    START_CB(uavcan_equipment_power_BatteryInfo, BatteryInfo);
    START_CB(uavcan_equipment_actuator_ArrayCommand, ArrayCommand);
    START_CB(uavcan_equipment_esc_RawCommand, RawCommand);
    START_CB(uavcan_equipment_indication_LightsCommand, LightsCommand);
    START_CB(com_hex_equipment_flow_Measurement, Measurement);

    debug_dronecan("DroneCAN: init done\n\r");
}

static void send_node_status()
{
    uavcan_protocol_NodeStatus msg;
    msg.uptime_sec = AP_HAL::millis() / 1000;
    msg.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    msg.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    msg.sub_mode = 0;
    msg.vendor_specific_status_code = 0;
    node_status_pub->broadcast(msg);
}

uint32_t last_status_send = 0;
void DroneCAN_sniffer::loop(void)
{
    if (AP_HAL::millis() - last_status_send > 1000) {
        last_status_send = AP_HAL::millis();
        send_node_status();
    }
    _uavcan_iface_mgr->process(1);
}

void DroneCAN_sniffer::print_stats(void)
{
    hal.console->printf("%lu\n", (unsigned long)AP_HAL::micros());
    for (uint16_t i=0;i<100;i++) {
        if (counters[i].msg_name == nullptr) {
            break;
        }
        hal.console->printf("%s: %lu AVG_US: %lu MAX_US: %lu MIN_US: %lu\n", counters[i].msg_name,
                                                                            (long unsigned)counters[i].count,
                                                                            (long unsigned)counters[i].avg_period_us,
                                                                            (long unsigned)counters[i].max_period_us,
                                                                            (long unsigned)counters[i].min_period_us);
        counters[i].count = 0;
        counters[i].avg_period_us = 0;
        counters[i].max_period_us = 0;
        counters[i].min_period_us = 0;
    }
    hal.console->printf("\n");
}

static DroneCAN_sniffer sniffer;

DroneCAN_sniffer::DroneCAN_sniffer()
{}

DroneCAN_sniffer::~DroneCAN_sniffer()
{
}

void setup(void)
{
    hal.scheduler->delay(2000);
    hal.console->printf("Starting DroneCAN sniffer\n");
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
        hal.console->discard_input();
        hal.scheduler->reboot(false);
    }
    hal.console->discard_input();
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
