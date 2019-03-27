/*
  simple KDECAN network sniffer as an ArduPilot firmware
 */
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#if (CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS || CONFIG_HAL_BOARD == HAL_BOARD_LINUX) && HAL_WITH_UAVCAN

#include <AP_HAL/CAN.h>
#include <AP_HAL/Semaphores.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <AP_HAL_Linux/CAN.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <AP_HAL_ChibiOS/CAN.h>
#endif

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>

#include <cinttypes>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define debug_can(fmt, args...) do { hal.console->printf(fmt, ##args); } while (0)

#define NUM_ESCS 8

class KDECAN_sniffer {
public:
    KDECAN_sniffer() {
        for (uint8_t i = 0; i < NUM_ESCS; i++) {
            _esc_info[i].mcu_id = 0xA5961824E7BD3C00 | i;
        }               
    }

    void init(void);
    void loop(void);
    void print_stats(void);
    void send_enumeration(uint8_t num);

private:
    uint8_t _driver_index = 0;
    uint8_t _interface = 0;
    uavcan::ICanDriver* _can_driver;
    uint8_t _mask_received_pwm = 0;

    struct esc_info {
        uint8_t node_id;
        uint64_t mcu_id;
        uint64_t enum_timeout;

        esc_info() : node_id(1), mcu_id(0), enum_timeout(0) {}
    } _esc_info[NUM_ESCS];
    
    uint8_t _max_node_id = 0;

    static const uint8_t BROADCAST_NODE_ID = 1;

    static const uint8_t ESC_INFO_OBJ_ADDR = 0;
    static const uint8_t SET_PWM_OBJ_ADDR = 1;
    static const uint8_t VOLTAGE_OBJ_ADDR = 2;
    static const uint8_t CURRENT_OBJ_ADDR = 3;
    static const uint8_t RPM_OBJ_ADDR = 4;
    static const uint8_t TEMPERATURE_OBJ_ADDR = 5;
    static const uint8_t GET_PWM_INPUT_OBJ_ADDR = 6;
    static const uint8_t GET_PWM_OUTPUT_OBJ_ADDR = 7;
    static const uint8_t MCU_ID_OBJ_ADDR = 8;
    static const uint8_t UPDATE_NODE_ID_OBJ_ADDR = 9;
    static const uint8_t START_ENUM_OBJ_ADDR = 10;
    static const uint8_t TELEMETRY_OBJ_ADDR = 11;
};

static struct {
    uint32_t frame_id;
    uint32_t count;
} counters[100];

static void count_msg(uint32_t frame_id)
{
    for (uint16_t i=0; i<ARRAY_SIZE(counters); i++) {
        if (counters[i].frame_id == frame_id) {
            counters[i].count++;
            break;
        }
        if (counters[i].frame_id == 0) {
            counters[i].frame_id = frame_id;
            counters[i].count++;
            break;
        }
    }
}

void KDECAN_sniffer::init(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    AP_HAL::CANManager* can_mgr = new ChibiOS::CANManager;
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    AP_HAL::CANManager* can_mgr = new Linux::CANManager;
#endif
    
    if (can_mgr == nullptr) {
        AP_HAL::panic("Couldn't allocate CANManager, something is very wrong");
    }

    const_cast <AP_HAL::HAL&> (hal).can_mgr[_driver_index] = can_mgr;
    can_mgr->begin(1000000, _interface);
    can_mgr->initialized(true);

    if (!can_mgr->is_initialized()) {
        debug_can("Can not initialised\n");
        return;
    }

    _can_driver = can_mgr->get_driver();

    if (_can_driver == nullptr) {
        debug_can("KDECAN: no CAN driver\n\r");
        return;
    }

    debug_can("KDECAN: init done\n\r");
}

void KDECAN_sniffer::loop(void)
{
    if (_can_driver == nullptr) {
        return;
    }
    
    uavcan::CanFrame empty_frame { (0 | uavcan::CanFrame::FlagEFF), nullptr, 0 };
    const uavcan::CanFrame* select_frames[uavcan::MaxCanIfaces] { &empty_frame };
    uavcan::MonotonicTime timeout = uavcan::MonotonicTime::fromMSec(AP_HAL::millis() + 1);

    uavcan::CanSelectMasks inout_mask;
    inout_mask.read = 1 << _interface;

    uavcan::CanSelectMasks in_mask = inout_mask;
    _can_driver->select(inout_mask, select_frames, timeout);

    if (in_mask.read & inout_mask.read) {
        uavcan::CanFrame frame;
        uavcan::MonotonicTime time;
        uavcan::UtcTime utc_time;
        uavcan::CanIOFlags flags {};

        int16_t res = _can_driver->getIface(_interface)->receive(frame, time, utc_time, flags);
        
        if (res == 1) {
            uint32_t id = frame.id & uavcan::CanFrame::MaskExtID;
            uint8_t object_address = id & 0xFF;
            uint8_t esc_num = uint8_t((id >> 8) & 0xFF);

            count_msg(id);

            uint8_t i = 0;
            uint8_t n = NUM_ESCS;

            if (esc_num != BROADCAST_NODE_ID) {
                for (; i < NUM_ESCS; i++) {
                    if (object_address == UPDATE_NODE_ID_OBJ_ADDR) {
                        if (_esc_info[i].mcu_id == be64toh(*((be64_t*) &(frame.data[0])))) {
                            n = i + 1;
                            break;                            
                        }
                    } else if (_esc_info[i].node_id == esc_num) {
                        n = i + 1;
                        break;
                    }
                }
            }

            while (i < n) {
                uavcan::CanFrame res_frame;

                switch (object_address) {
                    case ESC_INFO_OBJ_ADDR: {
                        uint8_t info[5] { 1, 2, 3, 4, 0 };

                        res_frame.dlc = 5;
                        memcpy(res_frame.data, info, 5);

                        break;
                    }
                    case SET_PWM_OBJ_ADDR: {
                        if ((1 << (esc_num - 2) & _mask_received_pwm) && _mask_received_pwm != ((1 << _max_node_id) - 1)) {
                            count_msg(0xFFFFFFF0);
                            _mask_received_pwm = 0;
                        }

                        _mask_received_pwm |= 1 << (esc_num - 2);

                        if (_mask_received_pwm == ((1 << _max_node_id) - 1)) {
                            count_msg(0xFFFFFFFF);
                            _mask_received_pwm = 0;
                        }

                        res_frame.dlc = 0;

                        break;
                    }
                    case UPDATE_NODE_ID_OBJ_ADDR: {
                        if (_esc_info[i].enum_timeout != 0 && _esc_info[i].enum_timeout >= AP_HAL::micros64()) {
                            _esc_info[i].node_id = esc_num;
                            _max_node_id = MAX(_max_node_id, esc_num - 2 + 1);
                            hal.console->printf("Set node ID %d for ESC %d\n", esc_num, i);
                        }

                        _esc_info[i].enum_timeout = 0;

                        res_frame.dlc = 1;
                        memcpy(res_frame.data, &(_esc_info[i].node_id), 1);

                        break;
                    }
                    case START_ENUM_OBJ_ADDR: {
                        _esc_info[i].enum_timeout = AP_HAL::micros64() + be16toh(*((be16_t*) &(frame.data[0]))) * 1000;
                        hal.console->printf("Starting enumeration for ESC %d, timeout %" PRIu64 "\n", i, _esc_info[i].enum_timeout);
                        i++;
                        continue;
                    }
                    case TELEMETRY_OBJ_ADDR: {
                        uint8_t data[7] {};
                        *((be16_t*) &data[0]) = htobe16(get_random16());
                        *((be16_t*) &data[2]) = htobe16(get_random16());
                        *((be16_t*) &data[4]) = htobe16(get_random16());
                        data[6] = uint8_t(float(rand()) / RAND_MAX * 40.0f + 15);

                        res_frame.dlc = 7;
                        memcpy(res_frame.data, data, 7);
                        break;
                    }
                    case VOLTAGE_OBJ_ADDR:
                    case CURRENT_OBJ_ADDR:
                    case RPM_OBJ_ADDR:
                    case TEMPERATURE_OBJ_ADDR:
                    case GET_PWM_INPUT_OBJ_ADDR:
                    case GET_PWM_OUTPUT_OBJ_ADDR:
                    case MCU_ID_OBJ_ADDR:
                    default:
                        // discard frame
                        return;
                }

                res_frame.id = (_esc_info[i].node_id << 16) | object_address | uavcan::CanFrame::FlagEFF;
                timeout = uavcan::MonotonicTime::fromUSec(AP_HAL::millis() + 500);
                int16_t res2 = _can_driver->getIface(_interface)->send(res_frame, timeout, 0);

                if (res2 == 1) {
                    i++;
                }
            }
        }
    }
}

void KDECAN_sniffer::print_stats(void)
{
    hal.console->printf("%lu\n", AP_HAL::micros());
    for (uint16_t i=0;i<100;i++) {
        if (counters[i].frame_id == 0) {
            break;
        }
        hal.console->printf("0x%08" PRIX32 ": %" PRIu32 "\n", counters[i].frame_id, counters[i].count);
        counters[i].count = 0;
    }
    hal.console->printf("\n");
}

void KDECAN_sniffer::send_enumeration(uint8_t num)
{
    if (_esc_info[num].enum_timeout == 0 || AP_HAL::micros64() > _esc_info[num].enum_timeout) {
        _esc_info[num].enum_timeout = 0;
        hal.console->printf("Not running enumeration for ESC %d\n", num);
        return;
    }

    while (true) {
        uint8_t mcu[8] {};
        *((be64_t*) mcu) = htobe64(_esc_info[num].mcu_id);
        uavcan::CanFrame res_frame { (_esc_info[num].node_id << 16) | START_ENUM_OBJ_ADDR | uavcan::CanFrame::FlagEFF,
                                     mcu,
                                     8 };
        uavcan::MonotonicTime timeout = uavcan::MonotonicTime::fromMSec(AP_HAL::millis() + 1);

        int16_t res = _can_driver->getIface(_interface)->send(res_frame, timeout, 0);

        if (res == 1) {
            return;
        }
    }
}

static KDECAN_sniffer sniffer;

void setup(void)
{
    hal.scheduler->delay(2000);
    hal.console->printf("Starting KDECAN sniffer\n");
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

    if (hal.console->available() >= 3) {
        char c = hal.console->read();

        if (c == 'e') {
            c = hal.console->read();

            if (c == ' ') {
                c = hal.console->read();

                if (c >= '0' && c < '9') {
                    uint8_t num = c - '0';
                    sniffer.send_enumeration(num);
                }
            }
        } else if (c == 'r') {
            hal.console->printf("rebooting\n");
            hal.scheduler->reboot(false);
        }
    }

    // auto-reboot for --upload
    if (hal.console->available() > 50) {
        hal.console->printf("rebooting\n");
        hal.scheduler->reboot(false);
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
