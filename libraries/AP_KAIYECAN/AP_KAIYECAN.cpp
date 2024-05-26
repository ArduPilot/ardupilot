#include "AP_KAIYECAN.h"

#if AP_KAIYECAN_ENABLED
#include <stdio.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>    // for MIN,MAX

extern const AP_HAL::HAL& hal;
extern int16_t motors_store[12];
#define AP_KAIYECAN_DEBUG 0

// table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_KAIYECAN::var_info[] = {

    // @Param: NPOLE
    // @DisplayName: Number of motor poles
    // @Description: Sets the number of motor poles to calculate the correct RPM value
    AP_GROUPINFO("NPOLE", 1, AP_KAIYECAN, _num_poles, DEFAULT_NUM_POLES),

    AP_GROUPEND
};

AP_KAIYECAN::AP_KAIYECAN()
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_KAIYECAN must be singleton");
    }
#endif
    _singleton = this;
}

void AP_KAIYECAN::init()
{
    if (_driver != nullptr) {
        // only allow one instance
        return;
    }

    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        if (CANSensor::get_driver_type(i) == AP_CAN::Protocol::KAIYECAN) {
            _driver = new AP_KAIYECAN_Driver();
            return;
        }
    }
}

void AP_KAIYECAN::update()
{
    if (_driver == nullptr) {
        return;
    }
    _driver->update((uint8_t)_num_poles.get());
}

AP_KAIYECAN_Driver::AP_KAIYECAN_Driver() : CANSensor("KAIYECAN")
{
    register_driver(AP_CAN::Protocol::KAIYECAN);
    // start thread for receiving and sending CAN frames. Tests show we use about 640 bytes of stack
    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_KAIYECAN_Driver::loop, void), "kayecan", 2048, AP_HAL::Scheduler::PRIORITY_CAN, 0);
}

// parse inbound frames
void AP_KAIYECAN_Driver::handle_frame(AP_HAL::CANFrame &frame)
{
    if (!frame.isExtended()) {
        return;
    }

    const frame_id_t id { .value = frame.id & AP_HAL::CANFrame::MaskExtID };

#if AP_KAIYECAN_DEBUG
    if (id.object_address != TELEMETRY_OBJ_ADDR) {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"KAIYECAN: rx id:%d, src:%d, dest:%d, len:%d", (int)id.object_address, (int)id.source_id, (int)id.destination_id, (int)frame.dlc);
    }
#endif
    
    // GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"KAIYECAN: rx id:%d, src:%d, dest:%d, len:%d", (int)id.object_address, (int)id.source_id, (int)id.destination_id, (int)frame.dlc);
    
    if (id.destination_id != AUTOPILOT_NODE_ID || id.source_id < ESC_NODE_ID_FIRST) {
        // not for us or invalid id (0 and 1 are invalid)
        return;
    }
       
    // check if frame is valid: directed at autopilot, doesn't come from broadcast and ESC was detected before
//     switch (id.object_address) {
//         case ESC_INFO_OBJ_ADDR:
//             if (frame.dlc == 5 &&
//                 (id.source_id < (ARRAY_SIZE(_output.pwm) + ESC_NODE_ID_FIRST)))
//             {
//                 if (__builtin_popcount(_init.detected_bitmask) >= KAYECAN_MAX_NUM_ESCS) {
//                     // we already have the maximum number of ESCs
//                     return;
//                 }
//                 const uint16_t bitmask = (1UL << (id.source_id - ESC_NODE_ID_FIRST));

//                 if ((bitmask & _init.detected_bitmask) != bitmask) {
//                     _init.detected_bitmask |= bitmask;
//                     GCS_SEND_TEXT(MAV_SEVERITY_INFO,"KAIYECAN: Found ESC id %u mapped to SERVO%u", id.source_id, id.source_id-1);
//                 }
//             }
//         break;

// #if HAL_WITH_ESC_TELEM
//         case TELEMETRY_OBJ_ADDR:
//             if (frame.dlc == 8 &&
//                 (1UL << (id.source_id - ESC_NODE_ID_FIRST) & _init.detected_bitmask))
//             {
//                 const uint8_t idx = id.source_id - ESC_NODE_ID_FIRST;
//                 const uint8_t num_poles = _telemetry.num_poles > 0 ? _telemetry.num_poles : DEFAULT_NUM_POLES;
//                 update_rpm(idx, uint16_t(uint16_t(frame.data[4] << 8 | frame.data[5]) * 60UL * 2 / num_poles));

//                 const TelemetryData t {
//                     .temperature_cdeg = int16_t(frame.data[6] * 100),
//                     .voltage = float(uint16_t(frame.data[0] << 8 | frame.data[1])) * 0.01f,
//                     .current = float(uint16_t(frame.data[2] << 8 | frame.data[3])) * 0.01f,
//                 };
//                 update_telem_data(idx, t,
//                     AP_ESC_Telem_Backend::TelemetryType::CURRENT |
//                     AP_ESC_Telem_Backend::TelemetryType::VOLTAGE |
//                     AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE);
//             }
//             break;
// #endif // HAL_WITH_ESC_TELEM
//     }
}

void AP_KAIYECAN_Driver::update(const uint8_t num_poles)
{
    if (_init.detected_bitmask == 0) {
        // nothing to do...
        return;
    }

#if HAL_WITH_ESC_TELEM
    _telemetry.num_poles = num_poles;
#endif
  
    WITH_SEMAPHORE(_output.sem);
    for (uint8_t i = 0; i < ARRAY_SIZE(_output.pwm); i++) {
        if ((_init.detected_bitmask & (1UL<<i)) == 0 || SRV_Channels::channel_function(i) <= SRV_Channel::Aux_servo_function_t::k_none) {
            _output.pwm[i] = 0;
            continue;
        }

        const SRV_Channel *c = SRV_Channels::srv_channel(i);
        if (c == nullptr) {
            _output.pwm[i] = 0;
            continue;
        }
        _output.pwm[i] = c->get_output_pwm();
    }

    _output.is_new = true;

#if AP_KAIYECAN_USE_EVENTS
    if (_output.thread_ctx != nullptr) {
        // trigger the thread to wake up immediately
        chEvtSignal(_output.thread_ctx, 1);
    }
#endif

#if AP_KAIYECAN_DEBUG
    static uint32_t last_send_ms = 0;
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_send_ms > 1000) {
        last_send_ms = now_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"%u: %u, %u, %u, %u, %u, %u, %u, %u",
        (unsigned)_init.detected_bitmask,
        (unsigned)_output.pwm[0], (unsigned)_output.pwm[1], (unsigned)_output.pwm[2], (unsigned)_output.pwm[3],
        (unsigned)_output.pwm[4], (unsigned)_output.pwm[5], (unsigned)_output.pwm[6], (unsigned)_output.pwm[7]);
    }
#endif
}

void AP_KAIYECAN_Driver::loop()
{
    uint16_t pwm[ARRAY_SIZE(_output.pwm)] {};

#if AP_KAIYECAN_USE_EVENTS
    _output.thread_ctx = chThdGetSelfX();
#endif

    uint8_t broadcast_esc_info_boot_spam_count = 3;
    uint32_t broadcast_esc_info_next_interval_ms = 100; // spam a few at boot at this rate

    while (true) {
#if AP_KAIYECAN_USE_EVENTS
        // sleep until we get new data, but also wake up at 400Hz to send the old data again
        chEvtWaitAnyTimeout(ALL_EVENTS, chTimeUS2I(2500));
 #else
        hal.scheduler->delay_microseconds(2500); // 400Hz
#endif

        const uint32_t now_ms = AP_HAL::millis();
        uint8_t testbuff1[4] ={0x20,0Xe8,0X31, 0XC0};
            
        // static bool flagbuff = false;
        // if(!flagbuff)
        // {
        //     for(uint8_t i = 0;i<253;i++)
        //     {
        //         send_packet(ESC_INFO_OBJ_ADDR, BROADCAST_NODE_ID, 100,testbuff,4); 
        //     }
        //     flagbuff = true;
        // }else
        {  
            motors_store[0]*=10;
            testbuff1[1] = 0x00ff&motors_store[0];
            testbuff1[2] = 0x00ff&motors_store[0]>>8;
            send_packet(ESC_INFO_OBJ_ADDR, BROADCAST_NODE_ID, 100,testbuff1,4);

            testbuff1[0] = 0x1e;
            motors_store[1]*=10;
            testbuff1[1] = 0x00ff&motors_store[0];
            testbuff1[2] = 0x00ff&motors_store[0]>>8;
            send_packet(ESC_INFO_OBJ_ADDR, BROADCAST_NODE_ID, 100,testbuff1,4);

            testbuff1[0] = 0x1f;
            motors_store[2]*=10;
            testbuff1[1] = 0x00ff&motors_store[0];
            testbuff1[2] = 0x00ff&motors_store[0]>>8;
            send_packet(ESC_INFO_OBJ_ADDR, BROADCAST_NODE_ID, 100,testbuff1,4);

            testbuff1[0] = 0x21;
            motors_store[4]*=10;
            testbuff1[1] = 0x00ff&motors_store[0];
            testbuff1[2] = 0x00ff&motors_store[0]>>8;
            send_packet(ESC_INFO_OBJ_ADDR, BROADCAST_NODE_ID, 100,testbuff1,4);
        }

        // This should run at 400Hz
        {
            WITH_SEMAPHORE(_output.sem);
            if (_output.is_new) {
                _output.last_new_ms = now_ms;
                _output.is_new = false;
                memcpy(&pwm, &_output.pwm, sizeof(pwm));

            } else if (_output.last_new_ms && now_ms - _output.last_new_ms > 1000) {
                // if we haven't gotten any PWM updates for a bit, zero it
                // out so we don't just keep sending the same values forever
                memset(&pwm, 0, sizeof(pwm));
                _output.last_new_ms = 0;
            }
        }

        // for (uint8_t i=0; i<ARRAY_SIZE(_output.pwm); i++) {
        //     // if ((_init.detected_bitmask & (1UL<<i)) != 0) {
        //         send_packet_uint16(SET_PWM_OBJ_ADDR, (i + ESC_NODE_ID_FIRST), 1, pwm[i]);
        //         GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "send_packet_uint16");
        //     // }
        // }
                // convert output to PWM and send to each motor
        // for (uint8_t i = 0; i < 12; i++) 
        // {
        //     GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "PWM for channel %u: %u", i,motors_store[i]);
        // }
            

#if HAL_WITH_ESC_TELEM
        // broadcast as request-telemetry msg to everyone
        if (_init.detected_bitmask != 0 && now_ms - _telemetry.timer_ms >= TELEMETRY_INTERVAL_MS) {
            if (send_packet(TELEMETRY_OBJ_ADDR, BROADCAST_NODE_ID, 10)) {
                _telemetry.timer_ms = now_ms;
            }
        }
#endif // HAL_WITH_ESC_TELEM

        if ((_init.detected_bitmask == 0 || broadcast_esc_info_boot_spam_count > 0) && (now_ms - _init.detected_bitmask_ms >= broadcast_esc_info_next_interval_ms)) {
            // broadcast an "anyone there?" quick at boot but then 1Hz forever until we see at least 1 esc respond
            if (broadcast_esc_info_boot_spam_count > 0) {
                broadcast_esc_info_boot_spam_count--;
            } else {
                broadcast_esc_info_next_interval_ms = 1000;
            }
            // uint8_t getmotor[1] ={0XC0};//0x00D1A081
  
            // if (send_packet(ID3, ID2, 100,getmotor,1)) {
            //     _init.detected_bitmask_ms = now_ms;
            // }
        }


    } // while true
}

bool AP_KAIYECAN_Driver::send_packet_uint16(const uint8_t address, const uint8_t dest_id, const uint32_t timeout_ms, const uint16_t data)
{
    const uint16_t data_be16 = htobe16(data);
    return send_packet(address, dest_id, timeout_ms, (uint8_t*)&data_be16, 2);
}

bool AP_KAIYECAN_Driver::send_packet(const uint8_t address, const uint8_t dest_id, const uint32_t timeout_ms, const uint8_t *data, const uint8_t data_len)
{
    // broadcast telemetry request frame
    const frame_id_t id {
        {
            .object_address = address,
            .destination_id = dest_id,
            .source_id = AUTOPILOT_NODE_ID,
            .priority = 0,
            .unused = 0
        }
    };

    AP_HAL::CANFrame frame = AP_HAL::CANFrame((id.value | AP_HAL::CANFrame::FlagEFF), data, data_len, false);

    const uint64_t timeout_us = uint64_t(timeout_ms) * 1000UL;
    return write_frame(frame, timeout_us);
}

bool AP_KAIYECAN_Driver::send_packet32(const uint32_t address,const uint32_t timeout_ms, const uint8_t *data, const uint8_t data_len)
{
    // broadcast telemetry request frame
    const frame_id_t id {
        {
            .object_address = static_cast<uint8_t>(0x000000ff & address),
            .destination_id = static_cast<uint8_t>(0x000000ff&(address>>8)),
            .source_id = static_cast<uint8_t>(0x000000ff&(address>>16)),
            .priority = 0,
            .unused = 0
        }
    };

    AP_HAL::CANFrame frame = AP_HAL::CANFrame((id.value | AP_HAL::CANFrame::FlagEFF), data, data_len, false);

    const uint64_t timeout_us = uint64_t(timeout_ms) * 1000UL;
    return write_frame(frame, timeout_us);
}

bool AP_KAIYECAN_Driver::set_esc_hz(void)
{
    uint8_t esc_hz[3] = {0x00,0x00,0xc0};
    esc_hz[0] = 0xff&ESC_HZ_SET_VALUE;
    esc_hz[1] = 0xff&(ESC_HZ_SET_VALUE>>8);
    send_packet32(SET_ESC_FRQ_BOARDCAST,100,esc_hz,3);

    return send_packet32(SET_ESC_FRQ_BOARDCAST,100,esc_hz,3);
}

// singleton instance
AP_KAIYECAN *AP_KAIYECAN::_singleton;

namespace AP {
AP_KAIYECAN *kayecan()
{
    return AP_KAIYECAN::get_singleton();
}
};

#endif // AP_KAIYECAN_ENABLED

