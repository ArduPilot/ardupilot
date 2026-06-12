#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Common/Location.h>
#include "AP_CompanionComputer_config.h"

/*
  NCU (上位机) ↔ FCU (飞控) 串口通信。
  帧格式与指令定义见 AP_CompanionComputer_config.h。
  Rover 通过 receive_companion_computer() / send2_companion_computer() 调度 update() / send_data()。
 */
class AP_CompanionComputer {
public:
    AP_CompanionComputer();

    /* Do not allow copies */
    AP_CompanionComputer(const AP_CompanionComputer &other) = delete;
    AP_CompanionComputer &operator=(const AP_CompanionComputer&) = delete;

    static AP_CompanionComputer *get_singleton() { return _singleton; }

    // 初始化串口（SERIALx_PROTOCOL=50, CC_PORT 指定实例）
    void init();
    // 接收 NCU 数据（由 Rover 50Hz 调度）
    void update();
    // 发送 FCU 状态反馈 0xBB 0x01（由 Rover 10Hz 调度）
    void send_data();

    // ModeVGSolar 每周期更新：控制模式、急停/转弯标志、模式侧故障位
    void update_mode_status(uint8_t control_mode, bool estop, bool turning, uint16_t fault_bits);
    // 离开 VGSOLAR 模式时清零模式侧反馈字段
    void reset_mode_status();

    static const struct AP_Param::GroupInfo var_info[];

private:
    static AP_CompanionComputer *_singleton;

    // Parameters
    AP_Int8 _enable;
    AP_Int8 _port_index;

    AP_HAL::UARTDriver *_uart;

    // NCU → FCU 接收状态机
    enum class RxState {
        WAITING_HEADER1,
        WAITING_HEADER2,
        WAITING_SOURCE,
        WAITING_TYPE,
        WAITING_LENGTH,
        RECEIVING_DATA
    } _rx_state;

    std::array<uint8_t, COMPANION_RECV_TOTAL_LENGTH> _rx_buffer;
    uint8_t _rx_count;
    uint32_t _rx_start_time;
    uint32_t _last_sent_ms;  // send_data 10Hz 限速

    uint8_t _cmd_type;
    uint8_t _data_len;

    bool _estop_active;  // NCU 系统控制指令触发的急停（parse_system_ctrl 实现后生效）

    // 状态反馈中由 ModeVGSolar 提供的字段（与 collect_sensor_faults 合并后写入 send_data）
    uint8_t _fb_control_mode;
    bool _fb_estop;
    bool _fb_turning;
    uint16_t _fb_fault_bits;
    bool _fb_mode_status_valid;

    void process_received_data(uint8_t oneByte);
    void handle_valid_packet();
    // 校验和: byte2..byte(n-2) 累加和低 8 位（帧头不参与）
    uint8_t calculate_checksum(const uint8_t *data, uint8_t len) const;
    bool validate_packet() const;

    // 传感器侧故障位（IMU/GPS/轮速计等）
    uint16_t collect_sensor_faults() const;
    static uint8_t compute_motion_state(int16_t velocity_cms, bool estop, bool turning, uint16_t fault_code);
};

namespace AP {
    AP_CompanionComputer &companioncomputer();
};
