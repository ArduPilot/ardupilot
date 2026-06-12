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

    // ModeVGSolar 导航 ACK（0xBB 0x02，cmd_type=NCU_CMD_POSITION）
    void send_position_ack(uint8_t status);
    // ModeVGSolar 导航期间更新；send_nav=true 时本周期发送 0xBB 0x04
    void set_nav_status(const NavStatusData &data, bool send_nav);
    void send_nav_data();

    // ModeVGSolar 每周期更新：控制模式、急停/转弯标志、模式侧故障位
    void update_mode_status(uint8_t control_mode, bool estop, bool turning, uint16_t fault_bits);
    // 离开 VGSOLAR 模式时清零模式侧反馈字段
    void reset_mode_status();

    // NCU 指令缓存（供 ModeVGSolar 读取）
    const SpeedCtrlData& get_latest_speed_ctrl() const { return _latest_speed_ctrl; }
    const TurnData& get_latest_turn() const { return _latest_turn; }
    const PositionData& get_latest_position() const { return _latest_position; }
    const SystemCtrlData& get_latest_system_ctrl() const { return _latest_system_ctrl; }

    bool is_new_speed_ctrl() const { return _new_cmd_flags & (1<<0); }
    bool is_new_turn() const { return _new_cmd_flags & (1<<1); }
    bool is_new_position() const { return _new_cmd_flags & (1<<2); }
    bool is_new_system_ctrl() const { return _new_cmd_flags & (1<<3); }

    void clear_new_speed_flag() { _new_cmd_flags &= ~(1<<0); }
    void clear_new_turn_flag() { _new_cmd_flags &= ~(1<<1); }
    void clear_new_position_flag() { _new_cmd_flags &= ~(1<<2); }
    void clear_new_system_flag() { _new_cmd_flags &= ~(1<<3); }

    bool is_estop_active() const { return _estop_active; }

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

    // NCU 指令缓存；bit0=SPEED bit1=TURN bit2=POSITION bit3=SYSTEM
    SpeedCtrlData   _latest_speed_ctrl;
    TurnData        _latest_turn;
    PositionData    _latest_position;
    SystemCtrlData  _latest_system_ctrl;
    uint8_t _new_cmd_flags;
    bool _estop_active;

    // 状态反馈中由 ModeVGSolar 提供的字段（与 collect_sensor_faults 合并后写入 send_data）
    uint8_t _fb_control_mode;
    bool _fb_estop;
    bool _fb_turning;
    uint16_t _fb_fault_bits;
    bool _fb_mode_status_valid;

    NavStatusData _nav_status;
    bool _nav_status_send;

    void process_received_data(uint8_t oneByte);
    void parse_speed_ctrl();
    void parse_turn();
    void parse_param_write();
    void parse_param_read();
    void parse_system_ctrl();
    void parse_position();

    // 校验和: byte2..byte(n-2) 累加和低 8 位（帧头不参与）
    uint8_t calculate_checksum(const uint8_t *data, uint8_t len) const;
    bool validate_packet() const;
    void send_response(uint8_t cmd_type, uint8_t status);
    void send_param_feedback(const ParamFeedbackData &data);

    // 滚刷参数读写，待 AP_Brush 接入后实现
    bool write_runtime_param(uint16_t param_index, uint8_t param_type, uint32_t param_value,
                             ParamFeedbackData &feedback_out);
    bool read_runtime_param(uint16_t param_index, ParamFeedbackData &feedback_out);

    uint16_t collect_sensor_faults() const;
    static uint8_t compute_motion_state(int16_t velocity_cms, bool estop, bool turning, uint16_t fault_code);
};

namespace AP {
    AP_CompanionComputer &companioncomputer();
};
