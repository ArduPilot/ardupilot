#pragma once

#include <AP_HAL/AP_HAL.h>
#include <array>

/*
 * 通用帧格式 (所有收/发数据包均遵循):
 *
 * Data Bit             Definition          Description
 * ------------------------------------------------
 * byte 0               FRAME_HEADER1       0xA5
 * byte 1               FRAME_HEADER2       0x5A
 * byte 2               command source      0xAA: NCU(上位机)  0xBB: FCU(飞控)
 * byte 3               command content     指令/反馈类型 (见 NCU_CMD_* / FCU_FB_*)
 * byte 4               DATA_LENGTH         数据体长度 (byte5 起连续 DATA_LENGTH 字节)
 * byte 5 .. 4+N        DATA                见各指令定义
 * byte 5+N             Checksum            校验和: byte2..byte(4+N) 累加和的低 8 位 (帧头不参与)
 * byte 6+N             end sign            0xFF
 *
 * 整帧长度 = DATA_LENGTH + 7  (FRAME_OVERHEAD)
 * 单帧接收超时: PACKET_TIMEOUT_MS (200ms)
 *
 * =============================================================================
 * ************ 接收通信协议 (NCU → FCU) *********************************
 * =============================================================================
 *
 * --- 公共帧头 (byte 0~4，以下各指令相同) ---
 * byte 0               FRAME_HEADER1       0xA5
 * byte 1               FRAME_HEADER2       0x5A
 * byte 2               command source      0xAA
 * byte 3               command content     见下表
 * byte 4               DATA_LENGTH         见下表
 *
 * --- 0x01 速度控制 (NCU_CMD_SPEED_CTRL, DATA_LENGTH = 0x05) ---
 * byte 5               control_mode        0x01: 航向角模式 (SPEED_MODE_YAW)
 *                                          0x02: 偏航速率模式 (SPEED_MODE_YAWRATE)
 * byte 6               velocity_L          线速度 int16 低 8 位, 单位 cm/s
 * byte 7               velocity_H          线速度 int16 高 8 位
 * byte 8               yaw_data_L          航向/偏航数据 int16 低 8 位, 单位 0.01° 或 0.01°/s
 * byte 9               yaw_data_H          航向/偏航数据 int16 高 8 位
 * byte 10              Checksum
 * byte 11              end sign            0xFF
 *
 * --- 0x02 转弯序列 (NCU_CMD_TURN, DATA_LENGTH = 0x06) ---
 * byte 5               turn_mode           0x01: 原地转弯 (TURN_MODE_SPOT)
 *                                          0x02: 行进中转弯 (TURN_MODE_MOVING)
 * byte 6               direction           0x01: 左转 (TURN_DIR_LEFT)
 *                                          0x02: 右转 (TURN_DIR_RIGHT)
 * byte 7               target_angle_L      目标角度 uint16 低 8 位, 单位 0.01°
 * byte 8               target_angle_H      目标角度 uint16 高 8 位
 * byte 9               angular_vel_L       角速度 uint16 低 8 位, 单位 0.01°/s
 * byte 10              angular_vel_H       角速度 uint16 高 8 位
 * byte 11              Checksum
 * byte 12              end sign            0xFF
 *
 * --- 0x03 参数写入 (NCU_CMD_PARAM_WRITE, DATA_LENGTH = 0x07) ---
 * byte 5               param_index_L       参数索引 uint16 低 8 位
 * byte 6               param_index_H       参数索引 uint16 高 8 位
 * byte 7               param_type          0x01:int32  0x02:float32  0x03:uint32
 * byte 8               param_value8        参数值 uint32 低 8 位
 * byte 9               param_value16       参数值 uint32 8~15 位
 * byte 10              param_value24       参数值 uint32 16~23 位
 * byte 11              param_value32       参数值 uint32 高 8 位
 * byte 12              Checksum
 * byte 13              end sign            0xFF
 *
 * --- 0x04 参数读取 (NCU_CMD_PARAM_READ, DATA_LENGTH = 0x02) ---
 * byte 5               param_index_L       参数索引 uint16 低 8 位
 * byte 6               param_index_H       参数索引 uint16 高 8 位
 * byte 7               Checksum
 * byte 8               end sign            0xFF
 *
 * --- 0x05 系统控制 (NCU_CMD_SYSTEM_CTRL, DATA_LENGTH = 0x01) ---
 * byte 5               command             0x01:急停  0x02:解除急停  0x03:重启  0x04:关机
 * byte 6               Checksum
 * byte 7               end sign            0xFF
 *
 * --- 0x06 位置导航 (NCU_CMD_POSITION, DATA_LENGTH = 0x0F) ---
 * byte 5               nav_mode            0x01:GPS  0x02:NED  0x03:Body  0x04:取消导航
 * byte 6               target_x8           目标 X int32 低 8 位
 * byte 7               target_x16          目标 X int32 8~15 位
 * byte 8               target_x24          目标 X int32 16~23 位
 * byte 9               target_x32          目标 X int32 高 8 位
 *                        (GPS模式: 经度×1e7; NED/Body模式: 北向/前向 cm)
 * byte 10              target_y8           目标 Y int32 低 8 位
 * byte 11              target_y16          目标 Y int32 8~15 位
 * byte 12              target_y24          目标 Y int32 16~23 位
 * byte 13              target_y32          目标 Y int32 高 8 位
 *                        (GPS模式: 纬度×1e7; NED/Body模式: 东向/右向 cm)
 * byte 14              arrival_yaw_L       到达航向 int16 低 8 位, 0.01°; 0xFFFF=不指定
 * byte 15              arrival_yaw_H       到达航向 int16 高 8 位
 * byte 16              arrival_radius_L    到达半径 uint16 低 8 位, 单位 cm
 * byte 17              arrival_radius_H    到达半径 uint16 高 8 位
 * byte 18              cruise_speed_L      巡航速度 int16 低 8 位, 单位 cm/s
 * byte 19              cruise_speed_H      巡航速度 int16 高 8 位
 * byte 20              Checksum
 * byte 21              end sign            0xFF
 *
 * *********************************************/
 /*
 * =============================================================================
 * ************ 发送通信协议 (FCU → NCU) *********************************
 * =============================================================================
 *
 * --- 公共帧头 (byte 0~4) ---
 * byte 0               FRAME_HEADER1       0xA5
 * byte 1               FRAME_HEADER2       0x5A
 * byte 2               command source      0xBB
 * byte 3               command content     见下表
 * byte 4               DATA_LENGTH         见下表
 *
 * --- 0x01 状态反馈 (FCU_FB_STATUS, 10Hz, DATA_LENGTH = 0x1A) ---
 * byte 5               battery_percent     电池电量 0~100 %
 * byte 6               longitude8          经度 int32 低 8 位 (×1e7)
 * byte 7               longitude16         经度 int32 8~15 位
 * byte 8               longitude24         经度 int32 16~23 位
 * byte 9               longitude32         经度 int32 高 8 位
 * byte 10              latitude8           纬度 int32 低 8 位 (×1e7)
 * byte 11              latitude16          纬度 int32 8~15 位
 * byte 12              latitude24          纬度 int32 16~23 位
 * byte 13              latitude32          纬度 int32 高 8 位
 * byte 14              heading_L           航向角 uint16 低 8 位, 0.01° (0~36000)
 * byte 15              heading_H           航向角 uint16 高 8 位
 * byte 16              velocity_L          地速 int16 低 8 位, cm/s
 * byte 17              velocity_H          地速 int16 高 8 位
 * byte 18              left_track_vel_L    左履带速度 int16 低 8 位, cm/s (WENC)
 * byte 19              left_track_vel_H    左履带速度 int16 高 8 位
 * byte 20              right_track_vel_L   右履带速度 int16 低 8 位, cm/s (WENC2)
 * byte 21              right_track_vel_H   右履带速度 int16 高 8 位
 * byte 22              roll_L              横滚 int16 低 8 位, 0.01°
 * byte 23              roll_H              横滚 int16 高 8 位
 * byte 24              pitch_L             俯仰 int16 低 8 位, 0.01°
 * byte 25              pitch_H             俯仰 int16 高 8 位
 * byte 26              control_mode        控制模式 (ControlMode 枚举)
 * byte 27              motion_state        运动状态 (MotionState 枚举)
 * byte 28              fault_code_L        故障码 uint16 低 8 位 (FaultBits 位标志)
 * byte 29              fault_code_H        故障码 uint16 高 8 位
 * byte 30              gps_status          GPS 定位状态
 * byte 31              Checksum
 * byte 32              end sign            0xFF
 *
 * --- 0x02 指令应答 (FCU_FB_CMD_ACK, DATA_LENGTH = 0x02, 整帧 0x09 字节) ---
 * byte 5               cmd_type            对应的 NCU 指令类型
 * byte 6               status              0x01:成功  0x02:失败
 * byte 7               Checksum
 * byte 8               end sign            0xFF
 *
 * --- 0x03 参数反馈 (FCU_FB_PARAM, DATA_LENGTH = 0x07) ---
 * byte 5               param_index_L       参数索引 uint16 低 8 位
 * byte 6               param_index_H       参数索引 uint16 高 8 位
 * byte 7               param_type          参数类型
 * byte 8               param_value8        参数值 uint32 低 8 位
 * byte 9               param_value16       参数值 uint32 8~15 位
 * byte 10              param_value24       参数值 uint32 16~23 位
 * byte 11              param_value32       参数值 uint32 高 8 位
 * byte 12              Checksum
 * byte 13              end sign            0xFF
 *
 * --- 0x04 导航状态 (FCU_FB_NAV_STATUS, DATA_LENGTH = 0x08) ---
 * byte 5               nav_state           0x00:空闲  0x01:导航中  0x02:已到达  0x03:失败  0x04:已取消
 * byte 6               coord_mode          坐标模式 (同 NCU 导航模式)
 * byte 7               distance_to_target8 距目标 uint32 低 8 位, cm
 * byte 8               distance_to_target16
 * byte 9               distance_to_target24
 * byte 10              distance_to_target32
 * byte 11              heading_error_L     航向误差 int16 低 8 位, 0.01°
 * byte 12              heading_error_H     航向误差 int16 高 8 位
 * byte 13              Checksum
 * byte 14              end sign            0xFF
 *
 * 控制模式 control_mode (ControlMode):
 *   0x00 STANDBY  0x01 YAW  0x02 YAWRATE  0x03 TURN  0x04 NAV_GPS  0x05 NAV_BODY
 *
 * 运动状态 motion_state (MotionState):
 *   0x00 STOPPED  0x01 FORWARD  0x02 BACKWARD  0x03 TURNING  0x04 ESTOP  0x05 FAULT
 *
 *
 * *********************************************
 *
 * =============================================================================
 */

// 协议常量
constexpr uint8_t COMPANION_FRAME_HEADER1     = 0xA5;
constexpr uint8_t COMPANION_FRAME_HEADER2     = 0x5A;
constexpr uint8_t COMPANION_CMD_SOURCE_NCU    = 0xAA;
constexpr uint8_t COMPANION_CMD_SOURCE_FC     = 0xBB;
constexpr uint8_t COMPANION_END_SIGN          = 0xFF;

constexpr uint8_t FRAME_OVERHEAD = 7;

// NCU → FCU 指令类型
constexpr uint8_t NCU_CMD_SPEED_CTRL   = 0x01;
constexpr uint8_t NCU_CMD_TURN         = 0x02;
constexpr uint8_t NCU_CMD_PARAM_WRITE  = 0x03;
constexpr uint8_t NCU_CMD_PARAM_READ   = 0x04;
constexpr uint8_t NCU_CMD_SYSTEM_CTRL  = 0x05;
constexpr uint8_t NCU_CMD_POSITION     = 0x06;

// FCU → NCU 反馈类型
constexpr uint8_t FCU_FB_STATUS        = 0x01;
constexpr uint8_t FCU_FB_CMD_ACK       = 0x02;
constexpr uint8_t FCU_FB_PARAM         = 0x03;
constexpr uint8_t FCU_FB_NAV_STATUS    = 0x04;

// 速度控制模式
constexpr uint8_t SPEED_MODE_YAW        = 0x01;
constexpr uint8_t SPEED_MODE_YAWRATE    = 0x02;

// 转弯常量
constexpr uint8_t TURN_MODE_SPOT        = 0x01;
constexpr uint8_t TURN_MODE_MOVING      = 0x02;
constexpr uint8_t TURN_DIR_LEFT         = 0x01;
constexpr uint8_t TURN_DIR_RIGHT        = 0x02;

// 参数类型
constexpr uint8_t PARAM_TYPE_INT32      = 0x01;
constexpr uint8_t PARAM_TYPE_FLOAT32    = 0x02;
constexpr uint8_t PARAM_TYPE_UINT32     = 0x03;

// 系统控制命令
constexpr uint8_t SYS_CMD_ESTOP         = 0x01;
constexpr uint8_t SYS_CMD_ESTOP_CLEAR   = 0x02;
constexpr uint8_t SYS_CMD_REBOOT        = 0x03;
constexpr uint8_t SYS_CMD_SHUTDOWN      = 0x04;

// 导航模式
constexpr uint8_t NAV_MODE_GPS          = 0x01;
constexpr uint8_t NAV_MODE_NED          = 0x02;
constexpr uint8_t NAV_MODE_BODY         = 0x03;
constexpr uint8_t NAV_MODE_CANCEL       = 0x04;

constexpr uint8_t NAV_STATE_ACTIVE      = 0x01;
constexpr uint8_t NAV_STATE_ARRIVED     = 0x02;
constexpr uint8_t NAV_STATE_FAILED      = 0x03;
constexpr uint8_t NAV_STATE_CANCELLED   = 0x04;

constexpr uint16_t NAV_YAW_UNSPECIFIED  = 0xFFFF;

// 反馈状态码
constexpr uint8_t CMD_ACK_SUCCESS       = 0x01;
constexpr uint8_t CMD_ACK_FAILED        = 0x02;

// 运动状态与控制模式
enum class MotionState : uint8_t {
    STOPPED    = 0x00,
    FORWARD    = 0x01,
    BACKWARD   = 0x02,
    TURNING    = 0x03,
    ESTOP      = 0x04,
    FAULT      = 0x05
};

enum class ControlMode : uint8_t {
    STANDBY   = 0x00,
    YAW       = 0x01,
    YAWRATE   = 0x02,
    TURN      = 0x03,
    NAV_GPS   = 0x04,
    NAV_BODY  = 0x05
};

// 故障码位定义
enum FaultBits : uint16_t {
    FAULT_NONE           = 0,
    FAULT_LEFT_MOTOR     = 1 << 0,
    FAULT_RIGHT_MOTOR    = 1 << 1,
    FAULT_FRONT_BRUSH    = 1 << 2,
    FAULT_REAR_BRUSH     = 1 << 3,
    FAULT_SUCTION_CUP    = 1 << 4,
    FAULT_IMU            = 1 << 5,
    FAULT_GPS_NO_SIGNAL  = 1 << 6,
    FAULT_COMM_TIMEOUT   = 1 << 7,
    FAULT_LOW_VOLTAGE    = 1 << 8,
    FAULT_TILT           = 1 << 9,
    FAULT_NAV_FAILED     = 1 << 10,
};

// NCU 运行参数索引（指南定义；其余索引 TODO: 对接 AP_Param）
constexpr uint16_t PARAM_IDX_BRUSH_FRONT = 0x0101;
constexpr uint16_t PARAM_IDX_BRUSH_REAR  = 0x0102;
constexpr uint16_t PARAM_IDX_BRUSH_BOTH  = 0x0103;
constexpr uint16_t PARAM_IDX_BRUSH_POWER = 0x0104;

// 帧长度
constexpr uint8_t NCU_DATA_LEN_SPEED_CTRL   = 5;
constexpr uint8_t NCU_DATA_LEN_TURN         = 6;
constexpr uint8_t NCU_DATA_LEN_PARAM_WRITE  = 7;
constexpr uint8_t NCU_DATA_LEN_PARAM_READ   = 2;
constexpr uint8_t NCU_DATA_LEN_SYSTEM_CTRL  = 1;
constexpr uint8_t NCU_DATA_LEN_POSITION     = 15;

constexpr uint8_t NCU_RX_MAX_DATA_LEN  = NCU_DATA_LEN_POSITION;
constexpr uint8_t COMPANION_RECV_TOTAL_LENGTH = FRAME_OVERHEAD + NCU_RX_MAX_DATA_LEN;

constexpr uint8_t FCU_DATA_LEN_STATUS      = 26;
constexpr uint8_t FCU_DATA_LEN_CMD_ACK     = 2;
constexpr uint8_t FCU_DATA_LEN_PARAM       = 7;
constexpr uint8_t FCU_DATA_LEN_NAV_STATUS  = 8;  // protocol 5.4: nav_state+coord_mode+dist32+heading_err

constexpr uint8_t FCU_TX_MAX_DATA_LEN  = FCU_DATA_LEN_STATUS;
constexpr uint8_t COMPANION_SEND_TOTAL_LENGTH = FRAME_OVERHEAD + FCU_TX_MAX_DATA_LEN;

constexpr uint8_t COMPANION_SEND_RESP_LENGTH  = 0x09;
constexpr uint8_t COMPANION_SEND_PARAM_LENGTH = FRAME_OVERHEAD + FCU_DATA_LEN_PARAM;
constexpr uint8_t COMPANION_SEND_NAV_LENGTH   = FRAME_OVERHEAD + FCU_DATA_LEN_NAV_STATUS;

constexpr uint32_t PACKET_TIMEOUT_MS        = 200;
constexpr uint32_t NCU_HEARTBEAT_TIMEOUT_MS = 200;

// 数据体结构体
#pragma pack(push, 1)

struct SpeedCtrlData {
    uint8_t control_mode;
    int16_t velocity;
    int16_t yaw_data;
};

struct TurnData {
    uint8_t turn_mode;
    uint8_t direction;
    uint16_t target_angle;
    uint16_t angular_vel;
};

struct ParamWriteData {
    uint16_t param_index;
    uint8_t  param_type;
    uint32_t param_value;
};

struct ParamReadData {
    uint16_t param_index;
};

struct SystemCtrlData {
    uint8_t command;
};

struct PositionData {
    uint8_t  nav_mode;
    int32_t  target_x;
    int32_t  target_y;
    int16_t  arrival_yaw;      // 0xFFFF(NAV_YAW_UNSPECIFIED) 表示到达后不指定航向
    uint16_t arrival_radius;   // cm；0 表示使用 WP_RADIUS 默认值
    int16_t  cruise_speed;     // cm/s；0 用 FCU 默认；负值表示倒车接近
};

struct StatusFeedbackData {
    uint8_t  battery_percent;
    int32_t  longitude;
    int32_t  latitude;
    uint16_t heading;          // 0~36000, 0.01°；与 ahrs.yaw_sensor（厘度）同语义
    int16_t  velocity;
    int16_t  left_track_vel;
    int16_t  right_track_vel;
    int16_t  roll;
    int16_t  pitch;
    uint8_t  control_mode;
    uint8_t  motion_state;
    uint16_t fault_code;
    uint8_t  gps_status;
};

struct CmdAckData {
    uint8_t cmd_type;
    uint8_t status;
};

struct ParamFeedbackData {
    uint16_t param_index;
    uint8_t  param_type;
    uint32_t param_value;
};

struct NavStatusData {
    uint8_t  nav_state;
    uint8_t  coord_mode;
    uint32_t distance_to_target;  // cm, uint32
    int16_t  heading_error;       // 0.01°；正值=目标在左侧
};

// FCU 发送帧（含帧头尾）
struct StatusFeedbackFrame {
    uint8_t header1;
    uint8_t header2;
    uint8_t cmd_source;
    uint8_t cmd_content;
    uint8_t data_length;
    StatusFeedbackData data;
    uint8_t checksum;
    uint8_t end_sign;
};

struct ParamFeedbackFrame {
    uint8_t header1;
    uint8_t header2;
    uint8_t cmd_source;
    uint8_t cmd_content;
    uint8_t data_length;
    ParamFeedbackData data;
    uint8_t checksum;
    uint8_t end_sign;
};

struct NavStatusFeedbackFrame {
    uint8_t header1;
    uint8_t header2;
    uint8_t cmd_source;
    uint8_t cmd_content;
    uint8_t data_length;
    NavStatusData data;
    uint8_t checksum;
    uint8_t end_sign;
};

#pragma pack(pop)

// 内部解析结构
struct ParsedSpeedCtrl {
    uint8_t mode;
    float velocity_cms;
    float yaw_value;
};

struct ParsedTurn {
    uint8_t mode;
    uint8_t direction;
    float target_angle_deg;
    float angular_vel_dps;
};

struct ParsedPosition {
    uint8_t  nav_mode;
    double   tgt_lat;
    double   tgt_lon;
    float    tgt_north_cm;
    float    tgt_east_cm;
    float    arrival_yaw_deg;
    uint16_t arrival_radius_cm;
    int16_t  cruise_speed_cms;
};

// 数据包构建器
class PacketBuilder {
public:
    template <typename T>
    static std::array<uint8_t, sizeof(T)> serialize(const T& packet) {
        std::array<uint8_t, sizeof(T)> buffer;
        memcpy(buffer.data(), &packet, sizeof(T));
        return buffer;
    }

    template <typename T>
    static T deserialize(const uint8_t* data) {
        T packet;
        memcpy(&packet, data, sizeof(T));
        return packet;
    }
};
