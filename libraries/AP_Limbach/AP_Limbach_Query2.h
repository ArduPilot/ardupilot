/*** * ECU通讯*** */
#pragma once

// 必要的头文件包含
#include <AP_HAL/AP_HAL.h>                     // 硬件抽象层
#include <AP_SerialManager/AP_SerialManager.h> // 串口管理
#include <vector>                              // STL向量容器

/**
 * @brief 查询消息结构体（101字节数据包）
 * @note 所有多字节字段均采用小端字节序
 */
struct Query2Msg
{
    uint8_t char_length;                    // 字节 0: 字符长度
    uint8_t id_code;                        // 字节 1: 识别码
    uint64_t reserved_2_9;                  // 字节 2-9: 预留位
    uint16_t intake_injection;              // 字节 10-11: 进气量决定的喷射量
    uint16_t intake_correction_injection;   // 字节 12-13: 进气量修正曲线决定的喷射量
    uint16_t base_curve_injection;          // 字节 14-15: 基本特性曲线决定的喷射量
    uint16_t intake_pressure_injection;     // 字节 16-17: 进气管负压曲线决定的喷射量
    uint16_t base_injection;                // 字节 18-19: 喷射的基本量
    uint16_t intake_correction_injection_2; // 字节 20-21: 进气修正后的喷射量
    uint16_t idle_position_injection;       // 字节 22-23: 怠速位置传感器决定的喷射量
    uint16_t warmup_curve_injection;        // 字节 24-25: 暖机过程曲线决定的喷射量
    uint16_t acceleration_injection;        // 字节 26-27: 加速时决定的喷射量
    uint16_t intake_valve_time;             // 字节 28-29: 进气门的开合时间
    uint16_t race_mode_injection;           // 字节 30-31: RACE 模式决定的喷射量
    uint16_t afr_control_injection;         // 字节 32-33: 空燃比控制器决定的喷射量
    uint16_t ignition_curve_advance;        // 字节 34-35: 点火时间曲线决定的点火提前角
    uint16_t intake_temp_advance;           // 字节 36-37: 进气温度曲线决定的点火提前角
    uint16_t intake_pressure_advance;       // 字节 38-39: 进气压力曲线决定的点火提前角
    uint16_t engine_temp_advance;           // 字节 40-41: 发动机温度曲线决定的点火提前角
    uint16_t acceleration_advance;          // 字节 42-43: 加速时决定的点火提前角
    uint16_t race_mode_advance;             // 字节 44-45: RACE 模式决定的点火提前角
    uint32_t total_storage_time;            // 字节 46-49: 26 次存储内总时间
    uint32_t total_rotations;               // 字节 50-53: 旋转总数
    uint16_t fuel_consumption;              // 字节 54-55: 燃油消耗（0.1l/h）
    uint16_t error_count;                   // 字节 56-57: 出错存储器中的错误数量
    uint32_t reserved_58_99;                // 字节 58-99: 预留位
    uint16_t checksum;                      // 字节 100: 检验字节
};

/**
 * @brief LIMBACH协议查询类
 * @note 实现Query2类型数据包的收发处理
 */
class AP_Limbach_Query2
{
public:
    /**
     * @brief 默认构造函数
     */
    AP_Limbach_Query2();

    // 禁止拷贝构造和赋值操作
    AP_Limbach_Query2(const AP_Limbach_Query2 &other) = delete;
    AP_Limbach_Query2 &operator=(const AP_Limbach_Query2 &) = delete;

    /**
     * @brief 初始化串口通信
     * @param serial_manager 串口管理对象引用
     */
    void init(const AP_SerialManager &serial_manager);

    /**
     * @brief 发送查询请求
     * @return bool 发送成功返回true
     */
    bool send_request(void);

    /**
     * @brief 处理接收到的数据
     * @return Query1Msg 解析后的数据结构
     * @throws 当数据校验失败时抛出异常
     */
    Query2Msg accepting_request(void);

private:
    AP_HAL::UARTDriver *_port;            // 串口驱动指针
    std::vector<uint8_t> response_buffer; // 接收数据缓冲区
};