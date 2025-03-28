/*** * ECU通讯*** */
#pragma once

// 必要的头文件包含
#include <AP_HAL/AP_HAL.h>          // 硬件抽象层
#include <AP_SerialManager/AP_SerialManager.h>  // 串口管理
#include <vector>                   // STL向量容器

/**
 * @brief 查询消息结构体（87字节数据包）
 * @note 所有多字节字段均采用小端字节序
 */
struct Query1Msg {
    // 字节0：协议头标识
    uint8_t char_length;            // 报文长度标识（固定值0x135）

    // 字节1：设备标识
    uint8_t id_code;                // 设备识别码（0x4表示主控ECU）

    // 字节2-3：保留字段
    uint16_t reserved_2_3;          // 协议预留位（必须置0）

    // 字节4-5：存储配置
    uint16_t store_def;             // 存储模式定义（位掩码）
                                    // 数值为 1 时，在关闭后数据自动在存储器中储存；
                                    // 其他数据， 不存储。
    
    // 字节6-9：保留字段
    uint32_t reserved_6_9;          // 协议扩展预留位

    // 字节10-11：状态标志
    uint16_t engine_state;          // 发动机状态（位掩码）：
                                    // 0: 手动调节控制
                                    // 1:启动状态1
                                    // 2:启动状态2
                                    // 3:机器预热过程
                                    // 4:正常运行1
                                    // 5:正常运行2


    // 字节12-13：转速数据
    uint16_t engine_rpm;            // 发动机转速（RPM）

    // 字节14-17：保留字段
    uint32_t reserved_14_17;        // 未来功能扩展

    // 字节18-19：暖机计时
    uint16_t warmup_time_remaining; // 暖机剩余时间（秒）

    // 字节20-23：保留字段
    uint32_t reserved_20_23;        // 诊断预留位

    // 字节24-25：怠速控制
    uint16_t idle_position;         // 怠速节气门开度（0-100%）

    // 字节26-27：干扰计数
    uint16_t interference_count;    // 电磁干扰脉冲计数

    // 字节28-29：进气系统
    uint16_t air_intake;            // 进气流量（g/s）0...1023
    //30-31进气压力(增压后、节流前)
    uint16_t air_pressure;          // 进气歧管压力（kPa）0...255 255=100%

    // 字节32-33：诊断数据
    uint16_t rpm_error_count;       // 转速信号错误计数

    // 字节34-35：燃油系统
    uint16_t injection_time;        // 喷射时间
    // 字节36-37：点火提前角
    uint16_t ignition_angle;        // 点火提前角（0.1°）

    // 字节38-39：保留字段
    uint16_t reserved_38_39;        // 校准预留位

    // 字节40-55：传感器电压（单位：mV）
    uint16_t throttle_position_voltage;  // 节气门位置传感器0...1023(1023=5v)
    uint16_t battery_voltage;            // 主电池电压0...1023(1023=5v)
    uint16_t afr_sensor_voltage;         // 空燃比传感器0...1023(1023=5v)
    uint16_t engine_temp_voltage;        // 水温传感器0...1023(1023=5v)
    uint16_t intake_temp_voltage;        // 进气温度传感器0...1023(1023=5v)
    uint16_t external_pressure_voltage;  // 大气压力传感器0...1023(1023=5v)
    uint16_t intake_pressure_voltage;    // 进气压力传感器0...1023(1023=5v)
    uint16_t intake_quantity_voltage;    // 空气流量计0...1023(1023=5v)

    // 字节56-57：空燃比
    int16_t afr_value;              // 实际空燃比 -128...127(0=入为1)

    // 字节58-73：保留字段
    uint16_t reserved_58_73;        // 扩展数据区

    // 字节74-83：运行参数
    uint16_t throttle_position;     // 节气门开度（%）0...90
    uint16_t engine_cooling_temp;   // 冷却液温度（℃）
    uint16_t ecu_battery_voltage;   // ECU工作电压（mV）
    uint16_t intake_temp;           // 进气温度（℃）
    uint16_t intake_pressure;       // 进气压力（kPa）

    // 字节84-85：状态标志
    uint16_t sensor_status;         // 传感器状态（位掩码）：
                                    // 1=空燃比传感器正常
                                    // 1=发动机温度传感器正常
                                    // 1=进气温度传感器正常
                                    // 1=进气压力传感器正常

    // 字节86：校验和
    uint8_t checksum;               // CRC8校验值
};

/**
 * @brief LIMBACH协议查询类
 * @note 实现Query1类型数据包的收发处理
 */
class AP_Limbach_Query1 {
public:
    /**
     * @brief 默认构造函数
     */
    AP_Limbach_Query1();

    // 禁止拷贝构造和赋值操作
    AP_Limbach_Query1(const AP_Limbach_Query1 &other) = delete;
    AP_Limbach_Query1 &operator=(const AP_Limbach_Query1&) = delete;

    /**
     * @brief 初始化串口通信
     * @param serial_manager 串口管理对象引用
     */
    void init(const AP_SerialManager& serial_manager);
    
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
    Query1Msg accepting_request(void);
        
private:
    AP_HAL::UARTDriver *_port;      // 串口驱动指针
    std::vector<uint8_t> response_buffer;  // 接收数据缓冲区

};