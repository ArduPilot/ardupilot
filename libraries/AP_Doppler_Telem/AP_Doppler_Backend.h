#pragma once

#include <AP_Math/AP_Math.h>   // strtof 声明
#include <cstring>             // strchr、strncmp 等

#include "string.h"
#define AP_DOPPLER_BAUD 115200
#define AP_DOPPLER_BUFSIZE_RX 512
#define AP_DOPPLER_BUFSIZE_TX 512

class AP_Doppler_Backend
{
public:
    AP_Doppler_Backend(AP_HAL::UARTDriver *port) :
        _port(port) { }

    virtual ~AP_Doppler_Backend()  {}

    virtual bool init();
    virtual void send();
    virtual void loop();
    typedef union {
        struct PACKED {
            uint8_t sensor;
            uint8_t frame;
            uint16_t appid;
            uint32_t data;
        };
        uint8_t raw[8];
    } sport_packet_t;

    // SPort is at 57600, D overrides this
    virtual uint32_t initial_baud() const
    {
        return AP_DOPPLER_BAUD;
    }

    // get next telemetry data for external consumers of SPort data


    




protected:

    AP_HAL::UARTDriver *_port;  // UART used to send data to Doppler receiver

    virtual bool init_serial_port();

    enum Message_Status : char {
        STATUS_INVALID = 'V',
        STATUS_ACQUIRING = 'A',
        ZERO_ERROR = '0',
        ONE_ERROR = '1',
        TWO_ERROR = '2',
        THREE_ERROR = '3',
        FOUR_ERROR = '4',
    };

    struct  {
        const char* FLAGS = "BE";
        float east_velocity_mm_s;    // 东向速度
        float north_velocity_mm_s;   // 北向速度
        float up_velocity_mm_s;      // 向上速度
        Message_Status  status;             // A=有效，V=无效
    } BottomTrackEarthVel;

    //系统姿态数据
    struct {
        const char* FLAGS = "SA";
        float roll_deg;
        float pitch_deg;
        float yaw_deg;

    } Posture_data;
    //系统参数数据
    struct {
        const char* FLAGS = "TS";
        char time[14];
        float satellites_PPT;
        float temperature_C;
        float deep_m;
        float voltage_m_s;
        Message_Status status;
        uint8_t version;
        
    } Parameters_data;
    //速度数据
    struct {
        const char* FLAGS = "BI";
        float X_velocity_m_s;
        float Y_velocity_m_s;
        float Z_velocity_m_s;   
        float velocity_error_mm_s;
        Message_Status  status;             // A=有效，V=无效   
        
    } velocity_data;
    //底锁定速度数据
    struct {
        const char* FLAGS = "BS";
        float x_velocity_mm_s;       // 船头方向
        float y_velocity_mm_s;       // 右舷方向
        float z_velocity_mm_s;       // 向下
        Message_Status  status;             // A=有效，V=无效
    } BottomTrackShipVel;


    struct  {
        const char* FLAGS = "BD";
        float east_distance_m;       // 东向距离（m）
        float north_distance_m;      // 北向距离（m）
        float up_distance_m;         // 向上距离（m）
        float bottom_distance_m;     // 设备离底距离（m）
        float time_since_valid_s;    // 距上次有效速度估计的时间（s）
    } BottomTrackDistance;

    struct  {
        const char* FLAGS = "WI";
        float x_velocity_mm_s;       // X轴速度（mm/s）
        float y_velocity_mm_s;       // Y轴速度（mm/s）
        float z_velocity_mm_s;       // Z轴速度（mm/s）
        float velocity_error_mm_s;   // 速度误差
        Message_Status  status;             // A=有效，V=无效
    }WaterTrackInstrumentVel;

    struct  {
        const char* FLAGS = "WS";
        float x_velocity_mm_s;       // 横向速度
        float y_velocity_mm_s;       // 纵向速度
        float z_velocity_mm_s;       // 正常方向速度
        Message_Status  status;             // A=有效，V=无效
    }WaterTrackShipVel;

    struct  {
        const char* FLAGS = "WE";
        float east_velocity_mm_s;    // 东向速度
        float north_velocity_mm_s;   // 北向速度
        float up_velocity_mm_s;      // 向上速度
        Message_Status  status;             // A=有效，V=无效
    }WaterTrackEarthVel;

    struct  {
        const char* FLAGS = "WD";
        float east_distance_m;       // 东向距离
        float north_distance_m;      // 北向距离
        float up_distance_m;         // 向上距离
        float center_distance_m;     // 水团中心到仪器的距离
        float time_since_valid_s;    // 距上次有效速度估计时间（s）
    }WaterTrackEarthDist;


    /*
      for Doppler  protocol (receivers)
    */

    enum Doppler_options_e : uint8_t {
        OPTION_AIRSPEED_AND_GROUNDSPEED = 1U<<0,
    };

private:
    void parse_SA(const char *payload);
    void parse_TS(const char *payload);
    void parse_BI(const char *payload);
    void parse_BS(const char *payload);
    void parse_BE(const char *payload);
    void parse_BD(const char *payload);
    void parse_WI(const char *payload);
    void parse_WS(const char *payload);
    void parse_WE(const char *payload);
    void parse_WD(const char *payload);

    

};

