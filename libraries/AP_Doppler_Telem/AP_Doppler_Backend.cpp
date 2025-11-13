#include "AP_Doppler_Backend.h"

#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_RPM/AP_RPM.h>
#include <GCS_MAVLink/GCS.h>


extern const AP_HAL::HAL& hal;

bool AP_Doppler_Backend::init()
{
    // if SPort Passthrough is using external data then it will
    // override this to do nothing:
    return init_serial_port();
}

bool AP_Doppler_Backend::init_serial_port()
{
//    if (!hal.scheduler->thread_create(
//            FUNCTOR_BIND_MEMBER(&AP_Doppler_Backend::loop, void),
//            "Doppler",
//            1024,
//            AP_HAL::Scheduler::PRIORITY_UART,
//            1)) {
//        return false;
//    }
    // we don't want flow control for either protocol
    _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    _port->begin(AP_DOPPLER_BAUD);
    return true;
}

/*
  thread to loop handling bytes
 */
void AP_Doppler_Backend::loop(void)
{
    if (_port == nullptr) 
    {
        return;
    }
    

    if (_port->available() == 0) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Doppler: No Data");
        return;
    }

    // 等待帧头 ':'
    int ch;
    while (_port->available() != 0) {
        ch = _port->read();
        if (ch == ':') 
        {
            
            gcs().send_text(MAV_SEVERITY_WARNING, "Doppler: Find_message_head");
            break;
        }
        
    }

    // 读取数据
    char buffer[128];
    uint8_t idx = 0;
    while (idx < sizeof(buffer) - 1) {
        if (_port->available() == 0) {
            break;
        }
        char c = _port->read();
        if (c == '\n' || c == '\r') break;
        buffer[idx++] = c;
    }
    buffer[idx] = '\0';

    // 分发解析
    if (strncmp(buffer, "SA", 2) == 0) parse_SA(buffer);
    else if (strncmp(buffer, "TS", 2) == 0) parse_TS(buffer);
    else if (strncmp(buffer, "BI", 2) == 0) parse_BI(buffer);
    else if (strncmp(buffer, "BS", 2) == 0) parse_BS(buffer);
    else if (strncmp(buffer, "BE", 2) == 0) parse_BE(buffer);
    else if (strncmp(buffer, "BD", 2) == 0) parse_BD(buffer);
    else if (strncmp(buffer, "WI", 2) == 0) parse_WI(buffer);
    else if (strncmp(buffer, "WS", 2) == 0) parse_WS(buffer);
    else if (strncmp(buffer, "WE", 2) == 0) parse_WE(buffer);
    else if (strncmp(buffer, "WD", 2) == 0) parse_WD(buffer);
}



void AP_Doppler_Backend::send()
{
    if (_port == nullptr) 
    {
        _port->write((const uint8_t*)":TS,REQUEST\r\n", 14);
    }
    gcs().send_text(MAV_SEVERITY_INFO,
                "Doppler: BT[%+.2f,%+.2f,%+.2f] mm/s",
                BottomTrackEarthVel.east_velocity_mm_s  ,
                BottomTrackEarthVel.north_velocity_mm_s ,
                BottomTrackEarthVel.up_velocity_mm_s    );


    gcs().send_text(MAV_SEVERITY_INFO,
                "Doppler:SA[%+.2f,%+.2f,%+.2f] mm/s",
                BottomTrackShipVel.x_velocity_mm_s ,
                BottomTrackShipVel.y_velocity_mm_s ,
                BottomTrackShipVel.z_velocity_mm_s    );

    gcs().send_text(MAV_SEVERITY_INFO,
                "Doppler:BI[%+.2f,%+.2f,%+.2f] m/s",
                velocity_data.X_velocity_m_s  ,
                velocity_data.Y_velocity_m_s  ,
                velocity_data.Z_velocity_m_s    );

}


// --------------- 几个超小的字段读取工具 ---------------
static inline float parse_float(const char *&p)
{
    // 跳过前导空格（PD6 中字段前可能有空格填充）
    while (*p == ' ') ++p;

    char *end = nullptr;
    // strtof 接受 const char* 输入，但 end 必须为 char*
    float v = strtof(p, &end);
    if (end == p) {              // 转换失败
        v = NAN;                 // 标记非法
        // 若解析失败，尝试跳到下一个逗号以免死循环
        const char *q = p;
        while (*q && *q != ',') ++q;
        p = q;
        if (*p == ',') ++p;
        return v;
    }
    // 将 p 推进到解析结束位置（注意类型转换）
    p = const_cast<const char*>(end);

    // 跳过逗号分隔符（如果有）
    if (*p == ',') ++p;
    return v;
}

static inline char parse_char(const char *&p)
{
    // 跳过前导空格
    while (*p == ' ') ++p;

    char c = *p;
    // 如果行尾或遇到逗号前没有字符，返回 0 表示无效
    if (c == '\0' || c == '\r' || c == '\n') {
        return '\0';
    }

    // advance past this char
    ++p;

    // 如果当前为逗号（紧接着），跳过
    if (*p == ',') ++p;

    return c;
}


// 如果后续需要整型/字符串再扩展即可，目前只用 float/char

// --------------- 具体帧解析 ---------------

void AP_Doppler_Backend::parse_SA(const char *payload)
{
    // SA,roll,pitch,yaw
    const char *p = payload + 3;   // 跳过 "SA,"
    Posture_data.pitch_deg = parse_float(p);
    Posture_data.roll_deg  = parse_float(p);
    Posture_data.yaw_deg   = parse_float(p);

    gcs().send_text(MAV_SEVERITY_INFO,"SA_success");
}

void AP_Doppler_Backend::parse_TS(const char *payload)
{
    // TS,2025-11-07 15:22:40,sat,temp,deep,volt,status,ver
    const char *p = payload + 3;

    /* 1. 时间字符串单独处理：读到下一个逗号为止 */
    char *comma = strchr(p, ',');
    if (!comma) return;
    size_t len = comma - p;
    if (len >= sizeof(Parameters_data.time)) len = sizeof(Parameters_data.time) - 1;
    memcpy(Parameters_data.time, p, len);
    Parameters_data.time[len] = '\0';
    p = comma + 1;

    Parameters_data.satellites_PPT = parse_float(p);
    Parameters_data.temperature_C  = parse_float(p);
    Parameters_data.deep_m         = parse_float(p);
    Parameters_data.voltage_m_s    = parse_float(p);
    char status_char               = parse_char(p);
    Parameters_data.status         = static_cast<Message_Status>(status_char);
    Parameters_data.version        = (uint8_t)parse_float(p);
    gcs().send_text(MAV_SEVERITY_INFO,"TS_success");
}

void AP_Doppler_Backend::parse_BI(const char *payload)
{
    const char *p = payload + 3;
    velocity_data.X_velocity_m_s     = parse_float(p);
    velocity_data.Y_velocity_m_s     = parse_float(p);
    velocity_data.Z_velocity_m_s     = parse_float(p);
    velocity_data.velocity_error_mm_s = parse_float(p);
    velocity_data.status              = static_cast<Message_Status>(parse_char(p));
    gcs().send_text(MAV_SEVERITY_INFO,"BI_success");
}

void AP_Doppler_Backend::parse_BS(const char *payload)
{
    const char *p = payload + 3;
    BottomTrackShipVel.x_velocity_mm_s = parse_float(p);
    BottomTrackShipVel.y_velocity_mm_s = parse_float(p);
    BottomTrackShipVel.z_velocity_mm_s = parse_float(p);
    BottomTrackShipVel.status          = static_cast<Message_Status>(parse_char(p));
    gcs().send_text(MAV_SEVERITY_INFO,"BS_success");
}

void AP_Doppler_Backend::parse_BE(const char *payload)
{
    const char *p = payload + 3;
    BottomTrackEarthVel.east_velocity_mm_s  = parse_float(p);
    BottomTrackEarthVel.north_velocity_mm_s = parse_float(p);
    BottomTrackEarthVel.up_velocity_mm_s    = parse_float(p);
    BottomTrackEarthVel.status              = static_cast<Message_Status>(parse_char(p));
    gcs().send_text(MAV_SEVERITY_INFO,"BE_success");
}

void AP_Doppler_Backend::parse_BD(const char *payload)
{
    const char *p = payload + 3;
    BottomTrackDistance.east_distance_m        = parse_float(p);
    BottomTrackDistance.north_distance_m       = parse_float(p);
    BottomTrackDistance.up_distance_m          = parse_float(p);
    BottomTrackDistance.bottom_distance_m      = parse_float(p);
    BottomTrackDistance.time_since_valid_s     = parse_float(p);
    gcs().send_text(MAV_SEVERITY_INFO,"BD_success");
}

void AP_Doppler_Backend::parse_WI(const char *payload)
{
    const char *p = payload + 3;
    WaterTrackInstrumentVel.x_velocity_mm_s     = parse_float(p);
    WaterTrackInstrumentVel.y_velocity_mm_s     = parse_float(p);
    WaterTrackInstrumentVel.z_velocity_mm_s     = parse_float(p);
    WaterTrackInstrumentVel.velocity_error_mm_s = parse_float(p);
    WaterTrackInstrumentVel.status              = static_cast<Message_Status>(parse_char(p));
    gcs().send_text(MAV_SEVERITY_INFO,"WI_success");
}

void AP_Doppler_Backend::parse_WS(const char *payload)
{
    const char *p = payload + 3;
    WaterTrackShipVel.x_velocity_mm_s = parse_float(p);
    WaterTrackShipVel.y_velocity_mm_s = parse_float(p);
    WaterTrackShipVel.z_velocity_mm_s = parse_float(p);
    WaterTrackShipVel.status          = static_cast<Message_Status>(parse_char(p));
    gcs().send_text(MAV_SEVERITY_INFO,"WS_success");
}

void AP_Doppler_Backend::parse_WE(const char *payload)
{
    const char *p = payload + 3;
    WaterTrackEarthVel.east_velocity_mm_s  = parse_float(p);
    WaterTrackEarthVel.north_velocity_mm_s = parse_float(p);
    WaterTrackEarthVel.up_velocity_mm_s    = parse_float(p);
    WaterTrackEarthVel.status              = static_cast<Message_Status>(parse_char(p));
    gcs().send_text(MAV_SEVERITY_INFO,"WE_success");
}

void AP_Doppler_Backend::parse_WD(const char *payload)
{
    const char *p = payload + 3;
    WaterTrackEarthDist.east_distance_m     = parse_float(p);
    WaterTrackEarthDist.north_distance_m    = parse_float(p);
    WaterTrackEarthDist.up_distance_m       = parse_float(p);
    WaterTrackEarthDist.center_distance_m   = parse_float(p);
    WaterTrackEarthDist.time_since_valid_s  = parse_float(p);
    gcs().send_text(MAV_SEVERITY_INFO,"WD_success");
}