#include "Tracker.h"

#if LOGGING_ENABLED == ENABLED

// Code to Write and Read packets from AP_Logger log memory

// Write an attitude packet
void Tracker::Log_Write_Attitude()
{
    Vector3f targets;
    targets.y = nav_status.pitch * 100.0f;
    targets.z = wrap_360_cd(nav_status.bearing * 100.0f);
    logger.Write_Attitude(ahrs, targets);
    logger.Write_EKF(ahrs);
    logger.Write_AHRS2(ahrs);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    sitl.Log_Write_SIMSTATE();
#endif
    logger.Write_POS(ahrs);
}

struct PACKED log_Vehicle_Baro {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float    press;
    float    alt_diff;
};

// Write a vehicle baro packet
void Tracker::Log_Write_Vehicle_Baro(float pressure, float altitude)
{
    struct log_Vehicle_Baro pkt = {
        LOG_PACKET_HEADER_INIT(LOG_V_BAR_MSG),
        time_us         : AP_HAL::micros64(),
        press           : pressure,
        alt_diff        : altitude
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Vehicle_Pos {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int32_t vehicle_lat;
    int32_t vehicle_lng;
    int32_t vehicle_alt;
    float vehicle_vel_x;
    float vehicle_vel_y;
    float vehicle_vel_z;
};

// Write a vehicle pos packet
void Tracker::Log_Write_Vehicle_Pos(int32_t lat, int32_t lng, int32_t alt, const Vector3f& vel)
{
    struct log_Vehicle_Pos pkt = {
        LOG_PACKET_HEADER_INIT(LOG_V_POS_MSG),
        time_us         : AP_HAL::micros64(),
        vehicle_lat     : lat,
        vehicle_lng     : lng,
        vehicle_alt     : alt,
        vehicle_vel_x   : vel.x,
        vehicle_vel_y   : vel.y,
        vehicle_vel_z   : vel.z,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

// type and unit information can be found in
// libraries/AP_Logger/Logstructure.h; search for "log_Units" for
// units and "Format characters" for field type information
const struct LogStructure Tracker::log_structure[] = {
    LOG_COMMON_STRUCTURES,
    {LOG_V_BAR_MSG, sizeof(log_Vehicle_Baro),
        "VBAR", "Qff", "TimeUS,Press,AltDiff", "sPm", "F00" },
    {LOG_V_POS_MSG, sizeof(log_Vehicle_Pos),
        "VPOS", "QLLefff", "TimeUS,Lat,Lng,Alt,VelX,VelY,VelZ", "sddmnnn", "FGGB000" }
};

void Tracker::Log_Write_Vehicle_Startup_Messages()
{
    logger.Write_Mode(control_mode, MODE_REASON_INITIALISED);
    gps.Write_AP_Logger_Log_Startup_messages();
}

void Tracker::log_init(void)
{
    logger.Init(log_structure, ARRAY_SIZE(log_structure));
}

#else // LOGGING_ENABLED

void Tracker::Log_Write_Attitude(void) {}

void Tracker::log_init(void) {}
void Tracker::Log_Write_Vehicle_Pos(int32_t lat, int32_t lng, int32_t alt, const Vector3f& vel) {}
void Tracker::Log_Write_Vehicle_Baro(float pressure, float altitude) {}
void Tracker::Log_Write_Vehicle_Startup_Messages() {}

#endif // LOGGING_ENABLED
