// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Tracker.h"

#if LOGGING_ENABLED == ENABLED

// Code to Write and Read packets from DataFlash log memory

// Write an attitude packet
void Tracker::Log_Write_Attitude()
{
    Vector3f targets;
    targets.y = nav_status.pitch * 100.0f;
    targets.z = wrap_360_cd(nav_status.bearing * 100.0f);
    DataFlash.Log_Write_Attitude(ahrs, targets);
    DataFlash.Log_Write_EKF(ahrs,false);
    DataFlash.Log_Write_AHRS2(ahrs);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    sitl.Log_Write_SIMSTATE(&DataFlash);
#endif
    DataFlash.Log_Write_POS(ahrs);
}

void Tracker::Log_Write_Baro(void)
{
    DataFlash.Log_Write_Baro(barometer);
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
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
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
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

const struct LogStructure Tracker::log_structure[] = {
    LOG_COMMON_STRUCTURES,
    {LOG_V_BAR_MSG, sizeof(log_Vehicle_Baro),
        "VBAR", "Qff", "TimeUS,Press,AltDiff" },
    {LOG_V_POS_MSG, sizeof(log_Vehicle_Pos),
        "VPOS", "QLLefff", "TimeUS,Lat,Lng,Alt,VelX,VelY,VelZ" }
};

void Tracker::Log_Write_Vehicle_Startup_Messages()
{
    DataFlash.Log_Write_Mode(control_mode);
}

// start a new log
void Tracker::start_logging()
{
    if (g.log_bitmask != 0) {
        if (!logging_started) {
            logging_started = true;
            DataFlash.setVehicle_Startup_Log_Writer(FUNCTOR_BIND(&tracker, &Tracker::Log_Write_Vehicle_Startup_Messages, void));
            DataFlash.StartNewLog();
        }
        // enable writes
        DataFlash.EnableWrites(true);
    }
}

void Tracker::log_init(void)
{
    DataFlash.Init(log_structure, ARRAY_SIZE(log_structure));
    if (!DataFlash.CardInserted()) {
        gcs_send_text(MAV_SEVERITY_WARNING, "No dataflash card inserted");
        g.log_bitmask.set(0);
    } else if (DataFlash.NeedPrep()) {
        gcs_send_text(MAV_SEVERITY_INFO, "Preparing log system");
        DataFlash.Prep();
        gcs_send_text(MAV_SEVERITY_INFO, "Prepared log system");
        for (uint8_t i=0; i<num_gcs; i++) {
            gcs[i].reset_cli_timeout();
        }
    }

    if (g.log_bitmask != 0) {
        start_logging();
    }
}

#else // LOGGING_ENABLED

void Tracker::Log_Write_Attitude(void) {}
void Tracker::Log_Write_Startup() {}
void Tracker::Log_Write_Baro(void) {}

void Tracker::start_logging() {}
void Tracker::log_init(void) {}

#endif // LOGGING_ENABLED
