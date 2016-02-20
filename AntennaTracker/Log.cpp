// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Tracker.h"

#if LOGGING_ENABLED == ENABLED

// Code to Write and Read packets from DataFlash log memory

// Write an attitude packet
void Tracker::Log_Write_Attitude()
{
    Vector3f targets;
    targets.y = nav_status.pitch * 100.0f;
    targets.z = wrap_360_cd_float(nav_status.bearing * 100.0f);
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

const struct LogStructure Tracker::log_structure[] = {
    LOG_COMMON_STRUCTURES,
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
