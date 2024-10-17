#include <AP_Logger/AP_Logger_config.h>

#if HAL_LOGGING_ENABLED

#include "AC_PosControl.h"

#include <AP_Logger/AP_Logger.h>
#include "LogStructure.h"

// a convenience function for writing out the position controller PIDs
void AC_PosControl::Write_PSCx(LogMessages id, float pos_desired, float pos_target, float pos, float vel_desired, float vel_target, float vel, float accel_desired, float accel_target, float accel)
{
    const struct log_PSCx pkt{
        LOG_PACKET_HEADER_INIT(id),
            time_us         : AP_HAL::micros64(),
            pos_desired   : pos_desired * 0.01f,
            pos_target    : pos_target * 0.01f,
            pos           : pos * 0.01f,
            vel_desired   : vel_desired * 0.01f,
            vel_target    : vel_target * 0.01f,
            vel           : vel * 0.01f,
            accel_desired : accel_desired * 0.01f,
            accel_target  : accel_target * 0.01f,
            accel         : accel * 0.01f
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void AC_PosControl::Write_PSCN(float pos_desired, float pos_target, float pos, float vel_desired, float vel_target, float vel, float accel_desired, float accel_target, float accel)
{
    Write_PSCx(LOG_PSCN_MSG, pos_desired, pos_target, pos, vel_desired, vel_target, vel, accel_desired, accel_target, accel);
}

void AC_PosControl::Write_PSCE(float pos_desired, float pos_target, float pos, float vel_desired, float vel_target, float vel, float accel_desired, float accel_target, float accel)
{
    Write_PSCx(LOG_PSCE_MSG, pos_desired, pos_target, pos, vel_desired, vel_target, vel, accel_desired, accel_target, accel);
}

void AC_PosControl::Write_PSCD(float pos_desired, float pos_target, float pos, float vel_desired, float vel_target, float vel, float accel_desired, float accel_target, float accel)
{
    Write_PSCx(LOG_PSCD_MSG, pos_desired, pos_target, pos, vel_desired, vel_target, vel, accel_desired, accel_target, accel);
}

// a convenience function for writing out the position controller offsets
void AC_PosControl::Write_PSOx(LogMessages id, float pos_target_offset_cm, float pos_offset_cm,
                               float vel_target_offset_cms, float vel_offset_cms,
                               float accel_target_offset_cmss, float accel_offset_cmss)
{
    const struct log_PSOx pkt{
        LOG_PACKET_HEADER_INIT(id),
            time_us             : AP_HAL::micros64(),
            pos_target_offset   : pos_target_offset_cm * 0.01f,
            pos_offset          : pos_offset_cm * 0.01f,
            vel_target_offset   : vel_target_offset_cms * 0.01f,
            vel_offset          : vel_offset_cms * 0.01f,
            accel_target_offset : accel_target_offset_cmss * 0.01f,
            accel_offset        : accel_offset_cmss * 0.01f,
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void AC_PosControl::Write_PSON(float pos_target_offset_cm, float pos_offset_cm,
                               float vel_target_offset_cms, float vel_offset_cms,
                               float accel_target_offset_cmss, float accel_offset_cmss)
{
    Write_PSOx(LOG_PSON_MSG, pos_target_offset_cm, pos_offset_cm, vel_target_offset_cms, vel_offset_cms, accel_target_offset_cmss, accel_offset_cmss);
}

void AC_PosControl::Write_PSOE(float pos_target_offset_cm, float pos_offset_cm,
                               float vel_target_offset_cms, float vel_offset_cms,
                               float accel_target_offset_cmss, float accel_offset_cmss)
{
    Write_PSOx(LOG_PSOE_MSG, pos_target_offset_cm, pos_offset_cm, vel_target_offset_cms, vel_offset_cms, accel_target_offset_cmss, accel_offset_cmss);
}

void AC_PosControl::Write_PSOD(float pos_target_offset_cm, float pos_offset_cm,
                               float vel_target_offset_cms, float vel_offset_cms,
                               float accel_target_offset_cmss, float accel_offset_cmss)
{
    Write_PSOx(LOG_PSOD_MSG, pos_target_offset_cm, pos_offset_cm, vel_target_offset_cms, vel_offset_cms, accel_target_offset_cmss, accel_offset_cmss);
}

void AC_PosControl::Write_PSOT(float pos_target_offset_cm, float pos_offset_cm,
                               float vel_target_offset_cms, float vel_offset_cms,
                               float accel_target_offset_cmss, float accel_offset_cmss)
{
    Write_PSOx(LOG_PSOT_MSG, pos_target_offset_cm, pos_offset_cm, vel_target_offset_cms, vel_offset_cms, accel_target_offset_cmss, accel_offset_cmss);
}

#endif  // HAL_LOGGING_ENABLED
