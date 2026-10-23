#include <AP_Logger/AP_Logger_config.h>

#if HAL_LOGGING_ENABLED

#include "AC_PosControl.h"

#include <AP_Logger/AP_Logger.h>
#include "LogStructure.h"

// Internal log writer for PSCx (North, East, Down tracking).
// Reduces duplication between Write_PSCN, PSCE, and PSCD.
// Used for logging desired/target/actual position, velocity, and acceleration per axis.
void AC_PosControl::Write_PSCx(LogMessages id, float pos_desired_m, float pos_target_m, float pos_m, float vel_desired_ms, float vel_target_ms, float vel_ms, float accel_desired_mss, float accel_target_mss, float accel_mss)
{
    const struct log_PSCx pkt{
        LOG_PACKET_HEADER_INIT(id),
            time_us         : AP_HAL::micros64(),
            pos_desired   : pos_desired_m,
            pos_target    : pos_target_m,
            pos           : pos_m,
            vel_desired   : vel_desired_ms,
            vel_target    : vel_target_ms,
            vel           : vel_ms,
            accel_desired : accel_desired_mss,
            accel_target  : accel_target_mss,
            accel         : accel_mss
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// Logs position controller state along the North axis to PSCN..
// Logs desired, target, and actual position [m], velocity [m/s], and acceleration [m/s²].
void AC_PosControl::Write_PSCN(float pos_desired_m, float pos_target_m, float pos_m, float vel_desired_ms, float vel_target_ms, float vel_ms, float accel_desired_mss, float accel_target_mss, float accel_mss)
{
    Write_PSCx(LOG_PSCN_MSG, pos_desired_m, pos_target_m, pos_m, vel_desired_ms, vel_target_ms, vel_ms, accel_desired_mss, accel_target_mss, accel_mss);
}

// Logs position controller state along the East axis to PSCE.
// Logs desired, target, and actual values for position [m], velocity [m/s], and acceleration [m/s²].
void AC_PosControl::Write_PSCE(float pos_desired_m, float pos_target_m, float pos_m, float vel_desired_ms, float vel_target_ms, float vel_ms, float accel_desired_mss, float accel_target_mss, float accel_mss)
{
    Write_PSCx(LOG_PSCE_MSG, pos_desired_m, pos_target_m, pos_m, vel_desired_ms, vel_target_ms, vel_ms, accel_desired_mss, accel_target_mss, accel_mss);
}

// Logs position controller state along the Down (vertical) axis to PSCD.
// Logs desired, target, and actual values for position [m], velocity [m/s], and acceleration [m/s²].
void AC_PosControl::Write_PSCD(float pos_desired_m, float pos_target_m, float pos_m, float vel_desired_ms, float vel_target_ms, float vel_ms, float accel_desired_mss, float accel_target_mss, float accel_mss)
{
    Write_PSCx(LOG_PSCD_MSG, pos_desired_m, pos_target_m, pos_m, vel_desired_ms, vel_target_ms, vel_ms, accel_desired_mss, accel_target_mss, accel_mss);
}

// Internal log writer for PSOx (North, East, Down tracking).
// Reduces duplication between Write_PSON, PSOE, and PSOD.
// Used for logging desired/target/actual position, velocity, and acceleration per axis.
void AC_PosControl::Write_PSOx(LogMessages id, float pos_target_offset_m, float pos_offset_m,
                               float vel_target_offset_ms, float vel_offset_ms,
                               float accel_target_offset_mss, float accel_offset_mss)
{
    const struct log_PSOx pkt{
        LOG_PACKET_HEADER_INIT(id),
            time_us             : AP_HAL::micros64(),
            pos_target_offset   : pos_target_offset_m,
            pos_offset          : pos_offset_m,
            vel_target_offset   : vel_target_offset_ms,
            vel_offset          : vel_offset_ms,
            accel_target_offset : accel_target_offset_mss,
            accel_offset        : accel_offset_mss,
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// Logs offset tracking along the North axis to PSON.
// Logs target and actual offset for position [m], velocity [m/s], and acceleration [m/s²].
void AC_PosControl::Write_PSON(float pos_target_offset_m, float pos_offset_m,
                               float vel_target_offset_ms, float vel_offset_ms,
                               float accel_target_offset_mss, float accel_offset_mss)
{
    Write_PSOx(LOG_PSON_MSG, pos_target_offset_m, pos_offset_m, vel_target_offset_ms, vel_offset_ms, accel_target_offset_mss, accel_offset_mss);
}

// Logs offset tracking along the East axis to PSOE.
// Logs target and actual offset for position [m], velocity [m/s], and acceleration [m/s²].
void AC_PosControl::Write_PSOE(float pos_target_offset_m, float pos_offset_m,
                               float vel_target_offset_ms, float vel_offset_ms,
                               float accel_target_offset_mss, float accel_offset_mss)
{
    Write_PSOx(LOG_PSOE_MSG, pos_target_offset_m, pos_offset_m, vel_target_offset_ms, vel_offset_ms, accel_target_offset_mss, accel_offset_mss);
}

// Logs offset tracking along the Down axis to PSOD.
// Logs target and actual offset for position [m], velocity [m/s], and acceleration [m/s²].
void AC_PosControl::Write_PSOD(float pos_target_offset_m, float pos_offset_m,
                               float vel_target_offset_ms, float vel_offset_ms,
                               float accel_target_offset_mss, float accel_offset_mss)
{
    Write_PSOx(LOG_PSOD_MSG, pos_target_offset_m, pos_offset_m, vel_target_offset_ms, vel_offset_ms, accel_target_offset_mss, accel_offset_mss);
}

// Logs terrain-following offset tracking along the Down axis to PSOT.
// Logs target and actual offset for position [m], velocity [m/s], and acceleration [m/s²].
void AC_PosControl::Write_PSOT(float pos_target_offset_m, float pos_offset_m,
                               float vel_target_offset_ms, float vel_offset_ms,
                               float accel_target_offset_mss, float accel_offset_mss)
{
    Write_PSOx(LOG_PSOT_MSG, pos_target_offset_m, pos_offset_m, vel_target_offset_ms, vel_offset_ms, accel_target_offset_mss, accel_offset_mss);
}

#endif  // HAL_LOGGING_ENABLED
