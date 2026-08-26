#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <stdint.h>

/*
  basic per-vehicle "swarm" telemetry, shared between SITL instances via
  AP_SITL_SharedMem's per-instance payload block (see SITL_SharedMem.h).

  Populated by Aircraft::sync_frame_time() from the vehicle's own
  simulated state, and readable by any other instance via
  AP_SITL_SharedMem::read_payload(). Intended for basic swarm-style
  awareness (e.g. simple avoidance/formation experiments) - it is not a
  replacement for the existing MAVLink/ADSB inter-vehicle links.

  Deliberately a flat POD struct of fixed-width types, since it crosses
  process boundaries via shared memory.
*/
struct AP_SITL_SwarmInfo {
    uint8_t  sysid;          // MAVLink system id of the publishing vehicle
    uint64_t sim_time_us;    // simulated time this info was captured
    int32_t  lat;            // degrees * 1e7
    int32_t  lng;            // degrees * 1e7
    int32_t  alt_cm;         // altitude AMSL, cm (Location::alt convention)
    float    vx, vy, vz;     // velocity, m/s, NED (earth frame)
    float    heading_deg;    // yaw/heading, degrees, 0..360
};

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
