/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  this defines data structures for public module interfaces in
  ArduPilot. 

  These structures are designed to not depend on other headers inside
  ArduPilot, although they do depend on the general ABI of the
  platform, and thus can depend on compilation options to some extent
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define AHRS_state_version 1

enum AHRS_status {
    AHRS_STATUS_INITIALISING    = 0,
    AHRS_STATUS_UNHEALTHY       = 1,
    AHRS_STATUS_HEALTHY         = 2
};

/*
  export the attitude and position of the vehicle.
 */
struct AHRS_state {
    // version of this structure (AHRS_state_version)
    uint32_t structure_version;

    // time since boot in microseconds
    uint64_t time_us;
    
    // status of AHRS solution
    enum AHRS_status status;
    
    // quaternion attitude, first element is length scalar. Same
    // conventions as AP_Math/quaternion.h
    float quat[4];

    // euler angles in radians. Order is roll, pitch, yaw
    float eulers[3];
    
    // global origin
    struct {
        // true when origin has been initialised with a global position
        bool initialised;

        // latitude and longitude in degrees * 10^7 (approx 1cm resolution)
        int32_t latitude;
        int32_t longitude;

        // altitude AMSL in meters, positive up
        float altitude;
    } origin;

    // global position
    struct {
        // true when we have a global position
        bool available;

        // latitude and longitude in degrees * 10^7 (approx 1cm resolution)
        int32_t latitude;
        int32_t longitude;

        // altitude AMSL in meters, positive up
        float altitude;
    } position;
    
    // NED relative position in meters. Relative to origin
    float relative_position[3];

    // current rotational rates in radians/second in body frame
    // order is roll, pitch, yaw
    float gyro_rate[3];

    // current earth frame acceleration estimate, including
    // gravitational forces, m/s/s order is NED
    float accel_ef[3];
};


/*
  prototypes for hook functions
 */
typedef void (*hook_setup_start_fn_t)(uint64_t);
void hook_setup_start(uint64_t time_us);
    
typedef void (*hook_setup_complete_fn_t)(uint64_t);
void hook_setup_complete(uint64_t time_us);
    
typedef void (*hook_AHRS_update_fn_t)(const struct AHRS_state *);
void hook_AHRS_update(const struct AHRS_state *state);

#ifdef __cplusplus
}
#endif

