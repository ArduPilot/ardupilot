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

#define AHRS_state_version 3
#define gyro_sample_version 1
#define accel_sample_version 2

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

    // the current primary accel instance
    uint8_t primary_accel;

    // the current primary gyro instance
    uint8_t primary_gyro;
    
    // current gyro bias. This is relative to the gyro data in
    // gyro_sample for primary_gyro. It should be added to a gyro
    // sample to get the corrected gyro estimate
    float gyro_bias[3];

    // north-east-down velocity m/s
    float velocity_ned[3];
};


/*
  export corrected gyro samples at hardware sampling rate
 */
struct gyro_sample {
    // version of this structure (gyro_sample_version)
    uint32_t structure_version;

    // which gyro this is
    uint8_t instance;
    
    // time since boot in microseconds
    uint64_t time_us;

    // time associated with this sample (seconds)
    float delta_time;
    
    // body frame rates in radian/sec
    float gyro[3];
};

/*
  export corrected accel samples at hardware sampling rate
 */
struct accel_sample {
    // version of this structure (accel_sample_version)
    uint32_t structure_version;

    // which accel this is
    uint8_t instance;
    
    // time since boot in microseconds
    uint64_t time_us;

    // time associated with this sample (seconds)
    float delta_time;

    // body frame rates in m/s/s
    float accel[3];

    // true if external frame sync is set
    bool fsync_set;
};
    
/*
  prototypes for hook functions
 */
typedef void (*ap_hook_setup_start_fn_t)(uint64_t);
void ap_hook_setup_start(uint64_t time_us);
    
typedef void (*ap_hook_setup_complete_fn_t)(uint64_t);
void ap_hook_setup_complete(uint64_t time_us);
    
typedef void (*ap_hook_AHRS_update_fn_t)(const struct AHRS_state *);
void ap_hook_AHRS_update(const struct AHRS_state *state);

typedef void (*ap_hook_gyro_sample_fn_t)(const struct gyro_sample *);
void ap_hook_gyro_sample(const struct gyro_sample *state);

typedef void (*ap_hook_accel_sample_fn_t)(const struct accel_sample *);
void ap_hook_accel_sample(const struct accel_sample *state);
    
#ifdef __cplusplus
}
#endif

