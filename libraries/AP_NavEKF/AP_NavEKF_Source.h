#pragma once

#include <AP_Param/AP_Param.h>

#define AP_NAKEKF_SOURCE_SET_MAX 3  // three sets of sources

class AP_NavEKF_Source
{

public:
    // Constructor
    AP_NavEKF_Source();

    /* Do not allow copies */
    AP_NavEKF_Source(const AP_NavEKF_Source &other) = delete;
    AP_NavEKF_Source &operator=(const AP_NavEKF_Source&) = delete;

    enum class SourceXY : uint8_t {
        NONE = 0,
        // BARO = 1 (not applicable)
        // RANGEFINDER = 2 (not applicable)
        GPS = 3,
        BEACON = 4,
        OPTFLOW = 5,
        EXTNAV = 6,
        WHEEL_ENCODER = 7
    };

    enum class SourceZ : uint8_t {
        NONE = 0,
        BARO = 1,
        RANGEFINDER = 2,
        GPS = 3,
        BEACON = 4,
        // OPTFLOW = 5 (not applicable, optical flow can be used for terrain alt but not relative or absolute alt)
        EXTNAV = 6
        // WHEEL_ENCODER = 7 (not applicable)
    };

    enum class SourceYaw : uint8_t {
        NONE = 0,
        COMPASS = 1,
        GPS = 2,
        GPS_COMPASS_FALLBACK = 3,
        EXTNAV = 6,
        GSF = 8
    };

    // enum for OPTIONS parameter
    enum class SourceOptions {
        FUSE_ALL_VELOCITIES = (1 << 0)  // fuse all velocities configured in source sets
    };

    // initialisation
    void init();

    // get current position source
    SourceXY getPosXYSource() const { return _source_set[active_source_set].posxy; }
    SourceZ getPosZSource() const { return _source_set[active_source_set].posz; }

    // set position, velocity and yaw sources to either 0=primary, 1=secondary, 2=tertiary
    void setPosVelYawSourceSet(uint8_t source_set_idx);
    uint8_t getPosVelYawSourceSet() const { return active_source_set; }

    // get/set velocity source
    SourceXY getVelXYSource() const { return _source_set[active_source_set].velxy; }
    SourceZ getVelZSource() const { return _source_set[active_source_set].velz; }

    // true/false of whether velocity source should be used
    bool useVelXYSource(SourceXY velxy_source) const;
    bool useVelZSource(SourceZ velz_source) const;

    // true if a velocity source is configured
    bool haveVelZSource() const;

    // get yaw source
    SourceYaw getYawSource() const;

    // align position of inactive sources to ahrs
    void align_inactive_sources();

    // sensor-specific helper functions

    // true if any source is GPS
    bool usingGPS() const;

    // true if source parameters have been configured (used for parameter conversion)
    bool configured();

    // mark parameters as configured (used to ensure parameter conversion is only done once)
    void mark_configured();

    // returns false if we fail arming checks, in which case the buffer will be populated with a failure message
    // requires_position should be true if horizontal position configuration should be checked
    bool pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const;

    // return true if ext nav is enabled on any source
    bool ext_nav_enabled(void) const;

    // return true if GPS yaw is enabled on any source
    bool gps_yaw_enabled(void) const;

    // return true if wheel encoder is enabled on any source
    bool wheel_encoder_enabled(void) const;

    static const struct AP_Param::GroupInfo var_info[];

private:

    // Parameters
    struct SourceSet {
        AP_Enum<SourceXY>  posxy;  // xy position source
        AP_Enum<SourceXY>  velxy;  // xy velocity source
        AP_Enum<SourceZ>   posz;   // position z (aka altitude or height) source
        AP_Enum<SourceZ>   velz;   // velocity z source
        AP_Enum<SourceYaw> yaw;    // yaw source
    } _source_set[AP_NAKEKF_SOURCE_SET_MAX];

    AP_Int16 _options;      // source options bitmask

    uint8_t active_source_set; // index of active source set
    bool _configured; // true once configured has returned true
};
