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

    enum class SourceXY {
        NONE = 0,
        // BARO = 1 (not applicable)
        // RANGEFINDER = 2 (not applicable)
        GPS = 3,
        BEACON = 4,
        OPTFLOW = 5,
        EXTNAV = 6,
        WHEEL_ENCODER = 7
    };

    enum class SourceZ {
        NONE = 0,
        BARO = 1,
        RANGEFINDER = 2,
        GPS = 3,
        BEACON = 4,
        // OPTFLOW = 5 (not applicable, optical flow can be used for terrain alt but not relative or absolute alt)
        EXTNAV = 6
        // WHEEL_ENCODER = 7 (not applicable)
    };

    enum class SourceYaw {
        NONE = 0,
        COMPASS = 1,
        EXTERNAL = 2,
        EXTERNAL_COMPASS_FALLBACK = 3
    };

    // enum for OPTIONS parameter
    enum class SourceOptions {
        FUSE_ALL_VELOCITIES = (1 << 0)  // fuse all velocities configured in source sets
    };

    // initialisation
    void init();

    // get current position source
    SourceXY getPosXYSource() const { return _active_source_set.posxy; }
    SourceZ getPosZSource() const { return _active_source_set.posz; }

    // set position, velocity and yaw sources to either 0=primary, 1=secondary, 2=tertiary
    void setPosVelYawSourceSet(uint8_t source_set_idx);

    // get/set velocity source
    SourceXY getVelXYSource() const { return _active_source_set.velxy; }
    SourceZ getVelZSource() const { return _active_source_set.velz; }
    void setVelZSource(SourceZ source) { _active_source_set.velz = source; }

    // true/false of whether velocity source should be used
    bool useVelXYSource(SourceXY velxy_source) const;
    bool useVelZSource(SourceZ velz_source) const;

    // true if a velocity source is configured
    bool haveVelZSource() const;

    // get yaw source
    SourceYaw getYawSource() const { return _active_source_set.yaw; }

    // align position of inactive sources to ahrs
    void align_inactive_sources();

    // sensor-specific helper functions

    // true if any source is GPS
    bool usingGPS() const;

    // true if source parameters have been configured (used for parameter conversion)
    bool configured_in_storage();

    // mark parameters as configured in storage (used to ensure parameter conversion is only done once)
    void mark_configured_in_storage();

    // returns false if we fail arming checks, in which case the buffer will be populated with a failure message
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const;

    static const struct AP_Param::GroupInfo var_info[];

private:

    // Parameters
    struct {
        AP_Int8 posxy;  // xy position source
        AP_Int8 velxy;  // xy velocity source
        AP_Int8 posz;   // position z (aka altitude or height) source
        AP_Int8 velz;   // velocity z source
        AP_Int8 yaw;    // yaw source
    } _source_set[AP_NAKEKF_SOURCE_SET_MAX];

    AP_Int16 _options;      // source options bitmask

    // active sources
    struct {
        SourceXY posxy;     // current xy position source
        SourceZ posz;       // current z position source
        SourceXY velxy;     // current xy velocity source
        SourceZ velz;       // current z velocity source
        SourceYaw yaw;      // current yaw source
    } _active_source_set;
    bool initialised;       // true once init has been run
    bool config_in_storage; // true once configured in storage has returned true
};
