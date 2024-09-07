#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include "AC_ForceTorque_Params.h"

// Maximum number of ForceTorque sensor instances available on this platform
#ifndef FORCETORQUE_MAX_INSTANCES
#define FORCETORQUE_MAX_INSTANCES 2
#endif

class AC_ForceTorque_Backend;

class ForceTorque
{
    friend class AC_ForceTorque_Backend;

public:
    ForceTorque();
    // /*新加6个变量，便于读取*/
    // Vector3f reading_force_N2;
    // Vector3f reading_torque_Nm2;
    /* Do not allow copies */
    ForceTorque(const ForceTorque &other) = delete;
    ForceTorque &operator=(const ForceTorque&) = delete;

    // ForceTorque driver types
    enum class Type {
        NONE   = 0,
        DR304_Serial = 1,
        Two_DR304_Serial=2,
        SIM = 100,
    };

    enum class Status {
        NotConnected = 0,
        NoData,
        OutOfRangeLow,
        OutOfRangeHigh,
        Good
    };

    // The ForceTorque_State structure is filled in by the backend driver
    struct ForceTorque_State {
        Vector3f force_N;       //force_N.x/.y/.z denote force in x/y/z axis
        Vector3f torque_Nm;     //torque_Nm.x/.y/.z denote torque in x/y/z axis
        Vector3f force_N2;       //force_N.x2/.y2/.z2 denote force in x/y/z axis
        Vector3f torque_Nm2;     //torque_Nm.x2/.y2/.z2 denote torque in x/y/z axis
        // float force_x;      // force in x axis
        // float force_y;      // force in y axis
        // float force_z;      // force in z axis 
        // float torque_x;     // torque in x axis
        // float torque_y;     // torque in y axis
        // float torque_z;     // torque in z axis  
        enum ForceTorque::Status status; // sensor status
        uint8_t  range_valid_count;     // number of consecutive valid readings (maxes out at 2)
        uint32_t last_reading_ms;       // system time of last successful update from sensor

        const struct AP_Param::GroupInfo *var_info;
    };

    static const struct AP_Param::GroupInfo *backend_var_info[FORCETORQUE_MAX_INSTANCES];

    // parameters for each instance
    static const struct AP_Param::GroupInfo var_info[];

    void set_log_frtq_bit(uint32_t log_frtq_bit) { _log_frtq_bit = log_frtq_bit; }


    //Return the number of ForceTorque instances. 
    uint8_t num_sensors(void) const {
        return num_instances;
    }

    // prearm checks
    bool prearm_healthy(char *failure_msg, const uint8_t failure_msg_len) const;

    // detect and initialise any available ForceTorques
    void init(InstallLocation location);

    // update state of all ForceTorques. Should be called at around
    // 100Hz from main loop
    void update(void);

    // return true if we have a ForceTorque with the specified install location
    bool has_location(enum InstallLocation location) const;

    // find ForceTorque instance with the specified installation location
    AC_ForceTorque_Backend *find_instance(enum InstallLocation location) const;

    AC_ForceTorque_Backend *get_backend(uint8_t id) const;

    // get ForceTorque type for an ID
    Type get_type(uint8_t id) const {
        return id >= FORCETORQUE_MAX_INSTANCES? Type::NONE : Type(params[id].type.get());
    }

    // methods to return force and torque on a particular installation location from
    // any sensor which can current supply it
    float force_x_N_location(enum InstallLocation location) const;
    float force_y_N_location(enum InstallLocation location) const;
    float force_z_N_location(enum InstallLocation location) const;
    float torque_x_Nm_location(enum InstallLocation location) const;
    float torque_y_Nm_location(enum InstallLocation location) const;
    float torque_z_Nm_location(enum InstallLocation location) const;
    ForceTorque::Status status_location(enum InstallLocation location) const;
    bool has_data_location(enum InstallLocation location) const;
    uint32_t last_reading_ms(enum InstallLocation location) const;


    static ForceTorque *get_singleton(void) { return _singleton; }

protected:
    AC_ForceTorque_Params params[FORCETORQUE_MAX_INSTANCES];

private:
    static ForceTorque *_singleton;

    ForceTorque_State state[FORCETORQUE_MAX_INSTANCES];
    AC_ForceTorque_Backend *drivers[FORCETORQUE_MAX_INSTANCES];
    uint8_t num_instances;
    bool init_done;
    HAL_Semaphore detect_sem;

    void detect_instance(uint8_t instance, uint8_t& serial_instance);

    bool _add_backend(AC_ForceTorque_Backend *driver, uint8_t instance, uint8_t serial_instance=0);

    uint32_t _log_frtq_bit = -1;
    void Log_FRTQ() const;  //LOG for FRoce and TorQue
};

namespace AP {
    ForceTorque *forcetorque();
};