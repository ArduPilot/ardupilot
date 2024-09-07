#include "AC_ForceTorque.h"
#include "AC_ForceTorque_DR304_Serial.h"
#include "AC_ForceTorque_2DR304_Serial.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL &hal;

// table of user settable parameters
//参数注册
const AP_Param::GroupInfo ForceTorque::var_info[] = {

	// @Group: 1_
	// @Path: AC_ForceTorque_Params.cpp
	AP_SUBGROUPINFO(params[0], "1_", 25, ForceTorque, AC_ForceTorque_Params),

    // @Group: 1_
    // @Path: AC_ForceTorque_Params.cpp
    AP_SUBGROUPVARPTR(drivers[0], "1_",  57, ForceTorque, backend_var_info[0]),

#if FORCETORQUE_MAX_INSTANCES > 1
    // @Group: 2_
    // @Path: AC_ForceTorque_Params.cpp
    AP_SUBGROUPINFO(params[1], "2_", 27, ForceTorque, AC_ForceTorque_Params),

    // @Group: 2_
    // @Path: AC_ForceTorque_Params.cpp
    AP_SUBGROUPVARPTR(drivers[1], "2_",  58, ForceTorque, backend_var_info[1]),
#endif
  
    AP_GROUPEND
};

const AP_Param::GroupInfo *ForceTorque::backend_var_info[FORCETORQUE_MAX_INSTANCES];

ForceTorque::ForceTorque()
{
    AP_Param::setup_object_defaults(this, var_info);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("ForceTorque must be singleton");
    }
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _singleton = this;
}

/*
  initialise the ForceTorque class. We do detection of attached 
  ForceTorques here. For now we won't allow for hot-plugging of
  ForceTorques.
 */
void ForceTorque::init(InstallLocation location_default)
{
    if (init_done) {
        // init called a 2nd time
        return;
    }
    init_done = true;

    // set orientation defaults
    for (uint8_t i=0; i<FORCETORQUE_MAX_INSTANCES; i++) {
        params[i].location.set_default(location_default);
    }

    for (uint8_t i=0, serial_instance = 0; i<FORCETORQUE_MAX_INSTANCES; i++) {
        // serial_instance will be increased inside detect_instance
        // if a serial driver is loaded for this instance
        WITH_SEMAPHORE(detect_sem);
        detect_instance(i, serial_instance);
        
        // initialise status
        state[i].status = Status::NotConnected;
        state[i].range_valid_count = 0;
    }
}

/*
  update ForceTorque state for all instances. This should be called at
  around 10Hz by main loop
 */
void ForceTorque::update(void)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != nullptr) {
            if ((Type)params[i].type.get() == Type::NONE) {
                // allow user to disable a ForceTorque at runtime
                state[i].status = Status::NotConnected;
                state[i].range_valid_count = 0;
                continue;
            }
           
            drivers[i]->update();
        }
    }

    if ((Type)params[0].type.get() == Type::Two_DR304_Serial || (Type)params[0].type.get() == Type::SIM) {
        for(uint8_t i=1; i<FORCETORQUE_MAX_INSTANCES; i++)
        {
            state[i] = state[0];
        }
    }

#if HAL_LOGGING_ENABLED
    Log_FRTQ();
#endif
}

bool ForceTorque::_add_backend(AC_ForceTorque_Backend *backend, uint8_t instance, uint8_t serial_instance)
{
    if (!backend) {
        return false;
    }
    if (instance >= FORCETORQUE_MAX_INSTANCES) {
        AP_HAL::panic("Too many ForceTorque backends");
    }
    if (drivers[instance] != nullptr) {
        // we've allocated the same instance twice
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }
    backend->init_serial(serial_instance);
    drivers[instance] = backend;
    num_instances = MAX(num_instances, instance+1);
    return true;
}

/*
  detect if an instance of a ForceTorque is connected. 
 */
void ForceTorque::detect_instance(uint8_t instance, uint8_t& serial_instance)
{
    const Type _type = (Type)params[instance].type.get();
    switch (_type) {

    case Type::DR304_Serial:
        if (AC_ForceTorque_DR304_Serial::detect(serial_instance)) {
            _add_backend(new AC_ForceTorque_DR304_Serial(state[instance], params[instance]), instance, serial_instance++);
        }
        break;
    case Type::Two_DR304_Serial:
        if (AC_ForceTorque_2DR304_Serial::detect(serial_instance)) {
            for(uint8_t i=instance; i<FORCETORQUE_MAX_INSTANCES; i++)
            {
                _add_backend(new AC_ForceTorque_2DR304_Serial(state[i], params[i]), i, serial_instance++);
            }
        }
        break;
    case Type::NONE:
    default:
        break;
    }

    // if the backend has some local parameters then make those available in the tree
    if (drivers[instance] && state[instance].var_info) {
        backend_var_info[instance] = state[instance].var_info;
        AP_Param::load_object_from_eeprom(drivers[instance], backend_var_info[instance]);

        // param count could have changed
        AP_Param::invalidate_count();
    }
}

AC_ForceTorque_Backend *ForceTorque::get_backend(uint8_t id) const {
    if (id >= num_instances) {
        return nullptr;
    }
    if (drivers[id] != nullptr) {
        if (drivers[id]->type() == Type::NONE) {
            // pretend it isn't here; disabled at runtime?
            return nullptr;
        }
    }
    return drivers[id];
};
//判断连接设备的状态
ForceTorque::Status ForceTorque::status_location(enum InstallLocation location) const
{
    AC_ForceTorque_Backend *backend = find_instance(location);
    if (backend == nullptr) {
        return Status::NotConnected;
    }
    return backend->status();
}

// return true if we have a ForceTorque with the specified install location
bool ForceTorque::has_location(enum InstallLocation location) const
{
    return (find_instance(location) != nullptr);
}

// find first ForceTorque instance with the specified install location
AC_ForceTorque_Backend *ForceTorque::find_instance(enum InstallLocation location) const
{
    // first try for a ForceTorque that is in the specified location
    for (uint8_t i=0; i<num_instances; i++) {
        AC_ForceTorque_Backend *backend = get_backend(i);
        if (backend != nullptr &&
            backend->location() == location &&
            backend->status() == Status::Good) {
            return backend;
        }
    }
    // if none in range then return first with correct location
    for (uint8_t i=0; i<num_instances; i++) {
        AC_ForceTorque_Backend *backend = get_backend(i);
        if (backend != nullptr &&
            backend->location() == location) {
            return backend;
        }
    }
    return nullptr;
}

float ForceTorque::force_x_N_location(enum InstallLocation location) const
{
    AC_ForceTorque_Backend *backend = find_instance(location);
    if (backend == nullptr) {
        return 0;
    }
    return backend->get_force_x_N(location);
}

float ForceTorque::force_y_N_location(enum InstallLocation location) const
{
    AC_ForceTorque_Backend *backend = find_instance(location);
    if (backend == nullptr) {
        return 0;
    }
    return backend->get_force_y_N(location);
}

float ForceTorque::force_z_N_location(enum InstallLocation location) const
{
    AC_ForceTorque_Backend *backend = find_instance(location);
    if (backend == nullptr) {
        return 0;
    }
    return backend->get_force_z_N(location);
}

float ForceTorque::torque_x_Nm_location(enum InstallLocation location) const
{
    AC_ForceTorque_Backend *backend = find_instance(location);
    if (backend == nullptr) {
        return 0;
    }
    return backend->get_torque_x_Nm(location);
}

float ForceTorque::torque_y_Nm_location(enum InstallLocation location) const
{
    AC_ForceTorque_Backend *backend = find_instance(location);
    if (backend == nullptr) {
        return 0;
    }
    return backend->get_torque_y_Nm(location);
}

float ForceTorque::torque_z_Nm_location(enum InstallLocation location) const
{
    AC_ForceTorque_Backend *backend = find_instance(location);
    if (backend == nullptr) {
        return 0;
    }
    return backend->get_torque_z_Nm(location);
}

bool ForceTorque::has_data_location(enum InstallLocation location) const
{
    AC_ForceTorque_Backend *backend = find_instance(location);
    if (backend == nullptr) {
        return false;
    }
    return backend->has_data();
}

uint32_t ForceTorque::last_reading_ms(enum InstallLocation location) const
{
    AC_ForceTorque_Backend *backend = find_instance(location);
    if (backend == nullptr) {
        return 0;
    }
    return backend->last_reading_ms();
}

//Write an FRTQ (ForceTorque) packet
//写日志
void ForceTorque::Log_FRTQ() const
{
    if (_log_frtq_bit == uint32_t(-1)) {
        return;
    }

    /** This comment can be canceled after the ground station modifies param:LOG_BITMASK   
        #define MASK_LOG_ICLI (1UL<<21)
        In ForceTorque.set_log_icli_bit(MASK_LOG_ICLI);  */

    // AP_Logger &logger = AP::logger();
    // if (!logger.should_log(_log_icli_bit)) {
    //     return;
    // }
    //循环遍历每一个串口,并将信息打印出来
    for (uint8_t i=0; i<FORCETORQUE_MAX_INSTANCES; i++) {
        const AC_ForceTorque_Backend *s = get_backend(i);
        if (s == nullptr) {
            continue;
        }
        InstallLocation loc=InstallLocation (s->location()+InstallLocation::Up_Rotor);
        const struct log_FRTQ pkt = {
                LOG_PACKET_HEADER_INIT(LOG_FRTQ_MSG),
                time_us         : AP_HAL::micros64(),
                instance        : i,
                force_x_N       : s->get_force_x_N(loc),
                force_y_N       : s->get_force_y_N(loc),
                force_z_N       : s->get_force_z_N(loc),
                torque_x_N      : s->get_torque_x_Nm(loc),
                torque_y_N      : s->get_torque_y_Nm(loc),
                torque_z_N      : s->get_torque_z_Nm(loc),
                status          : (uint8_t)s->status(),
                location        : s->location(),
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
        ////print device ID so as to solve two sensor data.
        //hal.console->printf("ForceTorque location of device ID:%d ",loc);
    }
}
//错误信息打印
bool ForceTorque::prearm_healthy(char *failure_msg, const uint8_t failure_msg_len) const
{
    for (uint8_t i = 0; i < FORCETORQUE_MAX_INSTANCES; i++) {
        if ((Type)params[i].type.get() == Type::NONE) {
            continue;
        }

        if (drivers[i] == nullptr) {
            hal.util->snprintf(failure_msg, failure_msg_len, "ForceTorque %X: Not Detected", i + 1);
            return false;
        }

        switch (drivers[i]->status()) {
        case Status::NoData:
            hal.util->snprintf(failure_msg, failure_msg_len, "ForceTorque %X: No Data", i + 1);
            return false;
        case Status::NotConnected:
            hal.util->snprintf(failure_msg, failure_msg_len, "ForceTorque %X: Not Connected", i + 1);
            return false;
        case Status::OutOfRangeLow:
        case Status::OutOfRangeHigh:
        case Status::Good:  
            break;
        }
    }

    return true;
}

ForceTorque *ForceTorque::_singleton;

namespace AP {

ForceTorque *forcetorque()
{
    return ForceTorque::get_singleton();
}

}