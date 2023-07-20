#pragma once

#if AP_UROS_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Scheduler.h>
#include <AP_HAL/Semaphores.h>
#include <AP_AHRS/AP_AHRS.h>

#include "fcntl.h"

#include <AP_Param/AP_Param.h>

extern const AP_HAL::HAL& hal;

class AP_UROS_Client
{
private:

    AP_Int8 enabled;

    HAL_Semaphore csem;

public:
    bool start(void);
    void main_loop(void);

    //! @brief Initialize the client.
    //! @return True on successful initialization, false on failure.
    bool init() WARN_IF_UNUSED;

    //! @brief Set up the client.
    //! @return True on successful creation, false on failure
    bool create() WARN_IF_UNUSED;

    //! @brief Update the client.
    void update();

    //! @brief Parameter storage.
    static const struct AP_Param::GroupInfo var_info[];
};

#endif // AP_UROS_ENABLED
