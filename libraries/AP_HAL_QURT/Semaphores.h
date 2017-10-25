#pragma once

#include "AP_HAL_QURT.h"
#include "AP_HAL/POSIXSemaphores.h"

namespace QURT {

class Semaphore : public AP_HAL::POSIXSemaphore {
};

}
