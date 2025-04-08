#include <AP_HAL/AP_HAL.h> // to include SEMAPHORE

namespace Canard {
typedef ::HAL_Semaphore Semaphore;
}

#define CANARD_MALLOC malloc
#define CANARD_FREE free
