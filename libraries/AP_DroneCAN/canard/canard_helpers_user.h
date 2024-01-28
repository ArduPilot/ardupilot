#include <AP_HAL/AP_HAL.h> // to include SEMAPHORE

#ifndef HAL_BUILD_AP_PERIPH
namespace Canard {
typedef ::HAL_Semaphore Semaphore;
}
#define CANARD_MUTEX_ENABLED
#endif

#define CANARD_MALLOC malloc
#define CANARD_FREE free
