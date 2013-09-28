
#include "Semaphores.h"

using namespace Linux;

bool LinuxSemaphore::give() {
    if (_taken) {
        _taken = false;
        return true;
    } else {
        return false;
    }
}

bool LinuxSemaphore::take(uint32_t timeout_ms) {
    return take_nonblocking();
}

bool LinuxSemaphore::take_nonblocking() {
    /* we need pthread semaphores here */
    if (!_taken) {
        _taken = true;
        return true;
    }
    return false;
}
