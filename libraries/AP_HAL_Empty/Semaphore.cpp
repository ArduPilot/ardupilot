
#include "Semaphore.h"

using namespace Empty;

EmptySemaphore::EmptySemaphore() :
    _owner(NULL),
    _k(NULL)
{}


bool EmptySemaphore::get(void* owner) {
    if (_owner == NULL) {
        _owner = owner;
        return true;
    } else {
        return false;
    }
}

bool EmptySemaphore::release(void* owner) {
    if (_owner == NULL || _owner != owner) {
        return false;
    } else {
        _owner = NULL;
        if (_k){
            _k();
            _k = NULL;
        }
        return true;
    }
}

bool EmptySemaphore::call_on_release(void* caller, AP_HAL::Proc k) {
    /* idk what semantics randy was looking for here, honestly.
     * seems like a bad idea. */
    _k = k;
    return true;
}

