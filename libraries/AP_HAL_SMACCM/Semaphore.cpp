
#include "Semaphore.h"

using namespace SMACCM;

SMACCMSemaphore::SMACCMSemaphore() :
    _owner(NULL),
    _k(NULL)
{}


bool SMACCMSemaphore::get(void* owner) {
    if (_owner == NULL) {
        _owner = owner;
        return true;
    } else {
        return false;
    }
}

bool SMACCMSemaphore::release(void* owner) {
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

bool SMACCMSemaphore::call_on_release(void* caller, AP_HAL::Proc k) {
    /* idk what semantics randy was looking for here, honestly.
     * seems like a bad idea. */
    _k = k;
    return true;
}

