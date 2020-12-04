#include "EventHandle.h"
#include <AP_HAL/AP_HAL.h>


bool AP_HAL::EventHandle::register_event(uint32_t evt_mask)
{
    WITH_SEMAPHORE(sem);
    evt_mask_ |= evt_mask;
    return true;
}

bool AP_HAL::EventHandle::unregister_event(uint32_t evt_mask)
{
    WITH_SEMAPHORE(sem);
    evt_mask_ &= ~evt_mask;
    return true;
}

bool AP_HAL::EventHandle::wait(uint64_t duration)
{
    if (evt_src_ == nullptr) {
        return false;
    }
    return evt_src_->wait(duration, this);
}

bool AP_HAL::EventHandle::set_source(AP_HAL::EventSource* src)
{
    WITH_SEMAPHORE(sem);
    evt_src_ = src;
    evt_mask_ = 0;
    return true;
}
