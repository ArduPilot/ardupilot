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

bool AP_HAL::EventHandle::wait(uint16_t duration_us)
{
    if (evt_src_ == nullptr) {
        return false;
    }
    return evt_src_->wait(duration_us, this);
}

bool AP_HAL::EventHandle::set_source(AP_HAL::EventSource* src)
{
    WITH_SEMAPHORE(sem);
    evt_src_ = src;
    evt_mask_ = 0;
    return true;
}
