#pragma once

#include "AP_HAL_Namespace.h"
#include <stdint.h>
#include "AP_HAL_Boards.h"

class AP_HAL::EventSource {
public:
    // generate event from thread context
    virtual void signal(uint32_t evt_mask) = 0;

    // generate event from interrupt context
    virtual void signalI(uint32_t evt_mask) { signal(evt_mask); }


    // Wait on an Event handle, method for internal use by EventHandle
    virtual bool wait(uint16_t duration_us, AP_HAL::EventHandle* evt_handle) = 0;
};

class AP_HAL::EventHandle {
public:
    //Set event source
    virtual bool set_source(AP_HAL::EventSource* src);

    AP_HAL::EventSource* get_source() { return evt_src_; }

    // return true if event type was successfully registered
    virtual bool register_event(uint32_t evt_mask);

    // return true if event type was successfully unregistered
    virtual bool unregister_event(uint32_t evt_mask);

    // return true if event was triggered within the duration
    virtual bool wait(uint16_t duration_us);

    virtual uint32_t get_evt_mask() const { return evt_mask_; }

private:
    // Mask of events to be handeled,
    // Max 32 events can be handled per event handle
    uint32_t evt_mask_;
    AP_HAL::EventSource *evt_src_;
    HAL_Semaphore sem;
};
