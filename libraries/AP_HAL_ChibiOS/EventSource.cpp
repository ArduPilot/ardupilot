#include "EventSource.h"
#include <AP_Math/AP_Math.h>

using namespace ChibiOS;

#if CH_CFG_USE_EVENTS == TRUE

bool EventSource::wait(uint64_t duration, AP_HAL::EventHandle *evt_handle)
{
    chibios_rt::EventListener evt_listener;
    eventmask_t evt_mask = evt_handle->get_evt_mask();
    msg_t ret = msg_t();
    ch_evt_src_.registerMask(&evt_listener, evt_mask);
    if (duration == 0) {
        ret = chEvtWaitAnyTimeout(evt_mask, TIME_IMMEDIATE);
    } else {
        ret = chEvtWaitAnyTimeout(evt_mask, MAX(CH_CFG_ST_TIMEDELTA, chTimeUS2I(duration)));
    }
    ch_evt_src_.unregister(&evt_listener);
    return ret == MSG_OK;
}

void EventSource::signal(uint32_t evt_mask)
{
    ch_evt_src_.broadcastFlags(evt_mask);
}

__RAMFUNC__ void EventSource::signalI(uint32_t evt_mask)
{
    chSysLockFromISR();
    ch_evt_src_.broadcastFlagsI(evt_mask);
    chSysUnlockFromISR();
}
#endif //#if CH_CFG_USE_EVENTS == TRUE
