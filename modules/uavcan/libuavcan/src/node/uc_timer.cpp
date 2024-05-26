/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/node/timer.hpp>
#include <cassert>

namespace uavcan
{
/*
 * TimerBase
 */
void TimerBase::handleDeadline(MonotonicTime current)
{
    UAVCAN_ASSERT(!isRunning());

    const MonotonicTime scheduled_time = getDeadline();

    if (period_ < MonotonicDuration::getInfinite())
    {
        startWithDeadline(scheduled_time + period_);
    }

    // Application can re-register the timer with different params, it's OK
    handleTimerEvent(TimerEvent(scheduled_time, current));
}

void TimerBase::startOneShotWithDeadline(MonotonicTime deadline)
{
    stop();
    period_ = MonotonicDuration::getInfinite();
    DeadlineHandler::startWithDeadline(deadline);
}

void TimerBase::startOneShotWithDelay(MonotonicDuration delay)
{
    stop();
    period_ = MonotonicDuration::getInfinite();
    DeadlineHandler::startWithDelay(delay);
}

void TimerBase::startPeriodic(MonotonicDuration period)
{
    UAVCAN_ASSERT(period < MonotonicDuration::getInfinite());
    stop();
    period_ = period;
    DeadlineHandler::startWithDelay(period);
}

}
