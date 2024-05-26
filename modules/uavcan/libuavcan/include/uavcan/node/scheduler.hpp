/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_NODE_SCHEDULER_HPP_INCLUDED
#define UAVCAN_NODE_SCHEDULER_HPP_INCLUDED

#include <uavcan/error.hpp>
#include <uavcan/util/linked_list.hpp>
#include <uavcan/transport/dispatcher.hpp>

namespace uavcan
{

class UAVCAN_EXPORT Scheduler;

class UAVCAN_EXPORT DeadlineHandler : public LinkedListNode<DeadlineHandler>
{
    MonotonicTime deadline_;

protected:
    Scheduler& scheduler_;

    explicit DeadlineHandler(Scheduler& scheduler)
        : scheduler_(scheduler)
    { }

    virtual ~DeadlineHandler() { stop(); }

public:
    virtual void handleDeadline(MonotonicTime current) = 0;

    void startWithDeadline(MonotonicTime deadline);
    void startWithDelay(MonotonicDuration delay);
    void generateDeadlineImmediately() { startWithDeadline(MonotonicTime::fromUSec(1)); }

    void stop();

    bool isRunning() const;

    MonotonicTime getDeadline() const { return deadline_; }
    Scheduler& getScheduler() const { return scheduler_; }
};


class UAVCAN_EXPORT DeadlineScheduler : Noncopyable
{
    LinkedListRoot<DeadlineHandler> handlers_;  // Ordered by deadline, lowest first

public:
    void add(DeadlineHandler* mdh);
    void remove(DeadlineHandler* mdh);
    bool doesExist(const DeadlineHandler* mdh) const;
    unsigned getNumHandlers() const { return handlers_.getLength(); }

    MonotonicTime pollAndGetMonotonicTime(ISystemClock& sysclock);
    MonotonicTime getEarliestDeadline() const;
};

/**
 * This class distributes processing time between library components (IO handling, deadline callbacks, ...).
 */
class UAVCAN_EXPORT Scheduler : Noncopyable
{
    enum { DefaultDeadlineResolutionMs = 5 };
    enum { MinDeadlineResolutionMs = 1 };
    enum { MaxDeadlineResolutionMs = 100 };

    enum { DefaultCleanupPeriodMs = 1000 };
    enum { MinCleanupPeriodMs = 10 };
    enum { MaxCleanupPeriodMs = 10000 };

    DeadlineScheduler deadline_scheduler_;
    Dispatcher dispatcher_;
    MonotonicTime prev_cleanup_ts_;
    MonotonicDuration deadline_resolution_;
    MonotonicDuration cleanup_period_;
    bool inside_spin_;

    struct InsideSpinSetter
    {
        Scheduler& owner;
        InsideSpinSetter(Scheduler& o)
            : owner(o)
        {
            owner.inside_spin_ = true;
        }
        ~InsideSpinSetter() { owner.inside_spin_ = false; }
    };

    MonotonicTime computeDispatcherSpinDeadline(MonotonicTime spin_deadline) const;
    void pollCleanup(MonotonicTime mono_ts, uint32_t num_frames_processed_with_last_spin);

public:
    Scheduler(ICanDriver& can_driver, IPoolAllocator& allocator, ISystemClock& sysclock)
        : dispatcher_(can_driver, allocator, sysclock)
        , prev_cleanup_ts_(sysclock.getMonotonic())
        , deadline_resolution_(MonotonicDuration::fromMSec(DefaultDeadlineResolutionMs))
        , cleanup_period_(MonotonicDuration::fromMSec(DefaultCleanupPeriodMs))
        , inside_spin_(false)
    { }

    /**
     * Spin until the deadline, or until some error occurs.
     * This function will return strictly when the deadline is reached, even if there are unprocessed frames.
     * Returns negative error code.
     */
    int spin(MonotonicTime deadline);

    /**
     * Non-blocking version of @ref spin() - spins until all pending frames and events are processed,
     * or until some error occurs. If there's nothing to do, returns immediately.
     * Returns negative error code.
     */
    int spinOnce();

    DeadlineScheduler& getDeadlineScheduler() { return deadline_scheduler_; }

    Dispatcher& getDispatcher()             { return dispatcher_; }
    const Dispatcher& getDispatcher() const { return dispatcher_; }

    ISystemClock& getSystemClock()         { return dispatcher_.getSystemClock(); }
    MonotonicTime getMonotonicTime() const { return dispatcher_.getSystemClock().getMonotonic(); }
    UtcTime getUtcTime()             const { return dispatcher_.getSystemClock().getUtc(); }

    /**
     * Worst case deadline callback resolution.
     * Higher resolution increases CPU usage.
     */
    MonotonicDuration getDeadlineResolution() const { return deadline_resolution_; }
    void setDeadlineResolution(MonotonicDuration res)
    {
        res = min(res, MonotonicDuration::fromMSec(MaxDeadlineResolutionMs));
        res = max(res, MonotonicDuration::fromMSec(MinDeadlineResolutionMs));
        deadline_resolution_ = res;
    }

    /**
     * How often the scheduler will run cleanup (listeners, outgoing transfer registry, ...).
     * Cleanup execution time grows linearly with number of listeners and number of items
     * in the Outgoing Transfer ID registry.
     * Lower period increases CPU usage.
     */
    MonotonicDuration getCleanupPeriod() const { return cleanup_period_; }
    void setCleanupPeriod(MonotonicDuration period)
    {
        period = min(period, MonotonicDuration::fromMSec(MaxCleanupPeriodMs));
        period = max(period, MonotonicDuration::fromMSec(MinCleanupPeriodMs));
        cleanup_period_ = period;
    }
};

}

#endif // UAVCAN_NODE_SCHEDULER_HPP_INCLUDED
