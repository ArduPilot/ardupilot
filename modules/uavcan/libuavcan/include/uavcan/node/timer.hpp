/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_NODE_TIMER_HPP_INCLUDED
#define UAVCAN_NODE_TIMER_HPP_INCLUDED

#include <uavcan/std.hpp>
#include <uavcan/error.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/util/linked_list.hpp>
#include <uavcan/node/scheduler.hpp>
#include <uavcan/node/abstract_node.hpp>
#include <uavcan/util/templates.hpp>

#if !defined(UAVCAN_CPP11) || !defined(UAVCAN_CPP_VERSION)
# error UAVCAN_CPP_VERSION
#endif

#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
# include <functional>
#endif

namespace uavcan
{

class UAVCAN_EXPORT TimerBase;

/**
 * Objects of this type will be supplied into timer callbacks.
 */
struct UAVCAN_EXPORT TimerEvent
{
    MonotonicTime scheduled_time;  ///< Time when the timer callback was expected to be invoked
    MonotonicTime real_time;       ///< True time when the timer callback was invoked

    TimerEvent(MonotonicTime arg_scheduled_time, MonotonicTime arg_real_time)
        : scheduled_time(arg_scheduled_time)
        , real_time(arg_real_time)
    { }
};

/**
 * Inherit this class if you need a timer callback method in your class.
 */
class UAVCAN_EXPORT TimerBase : private DeadlineHandler
{
    MonotonicDuration period_;

    virtual void handleDeadline(MonotonicTime current) override;

public:
    using DeadlineHandler::stop;
    using DeadlineHandler::isRunning;
    using DeadlineHandler::getDeadline;
    using DeadlineHandler::getScheduler;

    explicit TimerBase(INode& node)
        : DeadlineHandler(node.getScheduler())
        , period_(MonotonicDuration::getInfinite())
    { }

    /**
     * Various ways to start the timer - periodically or once.
     * If it is running already, it will be restarted.
     * If the deadline is in the past, the event will fire immediately.
     * In periodic mode the timer does not accumulate error over time.
     */
    void startOneShotWithDeadline(MonotonicTime deadline);
    void startOneShotWithDelay(MonotonicDuration delay);
    void startPeriodic(MonotonicDuration period);

    /**
     * Returns period if the timer is in periodic mode.
     * Returns infinite duration if the timer is in one-shot mode or stopped.
     */
    MonotonicDuration getPeriod() const { return period_; }

    /**
     * Implement this method in your class to receive callbacks.
     */
    virtual void handleTimerEvent(const TimerEvent& event) = 0;
};

/**
 * Wrapper over TimerBase that forwards callbacks into arbitrary handlers, like
 * functor objects, member functions or static functions.
 *
 * Callback must be set before the first event; otherwise the event will generate a fatal error.
 *
 * Also take a look at @ref MethodBinder<>, which may come useful if C++11 features are not available.
 *
 * @tparam Callback_    Callback type. Shall accept const reference to TimerEvent as its argument.
 */
template <typename Callback_>
class UAVCAN_EXPORT TimerEventForwarder : public TimerBase
{
public:
    typedef Callback_ Callback;

private:
    Callback callback_;

    virtual void handleTimerEvent(const TimerEvent& event)
    {
        if (coerceOrFallback<bool>(callback_, true))
        {
            callback_(event);
        }
        else
        {
            handleFatalError("Invalid timer callback");
        }
    }

public:
    explicit TimerEventForwarder(INode& node)
        : TimerBase(node)
        , callback_()
    { }

    TimerEventForwarder(INode& node, const Callback& callback)
        : TimerBase(node)
        , callback_(callback)
    { }

    /**
     * Get/set the callback object.
     * Callback must be set before the first event happens; otherwise the event will generate a fatal error.
     */
    const Callback& getCallback() const { return callback_; }
    void setCallback(const Callback& callback) { callback_ = callback; }
};


#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11

/**
 * Use this timer in C++11 mode.
 * Callback type is std::function<>.
 */
typedef TimerEventForwarder<std::function<void (const TimerEvent& event)> > Timer;

#endif

}

#endif // UAVCAN_NODE_TIMER_HPP_INCLUDED
