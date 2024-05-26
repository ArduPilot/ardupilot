/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_PANIC_LISTENER_HPP_INCLUDED
#define UAVCAN_PROTOCOL_PANIC_LISTENER_HPP_INCLUDED

#include <cassert>
#include <uavcan/debug.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/node/subscriber.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/protocol/Panic.hpp>

#if !defined(UAVCAN_CPP_VERSION) || !defined(UAVCAN_CPP11)
# error UAVCAN_CPP_VERSION
#endif

#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
# include <functional>
#endif

namespace uavcan
{
/**
 * This class implements proper panic detector.
 * Refer to uavcan.protocol.Panic for details.
 * The listener can be stopped from the callback.
 *
 * @tparam Callback     Possible callback prototypes:
 *                      void (const ReceivedDataStructure<protocol::Panic>&)
 *                      void (const protocol::Panic&)
 */
template <
#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
          typename Callback = std::function<void (const ReceivedDataStructure<protocol::Panic>&)>
#else
          typename Callback = void (*)(const ReceivedDataStructure<protocol::Panic>&)
#endif
          >
class UAVCAN_EXPORT PanicListener : Noncopyable
{
    typedef MethodBinder<PanicListener*, void (PanicListener::*)(const ReceivedDataStructure<protocol::Panic>&)>
        PanicMsgCallback;

    Subscriber<protocol::Panic, PanicMsgCallback> sub_;
    MonotonicTime prev_msg_timestamp_;
    Callback callback_;
    uint8_t num_subsequent_msgs_;

    void invokeCallback(const ReceivedDataStructure<protocol::Panic>& msg)
    {
        if (coerceOrFallback<bool>(callback_, true))
        {
            callback_(msg);
        }
        else
        {
            UAVCAN_ASSERT(0);       // This is a logic error because normally we shouldn't start with an invalid callback
            sub_.getNode().registerInternalFailure("PanicListener invalid callback");
        }
    }

    void handleMsg(const ReceivedDataStructure<protocol::Panic>& msg)
    {
        UAVCAN_TRACE("PanicListener", "Received panic from snid=%i reason=%s",
                     int(msg.getSrcNodeID().get()), msg.reason_text.c_str());
        if (prev_msg_timestamp_.isZero())
        {
            num_subsequent_msgs_ = 1;
            prev_msg_timestamp_ = msg.getMonotonicTimestamp();
        }
        else
        {
            const MonotonicDuration diff = msg.getMonotonicTimestamp() - prev_msg_timestamp_;
            UAVCAN_ASSERT(diff.isPositive() || diff.isZero());
            if (diff.toMSec() > protocol::Panic::MAX_INTERVAL_MS)
            {
                num_subsequent_msgs_ = 1;
            }
            else
            {
                num_subsequent_msgs_++;
            }
            prev_msg_timestamp_ = msg.getMonotonicTimestamp();
            if (num_subsequent_msgs_ >= protocol::Panic::MIN_MESSAGES)
            {
                num_subsequent_msgs_ = protocol::Panic::MIN_MESSAGES;
                invokeCallback(msg);                      // The application can stop us from the callback
            }
        }
    }

public:
    explicit PanicListener(INode& node)
        : sub_(node)
        , callback_()
        , num_subsequent_msgs_(0)
    { }

    /**
     * Start the listener.
     * Once started it does not require further attention.
     * Returns negative error code.
     */
    int start(const Callback& callback)
    {
        stop();
        if (!coerceOrFallback<bool>(callback, true))
        {
            UAVCAN_TRACE("PanicListener", "Invalid callback");
            return -ErrInvalidParam;
        }
        callback_ = callback;
        return sub_.start(PanicMsgCallback(this, &PanicListener::handleMsg));
    }

    void stop()
    {
        sub_.stop();
        num_subsequent_msgs_ = 0;
        prev_msg_timestamp_ = MonotonicTime();
    }
};

}

#endif // UAVCAN_PROTOCOL_PANIC_LISTENER_HPP_INCLUDED
