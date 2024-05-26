/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_NODE_SUBSCRIBER_HPP_INCLUDED
#define UAVCAN_NODE_SUBSCRIBER_HPP_INCLUDED

#include <cassert>
#include <uavcan/build_config.hpp>
#include <uavcan/node/generic_subscriber.hpp>

#if !defined(UAVCAN_CPP_VERSION) || !defined(UAVCAN_CPP11)
# error UAVCAN_CPP_VERSION
#endif

#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
# include <functional>
#endif

namespace uavcan
{
/**
 * Use this class to subscribe to a message.
 *
 * @tparam DataType_        Message data type.
 *
 * @tparam Callback_        Type of the callback that will be used to deliver received messages
 *                          into the application. Type of the argument of the callback can be either:
 *                          - DataType_&
 *                          - const DataType_&
 *                          - ReceivedDataStructure<DataType_>&
 *                          - const ReceivedDataStructure<DataType_>&
 *                          For the first two options, @ref ReceivedDataStructure<> will be casted implicitly.
 *                          In C++11 mode this type defaults to std::function<>.
 *                          In C++03 mode this type defaults to a plain function pointer; use binder to
 *                          call member functions as callbacks.
 */
template <typename DataType_,
#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
          typename Callback_ = std::function<void (const ReceivedDataStructure<DataType_>&)>
#else
          typename Callback_ = void (*)(const ReceivedDataStructure<DataType_>&)
#endif
          >
class UAVCAN_EXPORT Subscriber
    : public GenericSubscriber<DataType_, DataType_, TransferListener>
{
public:
    typedef Callback_ Callback;

private:
    typedef GenericSubscriber<DataType_, DataType_, TransferListener> BaseType;

    Callback callback_;

    virtual void handleReceivedDataStruct(ReceivedDataStructure<DataType_>& msg) override
    {
        if (coerceOrFallback<bool>(callback_, true))
        {
            callback_(msg);
        }
        else
        {
            handleFatalError("Sub clbk");
        }
    }

public:
    typedef DataType_ DataType;

    explicit Subscriber(INode& node)
        : BaseType(node)
        , callback_()
    {
        StaticAssert<DataTypeKind(DataType::DataTypeKind) == DataTypeKindMessage>::check();
    }

    /**
     * Begin receiving messages.
     * Each message will be passed to the application via the callback.
     * Returns negative error code.
     */
    int start(const Callback& callback)
    {
        stop();

        if (!coerceOrFallback<bool>(callback, true))
        {
            UAVCAN_TRACE("Subscriber", "Invalid callback");
            return -ErrInvalidParam;
        }
        callback_ = callback;

        return BaseType::startAsMessageListener();
    }

    using BaseType::allowAnonymousTransfers;
    using BaseType::stop;
    using BaseType::getFailureCount;
};

}

#endif // UAVCAN_NODE_SUBSCRIBER_HPP_INCLUDED
