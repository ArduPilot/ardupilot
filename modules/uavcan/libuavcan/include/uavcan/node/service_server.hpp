/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_NODE_SERVICE_SERVER_HPP_INCLUDED
#define UAVCAN_NODE_SERVICE_SERVER_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/generic_publisher.hpp>
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
 * This type can be used in place of the response type in a service server callback to get more advanced control
 * of service request processing.
 *
 * PLEASE NOTE that since this class inherits the response type, service server callbacks can accept either
 * object of this class or the response type directly if the extra options are not needed.
 *
 * For example, both of these callbacks can be used with the same service type 'Foo':
 *
 *  void first(const ReceivedDataStructure<Foo::Request>& request,
 *             ServiceResponseDataStructure<Foo::Response>& response);
 *
 *  void second(const Foo::Request& request,
 *              Foo::Response& response);
 *
 * In the latter case, an implicit cast will happen before the callback is invoked.
 */
template <typename ResponseDataType_>
class ServiceResponseDataStructure : public ResponseDataType_
{
    // Fields are weirdly named to avoid name clashing with the inherited data type
    bool _enabled_;

public:
    typedef ResponseDataType_ ResponseDataType;

    ServiceResponseDataStructure()
        : _enabled_(true)
    { }

    /**
     * When disabled, the server will not transmit the response transfer.
     * By default it is enabled, i.e. response will be sent.
     */
    void setResponseEnabled(bool x) { _enabled_ = x; }

    /**
     * Whether the response will be sent. By default it will.
     */
    bool isResponseEnabled() const { return _enabled_; }
};

/**
 * Use this class to implement UAVCAN service servers.
 *
 * Note that the references passed to the callback may point to stack-allocated objects, which means that the
 * references get invalidated once the callback returns.
 *
 * @tparam DataType_        Service data type.
 *
 * @tparam Callback_        Service calls will be delivered through the callback of this type, and service
 *                          response will be returned via the output parameter of the callback. Note that
 *                          the reference to service response data struct passed to the callback always points
 *                          to a default initialized response object.
 *                          Please also refer to @ref ReceivedDataStructure<> and @ref ServiceResponseDataStructure<>.
 *                          In C++11 mode this type defaults to std::function<>.
 *                          In C++03 mode this type defaults to a plain function pointer; use binder to
 *                          call member functions as callbacks.
 */
template <typename DataType_,
#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
          typename Callback_ = std::function<void (const ReceivedDataStructure<typename DataType_::Request>&,
                                                   ServiceResponseDataStructure<typename DataType_::Response>&)>
#else
          typename Callback_ = void (*)(const ReceivedDataStructure<typename DataType_::Request>&,
                                        ServiceResponseDataStructure<typename DataType_::Response>&)
#endif
          >
class UAVCAN_EXPORT ServiceServer
    : public GenericSubscriber<DataType_, typename DataType_::Request, TransferListener>
{
public:
    typedef DataType_ DataType;
    typedef typename DataType::Request RequestType;
    typedef typename DataType::Response ResponseType;
    typedef Callback_ Callback;

private:
    typedef GenericSubscriber<DataType, RequestType, TransferListener> SubscriberType;
    typedef GenericPublisher<DataType, ResponseType> PublisherType;

    PublisherType publisher_;
    Callback callback_;
    uint32_t response_failure_count_;

    virtual void handleReceivedDataStruct(ReceivedDataStructure<RequestType>& request) override
    {
        UAVCAN_ASSERT(request.getTransferType() == TransferTypeServiceRequest);

        ServiceResponseDataStructure<ResponseType> response;

        if (coerceOrFallback<bool>(callback_, true))
        {
            UAVCAN_ASSERT(response.isResponseEnabled());  // Enabled by default
            callback_(request, response);
        }
        else
        {
            handleFatalError("Srv serv clbk");
        }

        if (response.isResponseEnabled())
        {
            publisher_.setPriority(request.getPriority());      // Responding at the same priority.

            const int res = publisher_.publish(response, TransferTypeServiceResponse, request.getSrcNodeID(),
                                               request.getTransferID());
            if (res < 0)
            {
                UAVCAN_TRACE("ServiceServer", "Response publication failure: %i", res);
                publisher_.getNode().getDispatcher().getTransferPerfCounter().addError();
                response_failure_count_++;
            }
        }
        else
        {
            UAVCAN_TRACE("ServiceServer", "Response was suppressed by the application");
        }
    }

public:
    explicit ServiceServer(INode& node, bool force_std_can = false)
        : SubscriberType(node)
        , publisher_(node, force_std_can, getDefaultTxTimeout())
        , callback_()
        , response_failure_count_(0)
    {
        UAVCAN_ASSERT(getTxTimeout() == getDefaultTxTimeout());  // Making sure it is valid

        StaticAssert<DataTypeKind(DataType::DataTypeKind) == DataTypeKindService>::check();
    }

    /**
     * Starts the server.
     * Incoming service requests will be passed to the application via the callback.
     */
    int start(const Callback& callback)
    {
        stop();

        if (!coerceOrFallback<bool>(callback, true))
        {
            UAVCAN_TRACE("ServiceServer", "Invalid callback");
            return -ErrInvalidParam;
        }
        callback_ = callback;

        const int publisher_res = publisher_.init();
        if (publisher_res < 0)
        {
            UAVCAN_TRACE("ServiceServer", "Publisher initialization failure: %i", publisher_res);
            return publisher_res;
        }
        return SubscriberType::startAsServiceRequestListener();
    }

    /**
     * Stops the server.
     */
    using SubscriberType::stop;

    static MonotonicDuration getDefaultTxTimeout() { return MonotonicDuration::fromMSec(1000); }
    static MonotonicDuration getMinTxTimeout() { return PublisherType::getMinTxTimeout(); }
    static MonotonicDuration getMaxTxTimeout() { return PublisherType::getMaxTxTimeout(); }

    MonotonicDuration getTxTimeout() const { return publisher_.getTxTimeout(); }
    void setTxTimeout(MonotonicDuration tx_timeout) { publisher_.setTxTimeout(tx_timeout); }

    /**
     * Returns the number of failed attempts to decode data structs. Generally, a failed attempt means either:
     * - Transient failure in the transport layer.
     * - Incompatible data types.
     */
    uint32_t getRequestFailureCount() const { return SubscriberType::getFailureCount(); }
    uint32_t getResponseFailureCount() const { return response_failure_count_; }
};

}

#endif // UAVCAN_NODE_SERVICE_SERVER_HPP_INCLUDED
