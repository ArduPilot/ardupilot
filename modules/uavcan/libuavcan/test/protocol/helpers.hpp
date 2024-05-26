/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/node/subscriber.hpp>
#include <uavcan/node/timer.hpp>
#include <uavcan/node/service_client.hpp>
#include <uavcan/util/method_binder.hpp>
#include "../node/test_node.hpp"


template <typename DataType>
class SubscriptionCollector : uavcan::Noncopyable
{
    void handler(const DataType& msg)
    {
        this->msg.reset(new DataType(msg));
    }

public:
    std::unique_ptr<DataType> msg;

    typedef uavcan::MethodBinder<SubscriptionCollector*,
                                 void (SubscriptionCollector::*)(const DataType&)> Binder;

    Binder bind() { return Binder(this, &SubscriptionCollector::handler); }
};


template <typename DataType>
struct SubscriberWithCollector
{
    typedef SubscriptionCollector<DataType> Collector;
    typedef uavcan::Subscriber<DataType, typename Collector::Binder> Subscriber;

    Collector collector;
    Subscriber subscriber;

    SubscriberWithCollector(uavcan::INode& node)
        : subscriber(node)
    { }

    int start() { return subscriber.start(collector.bind()); }
};


template <typename DataType>
class ServiceCallResultCollector : uavcan::Noncopyable
{
    typedef uavcan::ServiceCallResult<DataType> ServiceCallResult;

public:
    class Result
    {
        const typename ServiceCallResult::Status status_;
        uavcan::ServiceCallID call_id_;
        typename DataType::Response response_;

    public:
        Result(typename ServiceCallResult::Status arg_status,
               uavcan::ServiceCallID arg_call_id,
               const typename DataType::Response& arg_response)
            : status_(arg_status)
            , call_id_(arg_call_id)
            , response_(arg_response)
        { }

        bool isSuccessful() const { return status_ == ServiceCallResult::Success; }

        typename ServiceCallResult::Status getStatus() const { return status_; }

        uavcan::ServiceCallID getCallID() const { return call_id_; }

        const typename DataType::Response& getResponse() const { return response_; }
        typename DataType::Response& getResponse() { return response_; }
    };

private:
    void handler(const uavcan::ServiceCallResult<DataType>& tmp_result)
    {
        std::cout << tmp_result << std::endl;
        result.reset(new Result(tmp_result.getStatus(), tmp_result.getCallID(), tmp_result.getResponse()));
    }

public:
    std::unique_ptr<Result> result;

    typedef uavcan::MethodBinder<ServiceCallResultCollector*,
                                 void (ServiceCallResultCollector::*)(const uavcan::ServiceCallResult<DataType>&)>
            Binder;

    Binder bind() { return Binder(this, &ServiceCallResultCollector::handler); }
};


template <typename DataType>
struct ServiceClientWithCollector
{
    typedef ServiceCallResultCollector<DataType> Collector;
    typedef uavcan::ServiceClient<DataType, typename Collector::Binder> ServiceClient;

    Collector collector;
    ServiceClient client;

    ServiceClientWithCollector(uavcan::INode& node)
        : client(node)
    { }

    int call(uavcan::NodeID node_id, const typename DataType::Request& request)
    {
        client.setCallback(collector.bind());
        return client.call(node_id, request);
    }
};


struct BackgroundSpinner : uavcan::TimerBase
{
    uavcan::INode& spinning_node;

    BackgroundSpinner(uavcan::INode& spinning_node, uavcan::INode& running_node)
        : uavcan::TimerBase(running_node)
        , spinning_node(spinning_node)
    { }

    virtual void handleTimerEvent(const uavcan::TimerEvent&)
    {
        spinning_node.spin(uavcan::MonotonicDuration::fromMSec(1));
    }
};

template <typename CanDriver, typename MessageType>
static inline void emulateSingleFrameBroadcastTransfer(CanDriver& can, uavcan::NodeID node_id,
                                                       const MessageType& message, uavcan::TransferID tid)
{
    uavcan::StaticTransferBuffer<100> buffer;
    uavcan::BitStream bitstream(buffer);
    uavcan::ScalarCodec codec(bitstream);

    // Manual message publication
    ASSERT_LT(0, MessageType::encode(message, codec));
    ASSERT_GE(8, buffer.getMaxWritePos());

    // DataTypeID data_type_id, TransferType transfer_type, NodeID src_node_id, NodeID dst_node_id,
    // uint_fast8_t frame_index, TransferID transfer_id, bool last_frame
    uavcan::Frame frame(MessageType::DefaultDataTypeID, uavcan::TransferTypeMessageBroadcast,
                        node_id, uavcan::NodeID::Broadcast, tid);
    frame.setStartOfTransfer(true);
    frame.setEndOfTransfer(true);

    ASSERT_EQ(buffer.getMaxWritePos(), frame.setPayload(buffer.getRawPtr(), buffer.getMaxWritePos()));

    uavcan::CanFrame can_frame;
    ASSERT_TRUE(frame.compile(can_frame));

    can.pushRxToAllIfaces(can_frame);
}
