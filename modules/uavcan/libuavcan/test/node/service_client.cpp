/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/node/service_client.hpp>
#include <uavcan/node/service_server.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/protocol/RestartNode.hpp>
#include <uavcan/protocol/GetDataTypeInfo.hpp>
#include <root_ns_a/StringService.hpp>
#include <root_ns_a/EmptyService.hpp>
#include <queue>
#include <sstream>
#include "test_node.hpp"


template <typename DataType>
struct ServiceCallResultHandler
{
    typedef typename uavcan::ServiceCallResult<DataType>::Status StatusType;
    StatusType last_status;
    uavcan::NodeID last_server_node_id;
    typename DataType::Response last_response;
    std::queue<typename DataType::Response> responses;

    void handleResponse(const uavcan::ServiceCallResult<DataType>& result)
    {
        std::cout << result << std::endl;
        last_status = result.getStatus();
        last_response = result.getResponse();
        last_server_node_id = result.getCallID().server_node_id;
        responses.push(result.getResponse());
    }

    bool match(StatusType status, uavcan::NodeID server_node_id, const typename DataType::Response& response) const
    {
        if (status == last_status &&
            server_node_id == last_server_node_id &&
            response == last_response)
        {
            return true;
        }
        else
        {
            std::cout << "MISMATCH: status=" << int(last_status) << ", last_server_node_id="
                << int(last_server_node_id.get()) << ", last response:\n" << last_response << std::endl;
            return false;
        }
    }

    typedef uavcan::MethodBinder<ServiceCallResultHandler*,
                                 void (ServiceCallResultHandler::*)(const uavcan::ServiceCallResult<DataType>&)> Binder;

    Binder bind() { return Binder(this, &ServiceCallResultHandler::handleResponse); }
};


static void stringServiceServerCallback(const uavcan::ReceivedDataStructure<root_ns_a::StringService::Request>& req,
                                        uavcan::ServiceResponseDataStructure<root_ns_a::StringService::Response>& rsp)
{
    rsp.string_response = "Request string: ";
    rsp.string_response += req.string_request;
}

static void rejectingStringServiceServerCallback(
    const uavcan::ReceivedDataStructure<root_ns_a::StringService::Request>& req,
    uavcan::ServiceResponseDataStructure<root_ns_a::StringService::Response>& rsp)
{
    rsp.string_response = "Request string: ";
    rsp.string_response += req.string_request;
    ASSERT_TRUE(rsp.isResponseEnabled());
    rsp.setResponseEnabled(false);
    ASSERT_FALSE(rsp.isResponseEnabled());
}

static void emptyServiceServerCallback(const uavcan::ReceivedDataStructure<root_ns_a::EmptyService::Request>&,
                                       uavcan::ServiceResponseDataStructure<root_ns_a::EmptyService::Response>&)
{
    // Nothing to do - the service is empty
}


TEST(ServiceClient, Basic)
{
    InterlinkedTestNodesWithSysClock nodes;

    // Type registration
    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<root_ns_a::StringService> _registrator;

    // Server
    uavcan::ServiceServer<root_ns_a::StringService> server(nodes.a);
    ASSERT_EQ(0, server.start(stringServiceServerCallback));

    {
        // Caller
        typedef uavcan::ServiceCallResult<root_ns_a::StringService> ResultType;
        typedef uavcan::ServiceClient<root_ns_a::StringService,
                                      typename ServiceCallResultHandler<root_ns_a::StringService>::Binder > ClientType;
        ServiceCallResultHandler<root_ns_a::StringService> handler;

        ClientType client1(nodes.b);
        ClientType client2(nodes.b);
        ClientType client3(nodes.b);

        ASSERT_EQ(0, client1.getNumPendingCalls());
        ASSERT_EQ(0, client2.getNumPendingCalls());
        ASSERT_EQ(0, client3.getNumPendingCalls());

        ASSERT_FALSE(client1.hasPendingCallToServer(1));

        client1.setCallback(handler.bind());
        client2.setCallback(client1.getCallback());
        client3.setCallback(client1.getCallback());
        client3.setRequestTimeout(uavcan::MonotonicDuration::fromMSec(100));

        ASSERT_EQ(1, nodes.a.getDispatcher().getNumServiceRequestListeners());
        ASSERT_EQ(0, nodes.b.getDispatcher().getNumServiceResponseListeners()); // NOT listening!

        root_ns_a::StringService::Request request;
        request.string_request = "Hello world";

        std::cout << "!!! Calling!" << std::endl;

        ASSERT_LT(0, client1.call(1, request)); // OK
        ASSERT_LT(0, client1.call(1, request)); // OK - second request
        ASSERT_LT(0, client2.call(1, request)); // OK
        ASSERT_LT(0, client3.call(99, request)); // Will timeout!
        ASSERT_LT(0, client3.call(1, request)); // OK - second request

        ASSERT_TRUE(client1.hasPendingCallToServer(1));
        ASSERT_TRUE(client2.hasPendingCallToServer(1));
        ASSERT_TRUE(client3.hasPendingCallToServer(99));
        ASSERT_TRUE(client3.hasPendingCallToServer(1));

        std::cout << "!!! Spinning!" << std::endl;

        ASSERT_EQ(3, nodes.b.getDispatcher().getNumServiceResponseListeners()); // Listening now!

        ASSERT_TRUE(client1.hasPendingCalls());
        ASSERT_TRUE(client2.hasPendingCalls());
        ASSERT_TRUE(client3.hasPendingCalls());

        ASSERT_EQ(2, client1.getNumPendingCalls());
        ASSERT_EQ(1, client2.getNumPendingCalls());
        ASSERT_EQ(2, client3.getNumPendingCalls());

        ASSERT_EQ(uavcan::NodeID(1), client2.getCallIDByIndex(0).server_node_id);
        ASSERT_EQ(uavcan::NodeID(), client2.getCallIDByIndex(1).server_node_id);

        nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(20));

        std::cout << "!!! Spin finished!" << std::endl;

        ASSERT_EQ(1, nodes.b.getDispatcher().getNumServiceResponseListeners()); // Third is still listening!

        ASSERT_FALSE(client1.hasPendingCalls());
        ASSERT_FALSE(client2.hasPendingCalls());
        ASSERT_TRUE(client3.hasPendingCalls());

        ASSERT_EQ(0, client1.getNumPendingCalls());
        ASSERT_EQ(0, client2.getNumPendingCalls());
        ASSERT_EQ(1, client3.getNumPendingCalls());     // The one addressed to 99 is still waiting

        // Validating
        root_ns_a::StringService::Response expected_response;
        expected_response.string_response = "Request string: Hello world";
        ASSERT_TRUE(handler.match(ResultType::Success, 1, expected_response));

        nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(200));

        ASSERT_FALSE(client1.hasPendingCalls());
        ASSERT_FALSE(client2.hasPendingCalls());
        ASSERT_FALSE(client3.hasPendingCalls());

        ASSERT_EQ(0, nodes.b.getDispatcher().getNumServiceResponseListeners()); // Third has timed out :(

        // Validating
        ASSERT_TRUE(handler.match(ResultType::ErrorTimeout, 99, root_ns_a::StringService::Response()));

        // Stray request
        ASSERT_LT(0, client3.call(99, request)); // Will timeout!
        ASSERT_TRUE(client3.hasPendingCalls());
        ASSERT_EQ(1, nodes.b.getDispatcher().getNumServiceResponseListeners());
    }

    // All destroyed - nobody listening
    ASSERT_EQ(0, nodes.b.getDispatcher().getNumServiceResponseListeners());
}


TEST(ServiceClient, Rejection)
{
    InterlinkedTestNodesWithSysClock nodes;

    // Type registration
    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<root_ns_a::StringService> _registrator;

    // Server
    uavcan::ServiceServer<root_ns_a::StringService> server(nodes.a);
    ASSERT_EQ(0, server.start(rejectingStringServiceServerCallback));

    // Caller
    typedef uavcan::ServiceCallResult<root_ns_a::StringService> ResultType;
    typedef uavcan::ServiceClient<root_ns_a::StringService,
                                  typename ServiceCallResultHandler<root_ns_a::StringService>::Binder > ClientType;
    ServiceCallResultHandler<root_ns_a::StringService> handler;

    ClientType client1(nodes.b);
    client1.setRequestTimeout(uavcan::MonotonicDuration::fromMSec(100));
    client1.setCallback(handler.bind());

    root_ns_a::StringService::Request request;
    request.string_request = "Hello world";

    ASSERT_LT(0, client1.call(1, request));

    ASSERT_EQ(uavcan::NodeID(1), client1.getCallIDByIndex(0).server_node_id);
    ASSERT_EQ(uavcan::NodeID(), client1.getCallIDByIndex(1).server_node_id);

    ASSERT_EQ(1, nodes.b.getDispatcher().getNumServiceResponseListeners());
    ASSERT_TRUE(client1.hasPendingCalls());
    ASSERT_TRUE(client1.hasPendingCallToServer(1));

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(200));
    ASSERT_FALSE(client1.hasPendingCalls());
    ASSERT_FALSE(client1.hasPendingCallToServer(1));

    ASSERT_EQ(0, nodes.b.getDispatcher().getNumServiceResponseListeners()); // Timed out

    ASSERT_TRUE(handler.match(ResultType::ErrorTimeout, 1, root_ns_a::StringService::Response()));
}


TEST(ServiceClient, ConcurrentCalls)
{
    InterlinkedTestNodesWithSysClock nodes;

    // Type registration
    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<root_ns_a::StringService> _registrator;

    // Server
    uavcan::ServiceServer<root_ns_a::StringService> server(nodes.a);
    ASSERT_EQ(0, server.start(stringServiceServerCallback));

    // Caller
    typedef uavcan::ServiceCallResult<root_ns_a::StringService> ResultType;
    typedef uavcan::ServiceClient<root_ns_a::StringService,
                                  typename ServiceCallResultHandler<root_ns_a::StringService>::Binder > ClientType;
    ServiceCallResultHandler<root_ns_a::StringService> handler;

    /*
     * Initializing
     */
    ClientType client(nodes.b);

    ASSERT_EQ(0, client.getNumPendingCalls());

    client.setCallback(handler.bind());

    ASSERT_EQ(1, nodes.a.getDispatcher().getNumServiceRequestListeners());
    ASSERT_EQ(0, nodes.b.getDispatcher().getNumServiceResponseListeners()); // NOT listening!

    ASSERT_FALSE(client.hasPendingCalls());
    ASSERT_EQ(0, client.getNumPendingCalls());

    /*
     * Calling ten requests, the last one will be cancelled
     * Note that there will be non-unique transfer ID values; the client must handle that correctly
     */
    uavcan::ServiceCallID last_call_id;
    for (int i = 0; i < 10; i++)
    {
        std::ostringstream os;
        os << i;
        root_ns_a::StringService::Request request;
        request.string_request = os.str().c_str();
        ASSERT_LT(0, client.call(1, request, last_call_id));
    }

    ASSERT_LT(0, client.call(99, root_ns_a::StringService::Request()));         // Will timeout in 1000 ms

    client.setRequestTimeout(uavcan::MonotonicDuration::fromMSec(100));

    ASSERT_LT(0, client.call(88, root_ns_a::StringService::Request()));         // Will timeout in 100 ms

    ASSERT_TRUE(client.hasPendingCalls());
    ASSERT_EQ(12, client.getNumPendingCalls());

    ASSERT_EQ(1, nodes.b.getDispatcher().getNumServiceResponseListeners());     // Listening

    /*
     * Cancelling one
     */
    client.cancelCall(last_call_id);

    ASSERT_TRUE(client.hasPendingCalls());
    ASSERT_EQ(11, client.getNumPendingCalls());

    ASSERT_EQ(1, nodes.b.getDispatcher().getNumServiceResponseListeners());     // Still listening

    /*
     * Spinning
     */
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(20));

    ASSERT_TRUE(client.hasPendingCalls());
    ASSERT_EQ(2, client.getNumPendingCalls());                                  // Two still pending
    ASSERT_EQ(1, nodes.b.getDispatcher().getNumServiceResponseListeners());     // Still listening

    /*
     * Validating the ones that didn't timeout
     */
    root_ns_a::StringService::Response last_response;
    for (int i = 0; i < 9; i++)
    {
        std::ostringstream os;
        os << "Request string: " << i;
        last_response.string_response = os.str().c_str();

        ASSERT_FALSE(handler.responses.empty());

        ASSERT_STREQ(last_response.string_response.c_str(), handler.responses.front().string_response.c_str());

        handler.responses.pop();
    }

    ASSERT_TRUE(handler.responses.empty());

    /*
     * Validating the 100 ms timeout
     */
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(100));

    ASSERT_TRUE(client.hasPendingCalls());
    ASSERT_EQ(1, client.getNumPendingCalls());                                  // One dropped
    ASSERT_EQ(1, nodes.b.getDispatcher().getNumServiceResponseListeners());     // Still listening

    ASSERT_TRUE(handler.match(ResultType::ErrorTimeout, 88, root_ns_a::StringService::Response()));

    /*
     * Validating the 1000 ms timeout
     */
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1000));

    ASSERT_FALSE(client.hasPendingCalls());
    ASSERT_EQ(0, client.getNumPendingCalls());                                  // All finished
    ASSERT_EQ(0, nodes.b.getDispatcher().getNumServiceResponseListeners());     // Not listening

    ASSERT_TRUE(handler.match(ResultType::ErrorTimeout, 99, root_ns_a::StringService::Response()));
}


TEST(ServiceClient, Empty)
{
    InterlinkedTestNodesWithSysClock nodes;

    // Type registration
    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<root_ns_a::EmptyService> _registrator;

    // Server
    uavcan::ServiceServer<root_ns_a::EmptyService> server(nodes.a);
    ASSERT_EQ(0, server.start(emptyServiceServerCallback));

    {
        // Caller
        typedef uavcan::ServiceClient<root_ns_a::EmptyService,
                                      typename ServiceCallResultHandler<root_ns_a::EmptyService>::Binder > ClientType;
        ServiceCallResultHandler<root_ns_a::EmptyService> handler;

        ClientType client(nodes.b);

        client.setCallback(handler.bind());

        root_ns_a::EmptyService::Request request;

        ASSERT_LT(0, client.call(1, request)); // OK
    }

    // All destroyed - nobody listening
    ASSERT_EQ(0, nodes.b.getDispatcher().getNumServiceResponseListeners());
}


TEST(ServiceClient, Priority)
{
    InterlinkedTestNodesWithSysClock nodes;

    // Type registration
    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<root_ns_a::EmptyService> _registrator;

    // Initializing
    typedef uavcan::ServiceClient<root_ns_a::EmptyService,
                                  typename ServiceCallResultHandler<root_ns_a::EmptyService>::Binder > ClientType;
    ServiceCallResultHandler<root_ns_a::EmptyService> handler;
    ClientType client(nodes.b);
    client.setCallback(handler.bind());

    // Calling
    root_ns_a::EmptyService::Request request;

    client.setPriority(0);
    ASSERT_LT(0, client.call(1, request)); // OK

    client.setPriority(10);
    ASSERT_LT(0, client.call(1, request)); // OK

    client.setPriority(20);
    ASSERT_LT(0, client.call(1, request)); // OK

    client.setPriority(31);
    ASSERT_LT(0, client.call(1, request)); // OK

    // Validating
    ASSERT_EQ(4, nodes.can_a.read_queue.size());

    uavcan::Frame frame;

    ASSERT_TRUE(frame.parse(nodes.can_a.read_queue.front()));
    nodes.can_a.read_queue.pop();
    ASSERT_EQ(0, frame.getPriority().get());

    ASSERT_TRUE(frame.parse(nodes.can_a.read_queue.front()));
    nodes.can_a.read_queue.pop();
    ASSERT_EQ(10, frame.getPriority().get());

    ASSERT_TRUE(frame.parse(nodes.can_a.read_queue.front()));
    nodes.can_a.read_queue.pop();
    ASSERT_EQ(20, frame.getPriority().get());

    ASSERT_TRUE(frame.parse(nodes.can_a.read_queue.front()));
    nodes.can_a.read_queue.pop();
    ASSERT_EQ(31, frame.getPriority().get());
}


TEST(ServiceClient, Sizes)
{
    using namespace uavcan;

    std::cout << "GetDataTypeInfo server: " <<
        sizeof(ServiceServer<protocol::GetDataTypeInfo>) << std::endl;

    std::cout << "RestartNode server: " <<
        sizeof(ServiceServer<protocol::RestartNode>) << std::endl;

    std::cout << "GetDataTypeInfo client: " <<
        sizeof(ServiceClient<protocol::GetDataTypeInfo>) << std::endl;
}
