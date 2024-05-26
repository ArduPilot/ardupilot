/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <iostream>
#include <uavcan/protocol/dynamic_node_id_server/types.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/node/service_server.hpp>
#include <uavcan/protocol/GetNodeInfo.hpp>

class GetNodeInfoMockServer
{
    typedef uavcan::MethodBinder<GetNodeInfoMockServer*,
        void (GetNodeInfoMockServer::*)(const uavcan::ReceivedDataStructure<uavcan::protocol::GetNodeInfo::Request>&,
                                        uavcan::protocol::GetNodeInfo::Response&) const>
            GetNodeInfoCallback;

    uavcan::ServiceServer<uavcan::protocol::GetNodeInfo, GetNodeInfoCallback> server_;

    void handleRequest(const uavcan::ReceivedDataStructure<uavcan::protocol::GetNodeInfo::Request>& req,
                       uavcan::protocol::GetNodeInfo::Response& res) const
    {
        res = response;
        std::cout << "GET NODE INFO:\nREQUEST\n" << req << "RESPONSE\n" << res << std::endl;
    }

public:
    uavcan::protocol::GetNodeInfo::Response response;

    GetNodeInfoMockServer(uavcan::INode& node) : server_(node) { }

    int start() { return server_.start(GetNodeInfoCallback(this, &GetNodeInfoMockServer::handleRequest)); }
};
