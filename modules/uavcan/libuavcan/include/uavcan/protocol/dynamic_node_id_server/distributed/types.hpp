/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISTRIBUTED_TYPES_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISTRIBUTED_TYPES_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/protocol/dynamic_node_id_server/types.hpp>

namespace uavcan
{
namespace dynamic_node_id_server
{
namespace distributed
{

using namespace ::uavcan::protocol::dynamic_node_id::server;

/**
 * Raft term
 */
typedef StorageType<Entry::FieldTypes::term>::Type Term;

}
}
}

#endif // Include guard
