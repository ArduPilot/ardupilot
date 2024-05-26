/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_TYPES_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_TYPES_HPP_INCLUDED

#include <uavcan/build_config.hpp>
// UAVCAN types
#include <uavcan/protocol/dynamic_node_id/server/Entry.hpp>

namespace uavcan
{
namespace dynamic_node_id_server
{

using namespace ::uavcan::protocol::dynamic_node_id;

/**
 * Node Unique ID
 */
typedef protocol::dynamic_node_id::server::Entry::FieldTypes::unique_id UniqueID;

}
}

#endif // Include guard
