/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_EVENT_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_EVENT_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/protocol/dynamic_node_id_server/types.hpp>

namespace uavcan
{
namespace dynamic_node_id_server
{
/**
 * @ref IEventTracer.
 * Event codes cannot be changed, only new ones can be added.
 */
enum UAVCAN_EXPORT TraceCode
{
    // Event name                          Argument
    // 0
    TraceError,                         // error code (may be negated)
    TraceRaftLogLastIndexRestored,      // recovered last index value
    TraceRaftLogAppend,                 // index of new entry
    TraceRaftLogRemove,                 // new last index value
    TraceRaftCurrentTermRestored,       // current term
    // 5
    TraceRaftCurrentTermUpdate,         // current term
    TraceRaftVotedForRestored,          // value of votedFor
    TraceRaftVotedForUpdate,            // value of votedFor
    TraceRaftDiscoveryBroadcast,        // number of known servers
    TraceRaftNewServerDiscovered,       // node ID of the new server
    // 10
    TraceRaftDiscoveryReceived,         // node ID of the sender
    TraceRaftClusterSizeInited,         // cluster size
    TraceRaftBadClusterSizeReceived,    // received cluster size
    TraceRaftCoreInited,                // update interval in usec
    TraceRaftStateSwitch,               // 0 - Follower, 1 - Candidate, 2 - Leader
    // 15
    Trace0,
    TraceRaftNewLogEntry,               // node ID value
    TraceRaftRequestIgnored,            // node ID of the client
    TraceRaftVoteRequestReceived,       // node ID of the client
    TraceRaftVoteRequestSucceeded,      // node ID of the server
    // 20
    TraceRaftVoteRequestInitiation,     // node ID of the server
    TraceRaftPersistStateUpdateError,   // negative error code
    TraceRaftCommitIndexUpdate,         // new commit index value
    TraceRaftNewerTermInResponse,       // new term value
    TraceRaftNewEntryCommitted,         // new commit index value
    // 25
    TraceRaftAppendEntriesCallFailure,  // error code (may be negated)
    TraceRaftElectionComplete,          // number of votes collected
    TraceRaftAppendEntriesRespUnsucfl,  // node ID of the client
    Trace2,
    Trace3,
    // 30
    TraceAllocationFollowupResponse,    // number of unique ID bytes in this response
    TraceAllocationFollowupDenied,      // reason code (see sources for details)
    TraceAllocationFollowupTimeout,     // timeout value in microseconds
    TraceAllocationBadRequest,          // number of unique ID bytes in this request
    TraceAllocationUnexpectedStage,     // stage number in the request - 1, 2, or 3
    // 35
    TraceAllocationRequestAccepted,     // number of bytes of unique ID after request
    TraceAllocationExchangeComplete,    // first 8 bytes of unique ID interpreted as signed 64 bit big endian
    TraceAllocationResponse,            // allocated node ID
    TraceAllocationActivity,            // source node ID of the message
    Trace12,
    // 40
    TraceDiscoveryNewNodeFound,         // node ID
    TraceDiscoveryCommitCacheUpdated,   // node ID marked as committed
    TraceDiscoveryNodeFinalized,        // node ID in lower 7 bits, bit 8 (256, 0x100) is set if unique ID is known
    TraceDiscoveryGetNodeInfoFailure,   // node ID
    TraceDiscoveryTimerStart,           // interval in microseconds
    // 45
    TraceDiscoveryTimerStop,            // reason code (see sources for details)
    TraceDiscoveryGetNodeInfoRequest,   // target node ID
    TraceDiscoveryNodeRestartDetected,  // node ID
    TraceDiscoveryNodeRemoved,          // node ID
    Trace22,
    // 50

    NumTraceCodes
};

/**
 * This interface allows the application to trace events that happen in the server.
 */
class UAVCAN_EXPORT IEventTracer
{
public:
#if UAVCAN_TOSTRING
    /**
     * It is safe to call this function with any argument.
     * If the event code is out of range, an assertion failure will be triggered and an error text will be returned.
     */
    static const char* getEventName(TraceCode code)
    {
        // import re;m = lambda s:',\n'.join('"%s"' % x for x in re.findall(r'\ {4}Trace[0-9]*([A-Za-z0-9]*),',s))
        static const char* const Strings[] =
        {
            "Error",
            "RaftLogLastIndexRestored",
            "RaftLogAppend",
            "RaftLogRemove",
            "RaftCurrentTermRestored",
            "RaftCurrentTermUpdate",
            "RaftVotedForRestored",
            "RaftVotedForUpdate",
            "RaftDiscoveryBroadcast",
            "RaftNewServerDiscovered",
            "RaftDiscoveryReceived",
            "RaftClusterSizeInited",
            "RaftBadClusterSizeReceived",
            "RaftCoreInited",
            "RaftStateSwitch",
            "",
            "RaftNewLogEntry",
            "RaftRequestIgnored",
            "RaftVoteRequestReceived",
            "RaftVoteRequestSucceeded",
            "RaftVoteRequestInitiation",
            "RaftPersistStateUpdateError",
            "RaftCommitIndexUpdate",
            "RaftNewerTermInResponse",
            "RaftNewEntryCommitted",
            "RaftAppendEntriesCallFailure",
            "RaftElectionComplete",
            "RaftAppendEntriesRespUnsucfl",
            "",
            "",
            "AllocationFollowupResponse",
            "AllocationFollowupDenied",
            "AllocationFollowupTimeout",
            "AllocationBadRequest",
            "AllocationUnexpectedStage",
            "AllocationRequestAccepted",
            "AllocationExchangeComplete",
            "AllocationResponse",
            "AllocationActivity",
            "",
            "DiscoveryNewNodeFound",
            "DiscoveryCommitCacheUpdated",
            "DiscoveryNodeFinalized",
            "DiscoveryGetNodeInfoFailure",
            "DiscoveryTimerStart",
            "DiscoveryTimerStop",
            "DiscoveryGetNodeInfoRequest",
            "DiscoveryNodeRestartDetected",
            "DiscoveryNodeRemoved",
            ""
        };
        uavcan::StaticAssert<sizeof(Strings) / sizeof(Strings[0]) == NumTraceCodes>::check();
        UAVCAN_ASSERT(code < NumTraceCodes);
        // coverity[dead_error_line]
        return (code < NumTraceCodes) ? Strings[static_cast<unsigned>(code)] : "INVALID_EVENT_CODE";
    }
#endif

    /**
     * The server invokes this method every time it believes that a noteworthy event has happened.
     * It is guaranteed that event code values will never change, but new ones can be added in future. This ensures
     * full backward compatibility.
     * @param event_code        Event code, see the sources for the enum with values.
     * @param event_argument    Value associated with the event; its meaning depends on the event code.
     */
    virtual void onEvent(TraceCode event_code, int64_t event_argument) = 0;

    virtual ~IEventTracer() { }
};

}
}

#endif // Include guard
