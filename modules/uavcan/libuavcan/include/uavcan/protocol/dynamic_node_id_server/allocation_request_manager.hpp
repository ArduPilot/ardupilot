/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_ALLOCATION_REQUEST_MANAGER_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_ALLOCATION_REQUEST_MANAGER_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/node/subscriber.hpp>
#include <uavcan/node/publisher.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/protocol/dynamic_node_id_server/types.hpp>
#include <uavcan/protocol/dynamic_node_id_server/event.hpp>
// UAVCAN types
#include <uavcan/protocol/dynamic_node_id/Allocation.hpp>

namespace uavcan
{
namespace dynamic_node_id_server
{
/**
 * The main allocator must implement this interface.
 */
class IAllocationRequestHandler
{
public:
    /**
     * Allocation request manager uses this method to detect if it is allowed to publish follow-up responses.
     */
    virtual bool canPublishFollowupAllocationResponse() const = 0;

    /**
     * This method will be invoked when a new allocation request is received.
     */
    virtual void handleAllocationRequest(const UniqueID& unique_id, NodeID preferred_node_id) = 0;

    virtual ~IAllocationRequestHandler() { }
};

/**
 * This class manages communication with allocation clients.
 * Three-stage unique ID exchange is implemented here, as well as response publication.
 */
class AllocationRequestManager
{
    typedef MethodBinder<AllocationRequestManager*,
                         void (AllocationRequestManager::*)(const ReceivedDataStructure<Allocation>&)>
        AllocationCallback;

    const MonotonicDuration stage_timeout_;

    MonotonicTime last_message_timestamp_;
    MonotonicTime last_activity_timestamp_;
    Allocation::FieldTypes::unique_id current_unique_id_;

    IAllocationRequestHandler& handler_;
    IEventTracer& tracer_;

    Subscriber<Allocation, AllocationCallback> allocation_sub_;
    Publisher<Allocation> allocation_pub_;

    enum { InvalidStage = 0 };

    void trace(TraceCode code, int64_t argument) { tracer_.onEvent(code, argument); }

    static uint8_t detectRequestStage(const Allocation& msg)
    {
        const uint8_t max_bytes_per_request = Allocation::MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST;

        if ((msg.unique_id.size() != max_bytes_per_request) &&
            (msg.unique_id.size() != (msg.unique_id.capacity() - max_bytes_per_request * 2U)) &&
            (msg.unique_id.size() != msg.unique_id.capacity()))     // Future proofness for CAN FD
        {
            return InvalidStage;
        }
        if (msg.first_part_of_unique_id)
        {
            return 1;       // Note that CAN FD frames can deliver the unique ID in one stage!
        }
        if (msg.unique_id.size() == Allocation::MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST)
        {
            return 2;
        }
        if (msg.unique_id.size() < Allocation::MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST)
        {
            return 3;
        }
        return InvalidStage;
    }

    uint8_t getExpectedStage() const
    {
        if (current_unique_id_.empty())
        {
            return 1;
        }
        if (current_unique_id_.size() >= (Allocation::MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST * 2))
        {
            return 3;
        }
        if (current_unique_id_.size() >= Allocation::MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST)
        {
            return 2;
        }
        return InvalidStage;
    }

    void publishFollowupAllocationResponse()
    {
        Allocation msg;
        msg.unique_id = current_unique_id_;
        UAVCAN_ASSERT(msg.unique_id.size() < msg.unique_id.capacity());

        UAVCAN_TRACE("AllocationRequestManager", "Intermediate response with %u bytes of unique ID",
                     unsigned(msg.unique_id.size()));

        trace(TraceAllocationFollowupResponse, msg.unique_id.size());

        const int res = allocation_pub_.broadcast(msg);
        if (res < 0)
        {
            trace(TraceError, res);
            allocation_pub_.getNode().registerInternalFailure("Dynamic allocation broadcast");
        }
    }

    void handleAllocation(const ReceivedDataStructure<Allocation>& msg)
    {
        trace(TraceAllocationActivity, msg.getSrcNodeID().get());
        last_activity_timestamp_ = msg.getMonotonicTimestamp();

        if (!msg.isAnonymousTransfer())
        {
            return;         // This is a response from another allocator, ignore
        }

        /*
         * Reset the expected stage on timeout
         */
        if (msg.getMonotonicTimestamp() > (last_message_timestamp_ + stage_timeout_))
        {
            UAVCAN_TRACE("AllocationRequestManager", "Stage timeout, reset");
            current_unique_id_.clear();
            trace(TraceAllocationFollowupTimeout, (msg.getMonotonicTimestamp() - last_message_timestamp_).toUSec());
        }

        /*
         * Checking if request stage matches the expected stage
         */
        const uint8_t request_stage = detectRequestStage(msg);
        if (request_stage == InvalidStage)
        {
            trace(TraceAllocationBadRequest, msg.unique_id.size());
            return;             // Malformed request - ignore without resetting
        }

        const uint8_t expected_stage = getExpectedStage();
        if (expected_stage == InvalidStage)
        {
            UAVCAN_ASSERT(0);
            return;
        }

        if (request_stage != expected_stage)
        {
            trace(TraceAllocationUnexpectedStage, request_stage);
            return;             // Ignore - stage mismatch
        }

        const uint8_t max_expected_bytes =
            static_cast<uint8_t>(current_unique_id_.capacity() - current_unique_id_.size());
        UAVCAN_ASSERT(max_expected_bytes > 0);
        if (msg.unique_id.size() > max_expected_bytes)
        {
            trace(TraceAllocationBadRequest, msg.unique_id.size());
            return;             // Malformed request
        }

        /*
         * Updating the local state
         */
        for (uint8_t i = 0; i < msg.unique_id.size(); i++)
        {
            current_unique_id_.push_back(msg.unique_id[i]);
        }

        trace(TraceAllocationRequestAccepted, current_unique_id_.size());

        /*
         * Proceeding with allocation if possible
         * Note that single-frame CAN FD allocation requests will be delivered to the server even if it's not leader.
         */
        if (current_unique_id_.size() == current_unique_id_.capacity())
        {
            UAVCAN_TRACE("AllocationRequestManager", "Allocation request received; preferred node ID: %d",
                         int(msg.node_id));

            UniqueID unique_id;
            copy(current_unique_id_.begin(), current_unique_id_.end(), unique_id.begin());
            current_unique_id_.clear();

            {
                uint64_t event_agrument = 0;
                for (uint8_t i = 0; i < 8; i++)
                {
                    event_agrument |= static_cast<uint64_t>(unique_id[i]) << (56U - i * 8U);
                }
                trace(TraceAllocationExchangeComplete, static_cast<int64_t>(event_agrument));
            }

            handler_.handleAllocationRequest(unique_id, msg.node_id);
        }
        else
        {
            if (handler_.canPublishFollowupAllocationResponse())
            {
                publishFollowupAllocationResponse();
            }
            else
            {
                trace(TraceAllocationFollowupDenied, 0);
                current_unique_id_.clear();
            }
        }

        /*
         * It is super important to update timestamp only if the request has been processed successfully.
         */
        last_message_timestamp_ = msg.getMonotonicTimestamp();
    }

public:
    AllocationRequestManager(INode& node, IEventTracer& tracer, IAllocationRequestHandler& handler)
        : stage_timeout_(MonotonicDuration::fromMSec(Allocation::FOLLOWUP_TIMEOUT_MS))
        , handler_(handler)
        , tracer_(tracer)
        , allocation_sub_(node)
        , allocation_pub_(node)
    { }

    int init(const TransferPriority priority)
    {
        int res = allocation_pub_.init(priority);
        if (res < 0)
        {
            return res;
        }
        allocation_pub_.setTxTimeout(MonotonicDuration::fromMSec(Allocation::FOLLOWUP_TIMEOUT_MS));

        res = allocation_sub_.start(AllocationCallback(this, &AllocationRequestManager::handleAllocation));
        if (res < 0)
        {
            return res;
        }
        allocation_sub_.allowAnonymousTransfers();

        return 0;
    }

    int broadcastAllocationResponse(const UniqueID& unique_id, NodeID allocated_node_id)
    {
        Allocation msg;

        msg.unique_id.resize(msg.unique_id.capacity());
        copy(unique_id.begin(), unique_id.end(), msg.unique_id.begin());

        msg.node_id = allocated_node_id.get();

        trace(TraceAllocationResponse, msg.node_id);
        last_activity_timestamp_ = allocation_pub_.getNode().getMonotonicTime();

        return allocation_pub_.broadcast(msg);
    }

    /**
     * When the last allocation activity was registered.
     * This value can be used to heuristically determine whether there are any unallocated nodes left in the network.
     */
    MonotonicTime getTimeOfLastAllocationActivity() const { return last_activity_timestamp_; }
};

}
}

#endif // Include guard
