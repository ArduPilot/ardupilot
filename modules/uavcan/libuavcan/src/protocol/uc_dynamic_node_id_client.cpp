/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cstdlib>
#include <uavcan/protocol/dynamic_node_id_client.hpp>

namespace uavcan
{

void DynamicNodeIDClient::terminate()
{
    UAVCAN_TRACE("DynamicNodeIDClient", "Client terminated");
    stop();
    dnida_sub_.stop();
}

MonotonicDuration DynamicNodeIDClient::getRandomDuration(uint32_t lower_bound_msec, uint32_t upper_bound_msec)
{
    UAVCAN_ASSERT(upper_bound_msec > lower_bound_msec);
    // coverity[dont_call]
    return MonotonicDuration::fromMSec(lower_bound_msec +
                                       static_cast<uint32_t>(std::rand()) % (upper_bound_msec - lower_bound_msec));
}

void DynamicNodeIDClient::restartTimer(const Mode mode)
{
    UAVCAN_ASSERT(mode < NumModes);
    UAVCAN_ASSERT((mode == ModeWaitingForTimeSlot) == (size_of_received_unique_id_ == 0));

    const MonotonicDuration delay = (mode == ModeWaitingForTimeSlot) ?
        getRandomDuration(protocol::dynamic_node_id::Allocation::MIN_REQUEST_PERIOD_MS,
                          protocol::dynamic_node_id::Allocation::MAX_REQUEST_PERIOD_MS) :
        getRandomDuration(protocol::dynamic_node_id::Allocation::MIN_FOLLOWUP_DELAY_MS,
                          protocol::dynamic_node_id::Allocation::MAX_FOLLOWUP_DELAY_MS);

    startOneShotWithDelay(delay);

    UAVCAN_TRACE("DynamicNodeIDClient", "Restart mode %d, delay %d ms",
                 static_cast<int>(mode), static_cast<int>(delay.toMSec()));
}

void DynamicNodeIDClient::handleTimerEvent(const TimerEvent&)
{
    UAVCAN_ASSERT(preferred_node_id_.isValid());
    UAVCAN_ASSERT(size_of_received_unique_id_ < protocol::dynamic_node_id::Allocation::FieldTypes::unique_id::MaxSize);

    if (isAllocationComplete())
    {
        UAVCAN_ASSERT(0);
        terminate();
        return;
    }

    /*
     * Filling the message.
     */
    protocol::dynamic_node_id::Allocation tx;
    tx.node_id = preferred_node_id_.get();
    tx.first_part_of_unique_id = (size_of_received_unique_id_ == 0);

    const uint8_t size_of_unique_id_in_request =
        min(protocol::dynamic_node_id::Allocation::MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST,
            static_cast<uint8_t>(tx.unique_id.capacity() - size_of_received_unique_id_));

    tx.unique_id.resize(size_of_unique_id_in_request);
    copy(unique_id_ + size_of_received_unique_id_,
         unique_id_ + size_of_received_unique_id_ + size_of_unique_id_in_request,
         tx.unique_id.begin());

    UAVCAN_ASSERT(equal(tx.unique_id.begin(), tx.unique_id.end(), unique_id_ + size_of_received_unique_id_));

    /*
     * Resetting the state - this way we can continue with a first stage request on the next attempt.
     */
    size_of_received_unique_id_ = 0;
    restartTimer(ModeWaitingForTimeSlot);

    /*
     * Broadcasting the message.
     */
    UAVCAN_TRACE("DynamicNodeIDClient", "Broadcasting; preferred ID %d, size of UID %d",
                 static_cast<int>(preferred_node_id_.get()),
                 static_cast<int>(tx.unique_id.size()));
    const int res = dnida_pub_.broadcast(tx);
    if (res < 0)
    {
        dnida_pub_.getNode().registerInternalFailure("DynamicNodeIDClient request failed");
    }
}

void DynamicNodeIDClient::handleAllocation(const ReceivedDataStructure<protocol::dynamic_node_id::Allocation>& msg)
{
    UAVCAN_ASSERT(preferred_node_id_.isValid());
    if (isAllocationComplete())
    {
        UAVCAN_ASSERT(0);
        terminate();
        return;
    }

    UAVCAN_TRACE("DynamicNodeIDClient", "Allocation message from %d, %d bytes of unique ID, node ID %d",
                 static_cast<int>(msg.getSrcNodeID().get()), static_cast<int>(msg.unique_id.size()),
                 static_cast<int>(msg.node_id));

    /*
     * Switching to passive state by default; will switch to active state if response matches.
     */
    size_of_received_unique_id_ = 0;
    restartTimer(ModeWaitingForTimeSlot);

    /*
     * Filtering out anonymous and invalid messages.
     */
    const bool full_response = (msg.unique_id.size() == msg.unique_id.capacity());

    if (msg.isAnonymousTransfer() ||
        msg.unique_id.empty() ||
        (full_response && (msg.node_id == 0)))
    {
        UAVCAN_TRACE("DynamicNodeIDClient", "Message from %d ignored", static_cast<int>(msg.getSrcNodeID().get()));
        return;
    }

    /*
     * If matches, either switch to active mode or complete the allocation.
     */
    if (equal(msg.unique_id.begin(), msg.unique_id.end(), unique_id_))
    {
        if (full_response)
        {
            allocated_node_id_ = msg.node_id;
            allocator_node_id_ = msg.getSrcNodeID();
            terminate();
            UAVCAN_ASSERT(isAllocationComplete());
            UAVCAN_TRACE("DynamicNodeIDClient", "Allocation complete, node ID %d provided by %d",
                         static_cast<int>(allocated_node_id_.get()), static_cast<int>(allocator_node_id_.get()));
        }
        else
        {
            size_of_received_unique_id_ = msg.unique_id.size();
            restartTimer(ModeDelayBeforeFollowup);
        }
    }
}

int DynamicNodeIDClient::start(const UniqueID& unique_id,
                               const NodeID preferred_node_id,
                               const TransferPriority transfer_priority)
{
    terminate();

    // Allocation is not possible if node ID is already set
    if (dnida_pub_.getNode().getNodeID().isUnicast())
    {
        return -ErrLogic;
    }

    // Unique ID initialization & validation
    copy(unique_id.begin(), unique_id.end(), unique_id_);
    bool unique_id_is_zero = true;
    for (uint8_t i = 0; i < sizeof(unique_id_); i++)
    {
        if (unique_id_[i] != 0)
        {
            unique_id_is_zero = false;
            break;
        }
    }

    if (unique_id_is_zero)
    {
        return -ErrInvalidParam;
    }

    if (!preferred_node_id.isValid())  // Only broadcast and unicast are allowed
    {
        return -ErrInvalidParam;
    }

    // Initializing the fields
    preferred_node_id_ = preferred_node_id;
    allocated_node_id_ = NodeID();
    allocator_node_id_ = NodeID();
    UAVCAN_ASSERT(preferred_node_id_.isValid());
    UAVCAN_ASSERT(!allocated_node_id_.isValid());
    UAVCAN_ASSERT(!allocator_node_id_.isValid());

    // Initializing node objects - Rule A
    int res = dnida_pub_.init();
    if (res < 0)
    {
        return res;
    }
    dnida_pub_.allowAnonymousTransfers();
    dnida_pub_.setPriority(transfer_priority);

    res = dnida_sub_.start(AllocationCallback(this, &DynamicNodeIDClient::handleAllocation));
    if (res < 0)
    {
        return res;
    }
    dnida_sub_.allowAnonymousTransfers();

    restartTimer(ModeWaitingForTimeSlot);

    return 0;
}

}
