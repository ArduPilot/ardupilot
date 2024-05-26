/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_TRANSPORT_TRANSFER_SENDER_HPP_INCLUDED
#define UAVCAN_TRANSPORT_TRANSFER_SENDER_HPP_INCLUDED

#include <cstdlib>
#include <cassert>
#include <uavcan/build_config.hpp>
#include <uavcan/error.hpp>
#include <uavcan/data_type.hpp>
#include <uavcan/transport/crc.hpp>
#include <uavcan/transport/transfer.hpp>
#include <uavcan/transport/dispatcher.hpp>

namespace uavcan
{

class UAVCAN_EXPORT TransferSender
{
    const MonotonicDuration max_transfer_interval_;

    Dispatcher& dispatcher_;

    TransferPriority priority_;
    CanTxQueue::Qos qos_;
    TransferCRC crc_base_;
    DataTypeID data_type_id_;
    CanIOFlags flags_;
    uint8_t iface_mask_;
    bool allow_anonymous_transfers_;
    bool canfd_frames_;
    void registerError() const;

public:
    enum { AllIfacesMask = 0xFF };

    static MonotonicDuration getDefaultMaxTransferInterval()
    {
        return MonotonicDuration::fromMSec(60 * 1000);
    }

    TransferSender(Dispatcher& dispatcher, const DataTypeDescriptor& data_type, CanTxQueue::Qos qos,
                    MonotonicDuration max_transfer_interval = getDefaultMaxTransferInterval())
        : max_transfer_interval_(max_transfer_interval)
        , dispatcher_(dispatcher)
        , priority_(TransferPriority::Default)
        , qos_(CanTxQueue::Qos())
        , flags_(CanIOFlags(0))
        , iface_mask_(AllIfacesMask)
        , allow_anonymous_transfers_(false)
    {
        init(data_type, qos);
    }

    TransferSender(Dispatcher& dispatcher, MonotonicDuration max_transfer_interval = getDefaultMaxTransferInterval())
        : max_transfer_interval_(max_transfer_interval)
        , dispatcher_(dispatcher)
        , priority_(TransferPriority::Default)
        , qos_(CanTxQueue::Qos())
        , flags_(CanIOFlags(0))
        , iface_mask_(AllIfacesMask)
        , allow_anonymous_transfers_(false)
    { }

    void init(const DataTypeDescriptor& dtid, CanTxQueue::Qos qos);

    bool isInitialized() const { return data_type_id_ != DataTypeID(); }

    CanIOFlags getCanIOFlags() const { return flags_; }
    void setCanIOFlags(CanIOFlags flags) { flags_ = flags; }

    uint8_t getIfaceMask() const { return iface_mask_; }
    void setIfaceMask(uint8_t iface_mask)
    {
        UAVCAN_ASSERT(iface_mask);
        iface_mask_ = iface_mask;
    }

    TransferPriority getPriority() const { return priority_; }
    void setPriority(TransferPriority prio) { priority_ = prio; }

    /**
     * Anonymous transfers (i.e. transfers that don't carry a valid Source Node ID) can be sent if
     * the local node is configured in passive mode (i.e. the node doesn't have a valid Node ID).
     * By default, this class will return an error if it is asked to send a transfer while the
     * node is configured in passive mode. However, if this option is enabled, it will be possible
     * to send anonymous transfers from passive mode.
     */
    void allowAnonymousTransfers() { allow_anonymous_transfers_ = true; }

    /**
     * Send with explicit Transfer ID.
     * Should be used only for service responses, where response TID should match request TID.
     */
    int send(const uint8_t* payload, unsigned payload_len, MonotonicTime tx_deadline,
             MonotonicTime blocking_deadline, TransferType transfer_type, NodeID dst_node_id,
             TransferID tid, bool force_std_can) const;

    /**
     * Send with automatic Transfer ID.
     *
     * Note that as long as the local node operates in passive mode, the
     * flag @ref CanIOFlagAbortOnError will be set implicitly for all outgoing frames.
     *
     * TID is managed by OutgoingTransferRegistry.
     */
    int send(const uint8_t* payload, unsigned payload_len, MonotonicTime tx_deadline,
             MonotonicTime blocking_deadline, TransferType transfer_type, NodeID dst_node_id, bool force_std_can) const;
};

}

#endif // UAVCAN_TRANSPORT_TRANSFER_SENDER_HPP_INCLUDED
