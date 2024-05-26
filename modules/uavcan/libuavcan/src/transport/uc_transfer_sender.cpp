/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/transport/transfer_sender.hpp>
#include <uavcan/debug.hpp>
#include <cassert>


namespace uavcan
{

void TransferSender::registerError() const
{
    dispatcher_.getTransferPerfCounter().addError();
}

void TransferSender::init(const DataTypeDescriptor& dtid, CanTxQueue::Qos qos)
{
    UAVCAN_ASSERT(!isInitialized());

    qos_          = qos;
    data_type_id_ = dtid.getID();
    crc_base_     = dtid.getSignature().toTransferCRC();
}

int TransferSender::send(const uint8_t* payload, unsigned payload_len, MonotonicTime tx_deadline,
                         MonotonicTime blocking_deadline, TransferType transfer_type, NodeID dst_node_id,
                         TransferID tid, bool force_std_can) const
{
    Frame frame(data_type_id_, transfer_type, dispatcher_.getNodeID(), dst_node_id, tid, !force_std_can && dispatcher_.isCanFdEnabled());

    frame.setPriority(priority_);
    frame.setStartOfTransfer(true);

    UAVCAN_TRACE("TransferSender", "%s", frame.toString().c_str());

    /*
     * Checking if we're allowed to send.
     * In passive mode we can send only anonymous transfers, if they are enabled.
     */
    if (dispatcher_.isPassiveMode())
    {
        const bool allow = allow_anonymous_transfers_ &&
                           (transfer_type == TransferTypeMessageBroadcast) &&
                           (int(payload_len) <= frame.getPayloadCapacity());
        if (!allow)
        {
            return -ErrPassiveMode;
        }
    }

    dispatcher_.getTransferPerfCounter().addTxTransfer();

    /*
     * Sending frames
     */
    if (frame.getPayloadCapacity() >= payload_len)           // Single Frame Transfer
    {
        const int res = frame.setPayload(payload, payload_len);
        if (res != int(payload_len))
        {
            UAVCAN_ASSERT(0);
            UAVCAN_TRACE("TransferSender", "Frame payload write failure, %i", res);
            registerError();
            return (res < 0) ? res : -ErrLogic;
        }

        frame.setEndOfTransfer(true);
        UAVCAN_ASSERT(frame.isStartOfTransfer() && frame.isEndOfTransfer() && !frame.getToggle());

        const CanIOFlags flags = frame.getSrcNodeID().isUnicast() ? flags_ : (flags_ | CanIOFlagAbortOnError);

        return dispatcher_.send(frame, tx_deadline, blocking_deadline, qos_, flags, iface_mask_);
    }
    else                                                   // Multi Frame Transfer
    {
        UAVCAN_ASSERT(!dispatcher_.isPassiveMode());
        UAVCAN_ASSERT(frame.getSrcNodeID().isUnicast());

        int offset = 0;
        {
            TransferCRC crc = crc_base_;
            crc.add(payload, payload_len);
            // Take care of potential padding introduce in CANFD frames over 8Bytes
            if (payload_len > 63 && frame.isCanFDFrame()) {
                uint8_t empty = 0;
                uint8_t padding = CanFrame::getNumPaddingBytes(payload_len);
                for (uint8_t i=0; i<padding; i++) {
                    crc.add(&empty, 1);
                }
            }

            static const int BUFLEN = sizeof(static_cast<CanFrame*>(0)->data);
            uint8_t buf[BUFLEN];

            buf[0] = uint8_t(crc.get() & 0xFFU);       // Transfer CRC, little endian
            buf[1] = uint8_t((crc.get() >> 8) & 0xFF);
            (void)copy(payload, payload + BUFLEN - 2, buf + 2);

            const int write_res = frame.setPayload(buf, BUFLEN);
            if (write_res < 2)
            {
                UAVCAN_TRACE("TransferSender", "Frame payload write failure, %i", write_res);
                registerError();
                return write_res;
            }
            offset = write_res - 2;
            UAVCAN_ASSERT(int(payload_len) > offset);
        }

        int num_sent = 0;

        while (true)
        {
            const int send_res = dispatcher_.send(frame, tx_deadline, blocking_deadline, qos_, flags_, iface_mask_);
            if (send_res < 0)
            {
                registerError();
                return send_res;
            }

            num_sent++;
            if (frame.isEndOfTransfer())
            {
                return num_sent;  // Number of frames transmitted
            }

            frame.setStartOfTransfer(false);
            frame.flipToggle();

            UAVCAN_ASSERT(offset >= 0);
            const int write_res = frame.setPayload(payload + offset, payload_len - unsigned(offset));
            if (write_res < 0)
            {
                UAVCAN_TRACE("TransferSender", "Frame payload write failure, %i", write_res);
                registerError();
                return write_res;
            }

            offset += write_res;
            UAVCAN_ASSERT(offset <= int(payload_len));
            if (offset >= int(payload_len))
            {
                frame.setEndOfTransfer(true);
            }
        }
    }

    UAVCAN_ASSERT(0);
    return -ErrLogic; // Return path analysis is apparently broken. There should be no warning, this 'return' is unreachable.
}

int TransferSender::send(const uint8_t* payload, unsigned payload_len, MonotonicTime tx_deadline,
                         MonotonicTime blocking_deadline, TransferType transfer_type, NodeID dst_node_id,
                         bool force_std_can) const
{
    /*
     * TODO: TID is not needed for anonymous transfers, this part of the code can be skipped?
     */
    const OutgoingTransferRegistryKey otr_key(data_type_id_, transfer_type, dst_node_id);

    UAVCAN_ASSERT(!tx_deadline.isZero());
    const MonotonicTime otr_deadline = tx_deadline + max(max_transfer_interval_ * 2,
                                                         OutgoingTransferRegistry::MinEntryLifetime);

    TransferID* const tid = dispatcher_.getOutgoingTransferRegistry().accessOrCreate(otr_key, otr_deadline);
    if (tid == UAVCAN_NULLPTR)
    {
        UAVCAN_TRACE("TransferSender", "OTR access failure, dtid=%d tt=%i",
                     int(data_type_id_.get()), int(transfer_type));
        return -ErrMemory;
    }

    const TransferID this_tid = tid->get();
    tid->increment();

    return send(payload, payload_len, tx_deadline, blocking_deadline, transfer_type,
                dst_node_id, this_tid, force_std_can);
}

}
