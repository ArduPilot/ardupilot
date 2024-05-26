/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <algorithm>
#include <queue>
#include <vector>
#include <gtest/gtest.h>
#include <uavcan/transport/transfer_listener.hpp>

/**
 * UAVCAN transfer representation used in various tests.
 */
struct Transfer
{
    uavcan::MonotonicTime ts_monotonic;
    uavcan::UtcTime ts_utc;
    uavcan::TransferPriority priority;
    uavcan::TransferType transfer_type;
    uavcan::TransferID transfer_id;
    uavcan::NodeID src_node_id;
    uavcan::NodeID dst_node_id;
    uavcan::DataTypeDescriptor data_type;
    std::string payload;

    Transfer(const uavcan::IncomingTransfer& tr, const uavcan::DataTypeDescriptor& data_type)
        : ts_monotonic(tr.getMonotonicTimestamp())
        , ts_utc(tr.getUtcTimestamp())
        , priority(tr.getPriority())
        , transfer_type(tr.getTransferType())
        , transfer_id(tr.getTransferID())
        , src_node_id(tr.getSrcNodeID())
        , dst_node_id() // default is invalid
        , data_type(data_type)
    {
        unsigned offset = 0;
        while (true)
        {
            uint8_t buf[256];
            int res = tr.read(offset, buf, sizeof(buf));
            if (res < 0)
            {
                std::cout << "IncomingTransferContainer: read failure " << res << std::endl;
                exit(1);
            }
            if (res == 0)
            {
                break;
            }
            payload += std::string(reinterpret_cast<const char*>(buf), unsigned(res));
            offset += unsigned(res);
        }
    }

    Transfer(uavcan::MonotonicTime ts_monotonic, uavcan::UtcTime ts_utc, uavcan::TransferPriority priority,
             uavcan::TransferType transfer_type, uavcan::TransferID transfer_id, uavcan::NodeID src_node_id,
             uavcan::NodeID dst_node_id, const std::string& payload, const uavcan::DataTypeDescriptor& data_type)
        : ts_monotonic(ts_monotonic)
        , ts_utc(ts_utc)
        , priority(priority)
        , transfer_type(transfer_type)
        , transfer_id(transfer_id)
        , src_node_id(src_node_id)
        , dst_node_id(dst_node_id)
        , data_type(data_type)
        , payload(payload)
    { }

    Transfer(uint64_t ts_monotonic, uint64_t ts_utc, uavcan::TransferPriority priority,
             uavcan::TransferType transfer_type, uavcan::TransferID transfer_id, uavcan::NodeID src_node_id,
             uavcan::NodeID dst_node_id, const std::string& payload, const uavcan::DataTypeDescriptor& data_type)
        : ts_monotonic(uavcan::MonotonicTime::fromUSec(ts_monotonic))
        , ts_utc(uavcan::UtcTime::fromUSec(ts_utc))
        , priority(priority)
        , transfer_type(transfer_type)
        , transfer_id(transfer_id)
        , src_node_id(src_node_id)
        , dst_node_id(dst_node_id)
        , data_type(data_type)
        , payload(payload)
    { }

    bool operator==(const Transfer& rhs) const
    {
        return
            (ts_monotonic  == rhs.ts_monotonic) &&
            ((!ts_utc.isZero() && !rhs.ts_utc.isZero()) ? (ts_utc == rhs.ts_utc) : true) &&
            (priority      == rhs.priority) &&
            (transfer_type == rhs.transfer_type) &&
            (transfer_id   == rhs.transfer_id) &&
            (src_node_id   == rhs.src_node_id) &&
            ((dst_node_id.isValid() && rhs.dst_node_id.isValid()) ? (dst_node_id == rhs.dst_node_id) : true) &&
            (data_type     == rhs.data_type) &&
            (payload       == rhs.payload);
    }

    std::string toString() const
    {
        std::ostringstream os;
        os << "ts_m="    << ts_monotonic
           << " ts_utc=" << ts_utc
           << " prio="   << int(priority.get())
           << " tt="     << int(transfer_type)
           << " tid="    << int(transfer_id.get())
           << " snid="   << int(src_node_id.get())
           << " dnid="   << int(dst_node_id.get())
           << " dtid="   << int(data_type.getID().get())
           << "\n\t'" << payload << "'";
        return os.str();
    }
};

/**
 * This subscriber accepts any types of transfers - this makes testing easier.
 * In reality, uavcan::TransferListener should accept only specific transfer types
 * which are dispatched/filtered by uavcan::Dispatcher.
 */
class TestListener : public uavcan::TransferListener
{
    typedef uavcan::TransferListener Base;

    std::queue<Transfer> transfers_;

public:
    TestListener(uavcan::TransferPerfCounter& perf, const uavcan::DataTypeDescriptor& data_type,
                 uavcan::uint16_t max_buffer_size, uavcan::IPoolAllocator& allocator)
        : Base(perf, data_type, max_buffer_size, allocator)
    { }

    void handleIncomingTransfer(uavcan::IncomingTransfer& transfer)
    {
        const Transfer rx(transfer, Base::getDataTypeDescriptor());
        transfers_.push(rx);
        std::cout << "Received transfer: " << rx.toString() << std::endl;

        const bool single_frame = dynamic_cast<uavcan::SingleFrameIncomingTransfer*>(&transfer) != UAVCAN_NULLPTR;

        const bool anonymous = single_frame &&
                               transfer.getSrcNodeID().isBroadcast() &&
                               (transfer.getTransferType() == uavcan::TransferTypeMessageBroadcast);

        ASSERT_EQ(anonymous, transfer.isAnonymousTransfer());
    }

    bool matchAndPop(const Transfer& reference)
    {
        if (transfers_.empty())
        {
            std::cout << "No received transfers" << std::endl;
            return false;
        }

        const Transfer tr = transfers_.front();
        transfers_.pop();

        const bool res = (tr == reference);
        if (!res)
        {
            std::cout << "TestSubscriber: Transfer mismatch:\n"
                      << "Expected: " << reference.toString() << "\n"
                      << "Received: " << tr.toString() << std::endl;
        }
        return res;
    }

    unsigned getNumReceivedTransfers() const { return static_cast<unsigned>(transfers_.size()); }
    bool isEmpty() const { return transfers_.empty(); }
};


namespace
{

std::vector<uavcan::RxFrame> serializeTransfer(const Transfer& transfer)
{
    const bool need_crc = transfer.payload.length() > (sizeof(uavcan::CanFrame::data) - 1);

    std::vector<uint8_t> raw_payload;
    if (need_crc)
    {
        uavcan::TransferCRC payload_crc = transfer.data_type.getSignature().toTransferCRC();
        payload_crc.add(reinterpret_cast<const uint8_t*>(transfer.payload.c_str()), uint16_t(transfer.payload.length()));
        // Little endian
        raw_payload.push_back(uint8_t(payload_crc.get() & 0xFF));
        raw_payload.push_back(uint8_t((payload_crc.get() >> 8) & 0xFF));
    }
    raw_payload.insert(raw_payload.end(), transfer.payload.begin(), transfer.payload.end());

    std::vector<uavcan::RxFrame> output;
    unsigned offset = 0;
    uavcan::MonotonicTime ts_monotonic = transfer.ts_monotonic;
    uavcan::UtcTime ts_utc = transfer.ts_utc;

    uavcan::Frame frm(transfer.data_type.getID(), transfer.transfer_type, transfer.src_node_id,
                      transfer.dst_node_id, transfer.transfer_id);
    frm.setStartOfTransfer(true);
    frm.setPriority(transfer.priority);

    while (true)
    {
        const int bytes_left = int(raw_payload.size()) - int(offset);
        EXPECT_TRUE(bytes_left >= 0);

        const int spres = frm.setPayload(&*(raw_payload.begin() + offset), unsigned(bytes_left));
        if (spres < 0)
        {
            std::cerr << ">_<" << std::endl;
            std::exit(1);
        }
        if (spres == bytes_left)
        {
            frm.setEndOfTransfer(true);
        }

        offset += unsigned(spres);

        const uavcan::RxFrame rxfrm(frm, ts_monotonic, ts_utc, 0);
        ts_monotonic += uavcan::MonotonicDuration::fromUSec(1);
        ts_utc += uavcan::UtcDuration::fromUSec(1);

        output.push_back(rxfrm);
        if (frm.isEndOfTransfer())
        {
            break;
        }

        frm.setStartOfTransfer(false);
        frm.flipToggle();
    }
    return output;
}

inline uavcan::DataTypeDescriptor makeDataType(uavcan::DataTypeKind kind, uint16_t id, const char* name = "")
{
    const uavcan::DataTypeSignature signature((uint64_t(kind) << 16) | uint16_t(id << 8) | uint16_t(id & 0xFF));
    return uavcan::DataTypeDescriptor(kind, id, signature, name);
}

}


class IncomingTransferEmulatorBase
{
    uavcan::MonotonicTime ts_;
    uavcan::TransferID tid_;
    uavcan::NodeID dst_node_id_;

public:
    IncomingTransferEmulatorBase(uavcan::NodeID dst_node_id)
        : dst_node_id_(dst_node_id)
    { }

    virtual ~IncomingTransferEmulatorBase() { }

    Transfer makeTransfer(uavcan::TransferPriority priority, uavcan::TransferType transfer_type,
                          uint8_t source_node_id, const std::string& payload, const uavcan::DataTypeDescriptor& type,
                          uavcan::NodeID dst_node_id_override = uavcan::NodeID())
    {
        ts_ += uavcan::MonotonicDuration::fromUSec(100);
        const uavcan::UtcTime utc = uavcan::UtcTime::fromUSec(ts_.toUSec() + 1000000000ul);
        const uavcan::NodeID dst_node_id = (transfer_type == uavcan::TransferTypeMessageBroadcast) ?
                                           uavcan::NodeID::Broadcast :
                                           (dst_node_id_override.isValid() ? dst_node_id_override : dst_node_id_);
        const Transfer tr(ts_, utc, priority, transfer_type, tid_, source_node_id, dst_node_id, payload, type);
        tid_.increment();
        return tr;
    }

    virtual void sendOneFrame(const uavcan::RxFrame& frame) = 0;

    void send(const std::vector<std::vector<uavcan::RxFrame> >& sers)
    {
        unsigned index = 0;
        while (true)
        {
            // Sending all transfers concurrently
            bool all_empty = true;
            for (std::vector<std::vector<uavcan::RxFrame> >::const_iterator it = sers.begin(); it != sers.end(); ++it)
            {
                if (it->size() <= index)
                {
                    continue;
                }
                all_empty = false;
                std::cout << "Incoming Transfer Emulator: Sending: " << it->at(index).toString() << std::endl;
                sendOneFrame(it->at(index));
            }
            index++;
            if (all_empty)
            {
                break;
            }
        }
    }

    void send(const Transfer* transfers, unsigned num_transfers)
    {
        std::vector<std::vector<uavcan::RxFrame> > sers;
        while (num_transfers--)
        {
            sers.push_back(serializeTransfer(*transfers++));
        }
        send(sers);
    }

    template <int SIZE> void send(const Transfer (&transfers)[SIZE]) { send(transfers, SIZE); }
};

/**
 * Zero allocator - always fails
 */
class NullAllocator : public uavcan::IPoolAllocator
{
public:
    virtual void* allocate(std::size_t) { return UAVCAN_NULLPTR; }
    virtual void deallocate(const void*) { }
    virtual uint16_t getBlockCapacity() const { return 0; }
};
