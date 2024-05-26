/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_TRANSPORT_FRAME_HPP_INCLUDED
#define UAVCAN_TRANSPORT_FRAME_HPP_INCLUDED

#include <cassert>
#include <uavcan/transport/transfer.hpp>
#include <uavcan/transport/can_io.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/data_type.hpp>

namespace uavcan
{

class UAVCAN_EXPORT Frame
{
#if UAVCAN_SUPPORT_CANFD
    enum { NonFDCANPayloadCapacity = 7,
           PayloadCapacity = 63 };
#else
    enum { PayloadCapacity = 7 };
#endif
    uint8_t payload_[PayloadCapacity];
    TransferPriority transfer_priority_;
    TransferType transfer_type_;
    DataTypeID data_type_id_;
    uint_fast8_t payload_len_;
    NodeID src_node_id_;
    NodeID dst_node_id_;
    TransferID transfer_id_;
    bool start_of_transfer_;
    bool end_of_transfer_;
    bool toggle_;
    bool canfd_frame_;

public:
    Frame() :
        transfer_type_(TransferType(NumTransferTypes)),                // Invalid value
        payload_len_(0),
        start_of_transfer_(false),
        end_of_transfer_(false),
        toggle_(false),
        canfd_frame_(false)
    { }

    Frame(DataTypeID data_type_id,
          TransferType transfer_type,
          NodeID src_node_id,
          NodeID dst_node_id,
          TransferID transfer_id,
          bool canfd_frame = false) :
        transfer_priority_(TransferPriority::Default),
        transfer_type_(transfer_type),
        data_type_id_(data_type_id),
        payload_len_(0),
        src_node_id_(src_node_id),
        dst_node_id_(dst_node_id),
        transfer_id_(transfer_id),
        start_of_transfer_(false),
        end_of_transfer_(false),
        toggle_(false),
        canfd_frame_(canfd_frame)
    {
        UAVCAN_ASSERT((transfer_type == TransferTypeMessageBroadcast) == dst_node_id.isBroadcast());
        UAVCAN_ASSERT(data_type_id.isValidForDataTypeKind(getDataTypeKindForTransferType(transfer_type)));
        UAVCAN_ASSERT(src_node_id.isUnicast() ? (src_node_id != dst_node_id) : true);
    }

    void setPriority(TransferPriority priority) { transfer_priority_ = priority; }
    TransferPriority getPriority() const { return transfer_priority_; }

    /**
     * Max payload length depends on the transfer type and frame index.
     */
#if UAVCAN_SUPPORT_CANFD
    uint8_t getPayloadCapacity() const { return canfd_frame_?PayloadCapacity:NonFDCANPayloadCapacity; }
#else
    uint8_t getPayloadCapacity() const { return PayloadCapacity; }
#endif
    uint8_t setPayload(const uint8_t* data, unsigned len);

    void setCanFD(bool set) { canfd_frame_ = set; }

    bool isCanFDFrame() const { return canfd_frame_; }

    unsigned getPayloadLen() const { return payload_len_; }
    const uint8_t* getPayloadPtr() const { return payload_; }

    TransferType getTransferType() const { return transfer_type_; }
    DataTypeID getDataTypeID()     const { return data_type_id_; }
    NodeID getSrcNodeID()          const { return src_node_id_; }
    NodeID getDstNodeID()          const { return dst_node_id_; }
    TransferID getTransferID()     const { return transfer_id_; }

    void setStartOfTransfer(bool x) { start_of_transfer_ = x; }
    void setEndOfTransfer(bool x)   { end_of_transfer_ = x; }

    bool isStartOfTransfer() const { return start_of_transfer_; }
    bool isEndOfTransfer()   const { return end_of_transfer_; }

    void flipToggle() { toggle_ = !toggle_; }
    bool getToggle() const { return toggle_; }

    bool parse(const CanFrame& can_frame);
    bool compile(CanFrame& can_frame) const;

    bool isValid() const;

    bool operator!=(const Frame& rhs) const { return !operator==(rhs); }
    bool operator==(const Frame& rhs) const;

#if UAVCAN_TOSTRING
    std::string toString() const;
#endif
};


class UAVCAN_EXPORT RxFrame : public Frame
{
    MonotonicTime ts_mono_;
    UtcTime ts_utc_;
    uint8_t iface_index_;

public:
    RxFrame()
        : iface_index_(0)
    { }

    RxFrame(const Frame& frame, MonotonicTime ts_mono, UtcTime ts_utc, uint8_t iface_index)
        : ts_mono_(ts_mono)
        , ts_utc_(ts_utc)
        , iface_index_(iface_index)
    {
        *static_cast<Frame*>(this) = frame;
    }

    bool parse(const CanRxFrame& can_frame);

    /**
     * Can't be zero.
     */
    MonotonicTime getMonotonicTimestamp() const { return ts_mono_; }

    /**
     * Can be zero if not supported by the platform driver.
     */
    UtcTime getUtcTimestamp() const { return ts_utc_; }

    uint8_t getIfaceIndex() const { return iface_index_; }

#if UAVCAN_TOSTRING
    std::string toString() const;
#endif
};

}

#endif // UAVCAN_TRANSPORT_FRAME_HPP_INCLUDED
