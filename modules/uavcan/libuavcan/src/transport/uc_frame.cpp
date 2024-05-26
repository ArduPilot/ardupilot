/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/transport/frame.hpp>
#include <uavcan/transport/can_io.hpp>
#include <uavcan/transport/crc.hpp>
#include <uavcan/debug.hpp>
#include <cassert>

namespace uavcan
{
/**
 * Frame
 */
uint8_t Frame::setPayload(const uint8_t* data, unsigned len)
{
    const uint8_t maxlen = getPayloadCapacity();
    len = min(unsigned(maxlen), len);
    (void)copy(data, data + len, payload_);
    payload_len_ = uint_fast8_t(len);
    return static_cast<uint8_t>(len);
}

template <int OFFSET, int WIDTH>
inline static uint32_t bitunpack(uint32_t val)
{
    StaticAssert<(OFFSET >= 0)>::check();
    StaticAssert<(WIDTH > 0)>::check();
    StaticAssert<((OFFSET + WIDTH) <= 29)>::check();
    return (val >> OFFSET) & ((1UL << WIDTH) - 1);
}

bool Frame::parse(const CanFrame& can_frame)
{
    if (can_frame.isErrorFrame() || can_frame.isRemoteTransmissionRequest() || !can_frame.isExtended())
    {
        UAVCAN_TRACE("Frame", "Parsing failed at line %d", __LINE__);
        return false;
    }

    if (can_frame.dlc > sizeof(can_frame.data))
    {
        UAVCAN_ASSERT(0);  // This is not a protocol error, so UAVCAN_ASSERT() is ok
        return false;
    }

    if (can_frame.dlc < 1)
    {
        UAVCAN_TRACE("Frame", "Parsing failed at line %d", __LINE__);
        return false;
    }

    canfd_frame_ = can_frame.canfd;

    /*
     * CAN ID parsing
     */
    const uint32_t id = can_frame.id & CanFrame::MaskExtID;

    transfer_priority_ = static_cast<uint8_t>(bitunpack<24, 5>(id));
    src_node_id_ = static_cast<uint8_t>(bitunpack<0, 7>(id));

    const bool service_not_message = bitunpack<7, 1>(id) != 0U;
    if (service_not_message)
    {
        const bool request_not_response = bitunpack<15, 1>(id) != 0U;
        transfer_type_ = request_not_response ? TransferTypeServiceRequest : TransferTypeServiceResponse;

        dst_node_id_ = static_cast<uint8_t>(bitunpack<8, 7>(id));
        data_type_id_ = static_cast<uint16_t>(bitunpack<16, 8>(id));
    }
    else
    {
        transfer_type_ = TransferTypeMessageBroadcast;
        dst_node_id_ = NodeID::Broadcast;

        data_type_id_ = static_cast<uint16_t>(bitunpack<8, 16>(id));

        if (src_node_id_.isBroadcast())
        {
            // Removing the discriminator
            data_type_id_ = static_cast<uint16_t>(data_type_id_.get() & 3U);
        }
    }

    /*
     * CAN payload parsing
     */
    payload_len_ = static_cast<uint8_t>(CanFrame::dlcToDataLength(can_frame.dlc) - 1U);
    (void)copy(can_frame.data, can_frame.data + payload_len_, payload_);

    const uint8_t tail = can_frame.data[CanFrame::dlcToDataLength(can_frame.dlc) - 1U];

    start_of_transfer_ = (tail & (1U << 7)) != 0;
    end_of_transfer_   = (tail & (1U << 6)) != 0;
    toggle_            = (tail & (1U << 5)) != 0;

    transfer_id_ = tail & TransferID::Max;

    return isValid();
}

template <int OFFSET, int WIDTH>
inline static uint32_t bitpack(uint32_t field)
{
    StaticAssert<(OFFSET >= 0)>::check();
    StaticAssert<(WIDTH > 0)>::check();
    StaticAssert<((OFFSET + WIDTH) <= 29)>::check();
    UAVCAN_ASSERT((field & ((1UL << WIDTH) - 1)) == field);
    return uint32_t((field & ((1UL << WIDTH) - 1)) << OFFSET);
}

bool Frame::compile(CanFrame& out_can_frame) const
{
    if (!isValid())
    {
        UAVCAN_ASSERT(0);        // This is an application error, so we need to maximize it.
        return false;
    }

    /*
     * CAN ID field
     */
    out_can_frame.id = CanFrame::FlagEFF |
        bitpack<0, 7>(src_node_id_.get()) |
        bitpack<24, 5>(transfer_priority_.get());

    if (transfer_type_ == TransferTypeMessageBroadcast)
    {
        out_can_frame.id |=
            bitpack<7, 1>(0U) |
            bitpack<8, 16>(data_type_id_.get());
    }
    else
    {
        const bool request_not_response = transfer_type_ == TransferTypeServiceRequest;
        out_can_frame.id |=
            bitpack<7, 1>(1U) |
            bitpack<8, 7>(dst_node_id_.get()) |
            bitpack<15, 1>(request_not_response ? 1U : 0U) |
            bitpack<16, 8>(data_type_id_.get());
    }

    /*
     * Payload
     */
    uint8_t tail = transfer_id_.get();
    if (start_of_transfer_)
    {
        tail |= (1U << 7);
    }
    if (end_of_transfer_)
    {
        tail |= (1U << 6);
    }
    if (toggle_)
    {
        tail |= (1U << 5);
    }

    UAVCAN_ASSERT(payload_len_ < sizeof(static_cast<CanFrame*>(UAVCAN_NULLPTR)->data));

    out_can_frame.dlc = CanFrame::dataLengthToDlc(static_cast<uint8_t>(payload_len_));
    (void)copy(payload_, payload_ + payload_len_, out_can_frame.data);

    if (payload_len_ < 8) {
        out_can_frame.data[payload_len_] = tail;
        out_can_frame.dlc++;
    } else if (payload_len_ == CanFrame::dlcToDataLength(out_can_frame.dlc)){
        out_can_frame.dlc++;
        out_can_frame.data[CanFrame::dlcToDataLength(out_can_frame.dlc) - 1] = tail;
    } else {
        out_can_frame.data[CanFrame::dlcToDataLength(out_can_frame.dlc) - 1] = tail;
    }

    out_can_frame.canfd = canfd_frame_;

    /*
     * Discriminator
     */
    if (src_node_id_.isBroadcast())
    {
        TransferCRC crc;
        crc.add(out_can_frame.data, out_can_frame.dlc);
        out_can_frame.id |= bitpack<10, 14>(crc.get() & ((1U << 14) - 1U));
    }

    return true;
}

bool Frame::isValid() const
{
    /*
     * Toggle
     */
    if (start_of_transfer_ && toggle_)
    {
        UAVCAN_TRACE("Frame", "Validness check failed at line %d", __LINE__);
        return false;
    }

    /*
     * Node ID
     */
    if (!src_node_id_.isValid() || !dst_node_id_.isValid())
    {
        UAVCAN_TRACE("Frame", "Validness check failed at line %d", __LINE__);
        return false;
    }

    if (src_node_id_.isUnicast() && (src_node_id_ == dst_node_id_))
    {
        UAVCAN_TRACE("Frame", "Validness check failed at line %d", __LINE__);
        return false;
    }

    /*
     * Transfer type
     */
    if (transfer_type_ >= NumTransferTypes)
    {
        UAVCAN_TRACE("Frame", "Validness check failed at line %d", __LINE__);
        return false;
    }

    if ((transfer_type_ == TransferTypeMessageBroadcast) != dst_node_id_.isBroadcast())
    {
        UAVCAN_TRACE("Frame", "Validness check failed at line %d", __LINE__);
        return false;
    }

    // Anonymous transfers
    if (src_node_id_.isBroadcast() &&
        (!start_of_transfer_ || !end_of_transfer_ || (transfer_type_ != TransferTypeMessageBroadcast)))
    {
        UAVCAN_TRACE("Frame", "Validness check failed at line %d", __LINE__);
        return false;
    }

    /*
     * Payload
     */
    if (payload_len_ > getPayloadCapacity())
    {
        UAVCAN_TRACE("Frame", "Validness check failed at line %d", __LINE__);
        return false;
    }

    /*
     * Data type ID
     */
    if (!data_type_id_.isValidForDataTypeKind(getDataTypeKindForTransferType(transfer_type_)))
    {
        UAVCAN_TRACE("Frame", "Validness check failed at line %d", __LINE__);
        return false;
    }

    /*
     * Priority
     */
    if (!transfer_priority_.isValid())
    {
        UAVCAN_TRACE("Frame", "Validness check failed at line %d", __LINE__);
        return false;
    }

    return true;
}

bool Frame::operator==(const Frame& rhs) const
{
    return
        (transfer_priority_ == rhs.transfer_priority_) &&
        (transfer_type_     == rhs.transfer_type_) &&
        (data_type_id_      == rhs.data_type_id_) &&
        (src_node_id_       == rhs.src_node_id_) &&
        (dst_node_id_       == rhs.dst_node_id_) &&
        (transfer_id_       == rhs.transfer_id_) &&
        (toggle_            == rhs.toggle_) &&
        (start_of_transfer_ == rhs.start_of_transfer_) &&
        (end_of_transfer_   == rhs.end_of_transfer_) &&
        (payload_len_       == rhs.payload_len_) &&
        equal(payload_, payload_ + payload_len_, rhs.payload_);
}

#if UAVCAN_TOSTRING
std::string Frame::toString() const
{
    static const int BUFLEN = 100;
    char buf[BUFLEN];
    int ofs = snprintf(buf, BUFLEN, "prio=%d dtid=%d tt=%d snid=%d dnid=%d sot=%d eot=%d togl=%d tid=%d payload=[",
                       int(transfer_priority_.get()), int(data_type_id_.get()), int(transfer_type_),
                       int(src_node_id_.get()), int(dst_node_id_.get()),
                       int(start_of_transfer_), int(end_of_transfer_), int(toggle_), int(transfer_id_.get()));

    for (unsigned i = 0; i < payload_len_; i++)
    {
        // Coverity Scan complains about payload_ being not default initialized. This is OK.
        // coverity[read_parm_fld]
        ofs += snprintf(buf + ofs, unsigned(BUFLEN - ofs), "%02x", payload_[i]);
        if ((i + 1) < payload_len_)
        {
            ofs += snprintf(buf + ofs, unsigned(BUFLEN - ofs), " ");
        }
    }
    (void)snprintf(buf + ofs, unsigned(BUFLEN - ofs), "]");
    return std::string(buf);
}
#endif

/**
 * RxFrame
 */
bool RxFrame::parse(const CanRxFrame& can_frame)
{
    if (!Frame::parse(can_frame))
    {
        return false;
    }
    if (can_frame.ts_mono.isZero())  // Monotonic timestamps are mandatory.
    {
        UAVCAN_ASSERT(0);                   // If it is not set, it's a driver failure.
        return false;
    }
    ts_mono_ = can_frame.ts_mono;
    ts_utc_ = can_frame.ts_utc;
    iface_index_ = can_frame.iface_index;
    return true;
}

#if UAVCAN_TOSTRING
std::string RxFrame::toString() const
{
    std::string out = Frame::toString();
    out.reserve(128);
    out += " ts_m="   + ts_mono_.toString();
    out += " ts_utc=" + ts_utc_.toString();
    out += " iface=";
    out += char('0' + iface_index_);
    return out;
}
#endif

}
