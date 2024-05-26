/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/data_type.hpp>
#include <uavcan/transport/crc.hpp>
#include <cstring>
#include <cassert>

namespace uavcan
{
/*
 * DataTypeID
 */
const uint16_t DataTypeID::MaxServiceDataTypeIDValue;
const uint16_t DataTypeID::MaxMessageDataTypeIDValue;
const uint16_t DataTypeID::MaxPossibleDataTypeIDValue;

DataTypeID DataTypeID::getMaxValueForDataTypeKind(const DataTypeKind dtkind)
{
    if (dtkind == DataTypeKindService)
    {
        return MaxServiceDataTypeIDValue;
    }
    else if (dtkind == DataTypeKindMessage)
    {
        return MaxMessageDataTypeIDValue;
    }
    else
    {
        UAVCAN_ASSERT(0);
        return DataTypeID(0);
    }
}

/*
 * DataTypeSignatureCRC
 */
DataTypeSignatureCRC DataTypeSignatureCRC::extend(uint64_t crc)
{
    DataTypeSignatureCRC ret;
    ret.crc_ = crc ^ 0xFFFFFFFFFFFFFFFF;
    return ret;
}

void DataTypeSignatureCRC::add(uint8_t byte)
{
    static const uint64_t Poly = 0x42F0E1EBA9EA3693;
    crc_ ^= uint64_t(byte) << 56;
    for (int i = 0; i < 8; i++)
    {
        crc_ = (crc_ & (uint64_t(1) << 63)) ? (crc_ << 1) ^ Poly : crc_ << 1;
    }
}

void DataTypeSignatureCRC::add(const uint8_t* bytes, unsigned len)
{
    UAVCAN_ASSERT(bytes);
    while (len--)
    {
        add(*bytes++);
    }
}

/*
 * DataTypeSignature
 */
void DataTypeSignature::mixin64(uint64_t x)
{
    DataTypeSignatureCRC crc = DataTypeSignatureCRC::extend(value_);
    for (int i = 0; i < 64; i += 8)   // LSB first
    {
        crc.add((x >> i) & 0xFF);
    }
    value_ = crc.get();
}

void DataTypeSignature::extend(DataTypeSignature dts)
{
    const uint64_t y = value_;
    mixin64(dts.get());
    mixin64(y);
}

TransferCRC DataTypeSignature::toTransferCRC() const
{
    TransferCRC tcrc;
    for (int i = 0; i < 64; i += 8)    // LSB first
    {
        tcrc.add((value_ >> i) & 0xFF);
    }
    return tcrc;
}

/*
 * DataTypeDescriptor
 */
const unsigned DataTypeDescriptor::MaxFullNameLen;

bool DataTypeDescriptor::isValid() const
{
    return id_.isValidForDataTypeKind(kind_) &&
           (full_name_ != UAVCAN_NULLPTR) &&
           (*full_name_ != '\0');
}

bool DataTypeDescriptor::match(DataTypeKind kind, const char* name) const
{
    return (kind_ == kind) && !std::strncmp(full_name_, name, MaxFullNameLen);
}

bool DataTypeDescriptor::match(DataTypeKind kind, DataTypeID id) const
{
    return (kind_ == kind) && (id_ == id);
}

#if UAVCAN_TOSTRING
std::string DataTypeDescriptor::toString() const
{
    char kindch = '?';
    switch (kind_)
    {
    case DataTypeKindMessage:
    {
        kindch = 'm';
        break;
    }
    case DataTypeKindService:
    {
        kindch = 's';
        break;
    }
    default:
    {
        UAVCAN_ASSERT(0);
        break;
    }
    }

    char buf[128];
    (void)snprintf(buf, sizeof(buf), "%s:%u%c:%016llx",
                   full_name_, static_cast<unsigned>(id_.get()), kindch,
                   static_cast<unsigned long long>(signature_.get()));
    return std::string(buf);
}
#endif

bool DataTypeDescriptor::operator==(const DataTypeDescriptor& rhs) const
{
    return
        (kind_ == rhs.kind_) &&
        (id_ == rhs.id_) &&
        (signature_ == rhs.signature_) &&
        !std::strncmp(full_name_, rhs.full_name_, MaxFullNameLen);
}

}
