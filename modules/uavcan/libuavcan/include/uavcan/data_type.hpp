/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_DATA_TYPE_HPP_INCLUDED
#define UAVCAN_DATA_TYPE_HPP_INCLUDED

#include <cassert>
#include <cstring>
#include <uavcan/std.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/transport/transfer.hpp>

namespace uavcan
{

class UAVCAN_EXPORT TransferCRC;

enum DataTypeKind
{
    DataTypeKindService,
    DataTypeKindMessage
};

static const uint8_t NumDataTypeKinds = 2;


static inline DataTypeKind getDataTypeKindForTransferType(const TransferType tt)
{
    if (tt == TransferTypeServiceResponse ||
        tt == TransferTypeServiceRequest)
    {
        return DataTypeKindService;
    }
    else if (tt == TransferTypeMessageBroadcast)
    {
        return DataTypeKindMessage;
    }
    else
    {
        UAVCAN_ASSERT(0);
        return DataTypeKind(0);
    }
}


class UAVCAN_EXPORT DataTypeID
{
    uint32_t value_;

public:
    static const uint16_t MaxServiceDataTypeIDValue = 255;
    static const uint16_t MaxMessageDataTypeIDValue = 65535;
    static const uint16_t MaxPossibleDataTypeIDValue = MaxMessageDataTypeIDValue;

    DataTypeID() : value_(0xFFFFFFFFUL) { }

    DataTypeID(uint16_t id)  // Implicit
        : value_(id)
    { }

    static DataTypeID getMaxValueForDataTypeKind(const DataTypeKind dtkind);

    bool isValidForDataTypeKind(DataTypeKind dtkind) const
    {
        return value_ <= getMaxValueForDataTypeKind(dtkind).get();
    }

    uint16_t get() const { return static_cast<uint16_t>(value_); }

    bool operator==(DataTypeID rhs) const { return value_ == rhs.value_; }
    bool operator!=(DataTypeID rhs) const { return value_ != rhs.value_; }

    bool operator<(DataTypeID rhs) const { return value_ < rhs.value_; }
    bool operator>(DataTypeID rhs) const { return value_ > rhs.value_; }
    bool operator<=(DataTypeID rhs) const { return value_ <= rhs.value_; }
    bool operator>=(DataTypeID rhs) const { return value_ >= rhs.value_; }
};


/**
 * CRC-64-WE
 * Description: http://reveng.sourceforge.net/crc-catalogue/17plus.htm#crc.cat-bits.64
 * Initial value: 0xFFFFFFFFFFFFFFFF
 * Poly: 0x42F0E1EBA9EA3693
 * Reverse: no
 * Output xor: 0xFFFFFFFFFFFFFFFF
 * Check: 0x62EC59E3F1A4F00A
 */
class UAVCAN_EXPORT DataTypeSignatureCRC
{
    uint64_t crc_;

public:
    static DataTypeSignatureCRC extend(uint64_t crc);

    DataTypeSignatureCRC() : crc_(0xFFFFFFFFFFFFFFFFULL) { }

    void add(uint8_t byte);

    void add(const uint8_t* bytes, unsigned len);

    uint64_t get() const { return crc_ ^ 0xFFFFFFFFFFFFFFFFULL; }
};


class UAVCAN_EXPORT DataTypeSignature
{
    uint64_t value_;

    void mixin64(uint64_t x);

public:
    DataTypeSignature() : value_(0) { }
    explicit DataTypeSignature(uint64_t value) : value_(value) { }

    void extend(DataTypeSignature dts);

    TransferCRC toTransferCRC() const;

    uint64_t get() const { return value_; }

    bool operator==(DataTypeSignature rhs) const { return value_ == rhs.value_; }
    bool operator!=(DataTypeSignature rhs) const { return !operator==(rhs); }
};

/**
 * This class contains complete description of a data type.
 */
class UAVCAN_EXPORT DataTypeDescriptor
{
    DataTypeSignature signature_;
    const char* full_name_;
    DataTypeKind kind_;
    DataTypeID id_;

public:
    static const unsigned MaxFullNameLen = 80;

    DataTypeDescriptor() :
        full_name_(""),
        kind_(DataTypeKind(0))
    { }

    DataTypeDescriptor(DataTypeKind kind, DataTypeID id, const DataTypeSignature& signature, const char* name) :
        signature_(signature),
        full_name_(name),
        kind_(kind),
        id_(id)
    {
        UAVCAN_ASSERT((kind == DataTypeKindMessage) || (kind == DataTypeKindService));
        UAVCAN_ASSERT(name);
        UAVCAN_ASSERT(std::strlen(name) <= MaxFullNameLen);
    }

    bool isValid() const;

    DataTypeKind getKind() const { return kind_; }
    DataTypeID getID() const { return id_; }
    const DataTypeSignature& getSignature() const { return signature_; }
    const char* getFullName() const { return full_name_; }

    bool match(DataTypeKind kind, const char* name) const;
    bool match(DataTypeKind kind, DataTypeID id) const;

    bool operator!=(const DataTypeDescriptor& rhs) const { return !operator==(rhs); }
    bool operator==(const DataTypeDescriptor& rhs) const;

#if UAVCAN_TOSTRING
    std::string toString() const;
#endif
};

}

#endif // UAVCAN_DATA_TYPE_HPP_INCLUDED
