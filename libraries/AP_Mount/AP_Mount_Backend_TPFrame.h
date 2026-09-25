/*
  Shared base for gimbal backends speaking the "#TP"/"#tp" wire framing used
  by at least two independent products: Topotek's own gimbal line (see
  AP_Mount_Topotek) and SkyDroid's OEM'd gimbal family (see AP_Mount_SkyDroid).
  Neither product's protocol document ever names or expands what "TP" stands
  for - it appears only as the literal 3-byte frame marker itself.  This
  class (and its name) is therefore built around that marker, not around
  either company's name, since the marker is shared by multiple products
  regardless of which one originated it.

  Packet format common to both products (each subclass documents its own
  command-identifier set and AddressByte values, which do differ):

  -------------------------------------------------------------------------------------------
  Field                 Index   Bytes       Description
  -------------------------------------------------------------------------------------------
  Frame Header          0       3           #TP (fixed length) or #tp (variable length)
  Address Bit           3       2           source address first, destination address second
  Data_Len              5       1           data length (hex nibble, max 0x0F)
  Control Bit           6       1           r -> query   w -> set/control
  Identification Bit    7       3           3 character command identifier
  Data                  10      Data_Len
  Check Bit                     2           sum of all preceding bytes, output as 2 ASCII hex
                                            characters (high nibble first)
 */

#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_TOPOTEK_ENABLED || HAL_MOUNT_SKYDROID_ENABLED

#include "AP_Mount_Backend_Serial.h"

// preamble layout is fixed by the protocol and identical for every product -
// see the packet-format table above
#define AP_MOUNT_TPFRAME_PACKETLEN_MIN   12   // packet length not including the data segment
#define AP_MOUNT_TPFRAME_MSGOFS_DATALEN  5    // data length, 1 ASCII hex nibble
#define AP_MOUNT_TPFRAME_MSGOFS_ID       7    // 3-character command identifier
#define AP_MOUNT_TPFRAME_MSGOFS_DATA     10   // start of the command-specific data segment
// large enough for the bigger of the two products' own PACKETLEN_MAX (Topotek's 36);
// each subclass's packetlen_max() enforces its own, possibly smaller, real limit
#define AP_MOUNT_TPFRAME_PACKETLEN_MAX   36

class AP_Mount_Backend_TPFrame : public AP_Mount_Backend_Serial
{

public:
    // inherit constructor
    using AP_Mount_Backend_Serial::AP_Mount_Backend_Serial;

    // Do not allow copies
    CLASS_NO_COPY(AP_Mount_Backend_TPFrame);

protected:

    // header type (fixed or variable length)
    // first three bytes of packet determined by this value
    enum class HeaderType : uint8_t {
        FIXED_LEN = 0x00,       // #TP will be sent
        VARIABLE_LEN = 0x01,    // #tp will be sent
    };

    // control byte (read or write)
    // sent as 7th byte of packet
    enum class ControlByte : uint8_t {
        READ = 114,     // 'r'
        WRITE = 119,    // 'w'
    };

    // parsing state.  Preamble states are shared by both products; a product
    // needing extra states beyond WAITING_FOR_DATA would extend this, though
    // neither does today
    enum class ParseState : uint8_t {
        WAITING_FOR_HEADER1 = 0,// #
        WAITING_FOR_HEADER2,    // T or t
        WAITING_FOR_HEADER3,    // P or p
        WAITING_FOR_ADDR1,      // source address
        WAITING_FOR_ADDR2,      // destination address
        WAITING_FOR_DATALEN,
        WAITING_FOR_CONTROL,    // r or w
        WAITING_FOR_ID1,        // e.g. 'G'
        WAITING_FOR_ID2,        // e.g. 'A'
        WAITING_FOR_ID3,        // e.g. 'C'
        WAITING_FOR_DATA,       // normally hex numbers in char form (e.g. '0A')
        WAITING_FOR_CRC_LOW,
        WAITING_FOR_CRC_HIGH,
    };

    // identifier bytes
    typedef char Identifier[3];

    // reading incoming packets from gimbal and confirm they are of the correct
    // format.  Calls handle_message() once a packet's CRC has been verified
    void read_incoming_packets();

    // called once a complete, CRC-verified packet has been received.
    // _msg_buff/_msg_buff_len describe it; msg_id points at its 3-character
    // command ID (AP_MOUNT_TPFRAME_MSGOFS_ID into _msg_buff) for convenience
    virtual void handle_message(const char* msg_id) = 0;

    // maximum number of bytes in a packet sent to or received from the gimbal.
    // Must not exceed AP_MOUNT_TPFRAME_PACKETLEN_MAX (the size of _msg_buff)
    virtual uint8_t packetlen_max() const = 0;

    // data segment length can be no more than this
    uint8_t datalen_max() const { return packetlen_max() - AP_MOUNT_TPFRAME_PACKETLEN_MIN; }

    // true if b is a valid destination/source address byte for this product -
    // each product's own AddressByte enum defines its actual address set
    virtual bool is_valid_address_byte(uint8_t b) const = 0;

    // return the address byte to send as the source of our own outgoing
    // packets.  Each product's protocol document defines its own rule for
    // this (e.g. whether UART vs network-attached connections use different
    // source addresses), so there is no shared default
    virtual uint8_t source_address_byte() const = 0;

    // calculate checksum
    uint8_t calculate_crc(const uint8_t *cmd, uint8_t len) const;

    // hexadecimal to character conversion
    uint8_t hex2char(uint8_t data) const;

    // send a fixed length packet to gimbal
    // returns true on success, false if serial port initialization failed
    bool send_fixedlen_packet(uint8_t address, const Identifier id, bool write, uint8_t value);

    // send a variable length packet to gimbal
    // returns true on success, false if serial port initialization failed
    bool send_variablelen_packet(HeaderType header, uint8_t address, const Identifier id, bool write, const uint8_t* databuff, uint8_t databuff_len);

    // members
    uint8_t _msg_buff[AP_MOUNT_TPFRAME_PACKETLEN_MAX];          // buffer holding bytes from latest packet received.  only used to calculate crc
    uint8_t _msg_buff_len;                                      // number of bytes in the msg buffer
    struct {
        ParseState state;                                       // parser state
        uint8_t data_len;                                       // expected number of data bytes
    } _parser;
};

#endif // HAL_MOUNT_TOPOTEK_ENABLED || HAL_MOUNT_SKYDROID_ENABLED
