/*
  structures for telemetry packets
  This header is common to ArduPilot AP_Radio and STM8 TX code
 */

#pragma once

enum telem_type {
    TELEM_STATUS= 0, // a telem_status packet
    TELEM_PLAY  = 1, // play a tune
    TELEM_FW    = 2, // update new firmware
};

// flags in telem_status structure
#define TELEM_FLAG_GPS_OK  (1U<<0)
#define TELEM_FLAG_ARM_OK  (1U<<1)
#define TELEM_FLAG_BATT_OK (1U<<2)
#define TELEM_FLAG_ARMED   (1U<<4)
#define TELEM_FLAG_POS_OK  (1U<<5)
#define TELEM_FLAG_VIDEO   (1U<<6)
#define TELEM_FLAG_HYBRID  (1U<<7)

struct PACKED telem_status {
    uint8_t pps; // packets per second received
    uint8_t rssi; // lowpass rssi
    uint8_t flags; // TELEM_FLAG_*
    uint8_t flight_mode; // flight mode
    uint8_t wifi_chan; // wifi channel number on Sonix
    uint8_t tx_max;  // max TX power
    uint8_t note_adjust; // buzzer tone adjustment
};

// write to new firmware. This is also used to play a tune
struct PACKED telem_firmware {
    uint8_t seq;
    uint8_t len;
    uint16_t offset;
    uint8_t data[8];
};

/*
  telemetry packet from RX to TX for cypress
 */
struct PACKED telem_packet_cypress {
    uint8_t crc; // simple CRC
     enum telem_type type;
     union {
        uint8_t pkt[14];
         struct telem_status status;
         struct telem_firmware fw;
     } payload;
};

/*
  telemetry packet from RX to TX for cc2500
 */
struct PACKED telem_packet_cc2500 {
    uint8_t length;
    uint8_t type;
    uint8_t txid[2];
    union {
        uint8_t pkt[12];
        struct telem_status status;
        struct telem_firmware fw;
    } payload;
    uint8_t crc[2];
};

/*
  packet type - controls data field. We have 4 bits, giving 16 possible
  data field types
 */
enum packet_type {
    PKTYPE_VOLTAGE    = 0,
    PKTYPE_YEAR       = 1,
    PKTYPE_MONTH      = 2,
    PKTYPE_DAY        = 3,
    PKTYPE_TELEM_RSSI = 4,
    PKTYPE_TELEM_PPS  = 5,
    PKTYPE_BL_VERSION = 6,
    PKTYPE_FW_ACK     = 7,
    PKTYPE_NUM_TYPES  = 8 // used for modulus
};

/*
  skyrocket specific packet for cc2500
 */
struct PACKED srt_packet {
    uint8_t length;     // required for cc2500 FIFO
    uint8_t txid[2];
    uint8_t version:4;  // protocol version
    uint8_t pkt_type:4; // packet type
    uint8_t chan1;
    uint8_t chan2;
    uint8_t chan3;
    uint8_t chan4;
    uint8_t chan_high;
    uint8_t data;       // data according to pkt_type
    uint8_t buttons;    // see channels.h
    uint8_t channr;
    uint8_t chanskip;
    uint8_t crc[2];
};
