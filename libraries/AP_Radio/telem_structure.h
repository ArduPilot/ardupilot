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

#define TELEM_FLAG_GPS_OK  (1U<<0)
#define TELEM_FLAG_ARM_OK  (1U<<1)
#define TELEM_FLAG_BATT_OK (1U<<2)
#define TELEM_FLAG_ARMED   (1U<<4)
#define TELEM_FLAG_POS_OK  (1U<<5)
#define TELEM_FLAG_VIDEO   (1U<<6)
#define TELEM_FLAG_HYBRID  (1U<<7)

struct telem_status {
    uint8_t pps; // packets per second received
    uint8_t rssi; // lowpass rssi
    uint8_t flags; // TELEM_FLAG_*
    uint8_t flight_mode; // flight mode
    uint8_t wifi_chan;
    uint8_t tx_max;
    uint8_t note_adjust;
};

// play a tune
struct telem_play {
    uint8_t seq;
    uint8_t tune_index;
};

// write to new firmware
struct telem_firmware {
    uint8_t seq;
    uint8_t len;
    uint16_t offset;
    uint8_t data[8];
};

/*
  telemetry packet from RX to TX
 */
struct telem_packet {
    uint8_t crc; // simple CRC
    enum telem_type type;
    union {
        uint8_t pkt[14];
        struct telem_status status;
        struct telem_firmware fw;
    } payload;
};


enum tx_telem_type {
    TXTELEM_RSSI = 0,
    TXTELEM_CRC1 = 1,
    TXTELEM_CRC2 = 2,
};

/*
  tx_status structure sent one byte at a time to RX. This is packed
  into channels 8, 9 and 10 (using 32 bits of a possible 33)
 */
struct telem_tx_status {
    uint8_t crc;
    enum tx_telem_type type;
    uint16_t data;
};
