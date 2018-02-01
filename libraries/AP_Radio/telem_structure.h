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

struct PACKED telem_status {
    uint8_t pps; // packets per second received
    uint8_t rssi; // lowpass rssi
    uint8_t flags; // TELEM_FLAG_*
    uint8_t flight_mode; // flight mode
    uint8_t wifi_chan;
    uint8_t tx_max;
    uint8_t note_adjust;
};

// play a tune
struct PACKED telem_play {
    uint8_t seq;
    uint8_t tune_index;
};

// write to new firmware
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


enum tx_telem_type {
    TXTELEM_RSSI = 0,
    TXTELEM_CRC1 = 1,
    TXTELEM_CRC2 = 2,
};

/*
  tx_status structure sent one byte at a time to RX. This is packed
  into channels 8, 9 and 10 (using 32 bits of a possible 33)
 */
struct PACKED telem_tx_status {
    uint8_t crc;
    enum tx_telem_type type;
    uint16_t data;
};

/*
  skyrocket specific packet for cc2500
 */
struct srt_packet {
    uint8_t length;
    uint8_t txid[2];
    uint8_t version; // protocol version
    uint8_t chan1;
    uint8_t chan2;
    uint8_t chan3;
    uint8_t chan4;
    uint8_t chan_high;
    uint8_t tx_voltage; // ADC value / 4
    uint8_t buttons;    // see channels.h
    uint8_t telem_pps;
    uint8_t telem_rssi;
    uint8_t channr;
    uint8_t chanskip;
    uint8_t crc[2];
};
