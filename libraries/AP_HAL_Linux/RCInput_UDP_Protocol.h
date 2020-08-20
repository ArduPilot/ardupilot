#pragma once

#define RCINPUT_UDP_NUM_CHANNELS 8
#define RCINPUT_UDP_NUM_CHANNELS_V3 16

struct __attribute__((packed)) rc_udp_packet {
    uint32_t version;
    uint64_t timestamp_us;
    uint16_t sequence;
    uint16_t pwms[RCINPUT_UDP_NUM_CHANNELS];
};

struct __attribute__((packed)) rc_udp_packet_v3 {
    uint32_t version;
    uint64_t timestamp_us;
    uint16_t sequence;
    /* Up-to 16 channels, but client can transmit less channels */
    uint16_t pwms[RCINPUT_UDP_NUM_CHANNELS_V3];
};
