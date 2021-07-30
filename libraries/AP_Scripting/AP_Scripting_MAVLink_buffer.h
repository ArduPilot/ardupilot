#pragma once

#include <GCS_MAVLink/GCS.h>

class command_buffer {
public:
    // constructor
    command_buffer(uint8_t buffer_size, int16_t _watch_CMD, uint32_t _watch_msgid);

    // Add a new buffer to the linked list
    void add_buffer(command_buffer * _new);

    // write item to buffer if it is the watched command ID and type
    bool write(const mavlink_command_long_t &cmd, const uint32_t time_ms, const mavlink_channel_t chan);
    bool write(const mavlink_command_int_t &cmd, const uint32_t time_ms, const mavlink_channel_t chan);

    // pop command from buffer, save last channel for ack
    uint8_t receive(mavlink_command_long_t &LONG, mavlink_command_int_t &INT, uint32_t &time_ms, uint8_t &chan);

    // send ack back to the last channel
    void send_ack(MAV_RESULT result);

    // send ack to a given channel
    void send_chan_ack(MAV_RESULT result, mavlink_channel_t chan);

private:

    // struct for object buffer
    struct scripting_cmd {
        union cmd_union {
            mavlink_command_long_t LONG;
            mavlink_command_int_t INT;
        } cmd;
        uint32_t time_ms;
        mavlink_channel_t chan;
    };

    // the message and command IDs to watch
    const int16_t watch_CMD;
    const uint32_t watch_msgid;

    // last channel popped
    int8_t _last_chan = -1;

    ObjectBuffer<scripting_cmd> buffer;

    command_buffer * next;

    HAL_Semaphore sem;

};
