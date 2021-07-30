
#include "AP_Scripting_MAVLink_buffer.h"

command_buffer::command_buffer(uint8_t buffer_size, int16_t _watch_CMD, uint32_t _watch_msgid) :
    buffer(buffer_size),
    watch_CMD(_watch_CMD),
    watch_msgid(_watch_msgid) {}

// Add a new buffer to the linked list
void command_buffer::add_buffer(command_buffer * _new) {
    WITH_SEMAPHORE(sem);
    if (next == nullptr) {
        next =_new;
        return;
    }
    next->add_buffer(_new);
}

// write command long to buffer if it is the watched command and ID
bool command_buffer::write(const mavlink_command_long_t &cmd, const uint32_t time_ms, const mavlink_channel_t chan) {
    WITH_SEMAPHORE(sem);
    bool ret = false;
    if ((watch_msgid == MAVLINK_MSG_ID_COMMAND_LONG) && (cmd.command == watch_CMD)) {
        struct scripting_cmd data {{.LONG = cmd}, time_ms, chan};
        buffer.push(data);
        ret = true;
    }
    if (next != nullptr) {
        ret |= next->write(cmd, time_ms, chan);
    }
    return ret;
}

// write command int to buffer if it is the watched command and ID
bool command_buffer::write(const mavlink_command_int_t& cmd, const uint32_t time_ms, const mavlink_channel_t chan) {
    WITH_SEMAPHORE(sem);
    bool ret = false;
    if ((watch_msgid == MAVLINK_MSG_ID_COMMAND_INT) && (cmd.command == watch_CMD)) {
        struct scripting_cmd data {{.INT = cmd}, time_ms, chan};
        buffer.push(data);
        ret = true;
    }
    if (next != nullptr) {
        ret |= next->write(cmd, time_ms, chan);
    }
    return ret;
}

// pop command from buffer, save last channel for ack
uint8_t command_buffer::receive(mavlink_command_long_t &LONG, mavlink_command_int_t &INT, uint32_t &time_ms, uint8_t &chan) {
    WITH_SEMAPHORE(sem);
    scripting_cmd command;
    if (buffer.pop(command)) {
        uint8_t ret;
        if (watch_msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
            LONG = command.cmd.LONG;
            // tell scripting the first, third and fourth returns are valid
            ret = 0b1101;

        } else if (watch_msgid == MAVLINK_MSG_ID_COMMAND_INT) {
            INT = command.cmd.INT;
            ret = 0b1110;

        } else {
            return 0;
        }
        time_ms = command.time_ms;
        chan = command.chan;
        _last_chan = chan;
        return ret;
    }
    return 0;
}

// send ack back to the last channel
void command_buffer::send_ack(MAV_RESULT result) {
    if (_last_chan >= mavlink_channel_t::MAVLINK_COMM_0  && _last_chan <= mavlink_channel_t::MAVLINK_COMM_3) {
        send_chan_ack(result, (mavlink_channel_t)_last_chan);
    }
}

// send ack to a given channel
void command_buffer::send_chan_ack(MAV_RESULT result, mavlink_channel_t chan) {
    mavlink_msg_command_ack_send(chan, watch_CMD, result);
}
