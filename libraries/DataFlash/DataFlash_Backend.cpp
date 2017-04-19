#include "DataFlash_Backend.h"

#include "DFMessageWriter.h"

extern const AP_HAL::HAL& hal;

DataFlash_Backend::DataFlash_Backend(DataFlash_Class &front,
                                     class DFMessageWriter_DFLogStart *writer) :
    _front(front),
    _startup_messagewriter(writer)
{
    writer->set_dataflash_backend(this);
}

uint8_t DataFlash_Backend::num_types() const
{
    return _front._num_types;
}

const struct LogStructure *DataFlash_Backend::structure(uint8_t num) const
{
    return _front.structure(num);
}

DataFlash_Backend::vehicle_startup_message_Log_Writer DataFlash_Backend::vehicle_message_writer() {
    return _front._vehicle_messages;
}

void DataFlash_Backend::periodic_10Hz(const uint32_t now)
{
}
void DataFlash_Backend::periodic_1Hz(const uint32_t now)
{
}
void DataFlash_Backend::periodic_fullrate(const uint32_t now)
{
}

void DataFlash_Backend::periodic_tasks()
{
    uint32_t now = AP_HAL::millis();
    if (now - _last_periodic_1Hz > 1000) {
        periodic_1Hz(now);
        _last_periodic_1Hz = now;
    }
    if (now - _last_periodic_10Hz > 100) {
        periodic_10Hz(now);
        _last_periodic_10Hz = now;
    }
    periodic_fullrate(now);
}

void DataFlash_Backend::start_new_log_reset_variables()
{
    _startup_messagewriter->reset();
}

void DataFlash_Backend::internal_error() {
    _internal_errors++;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    abort();
#endif
}

void DataFlash_Backend::set_mission(const AP_Mission *mission) {
    _startup_messagewriter->set_mission(mission);
}

// this method can be overridden to do extra things with your buffer.
// for example, in DataFlash_MAVLink we may push messages into the UART.
void DataFlash_Backend::push_log_blocks() {
    WriteMoreStartupMessages();
}

// returns true if all format messages have been written, and thus it is OK
// for other messages to go out to the log
bool DataFlash_Backend::WriteBlockCheckStartupMessages()
{
    if (_startup_messagewriter->fmt_done()) {
        return true;
    }

    if (_writing_startup_messages) {
        // we have been called by a messagewriter, so writing is OK
        return true;
    }

    // we're not writing startup messages, so this must be some random
    // caller hoping to write blocks out.  Push out log blocks - we
    // might end up clearing the buffer.....
    push_log_blocks();

    //  even if we did finish writing startup messages, we can't
    //  permit any message to go in as its timestamp will be before
    //  any we wrote in.  Time going backwards annoys log readers.

    // sorry!  currently busy writing out startup messages...
    return false;
}

// source more messages from the startup message writer:
void DataFlash_Backend::WriteMoreStartupMessages()
{

    if (_startup_messagewriter->finished()) {
        return;
    }

    _writing_startup_messages = true;
    _startup_messagewriter->process();
    _writing_startup_messages = false;
}

/*
 * support for Log_Write():
 */


bool DataFlash_Backend::Log_Write_Emit_FMT(uint8_t msg_type)
{
    // get log structure from front end:
    struct LogStructure logstruct = {
        // these will be overwritten, but need to keep the compiler happy:
        0,
        0,
        "IGNO",
        "",
        ""
    };
    if (!_front.fill_log_write_logstructure(logstruct, msg_type)) {
        // this is a bug; we've been asked to write out the FMT
        // message for a msg_type, but the frontend can't supply the
        // required information
        internal_error();
        return false;
    }

    if (!Log_Write_Format(&logstruct)) {
        return false;
    }

    return true;
}

bool DataFlash_Backend::Log_Write(const uint8_t msg_type, va_list arg_list, bool is_critical)
{
    // stack-allocate a buffer so we can WriteBlock(); this could be
    // 255 bytes!  If we were willing to lose the WriteBlock
    // abstraction we could do WriteBytes() here instead?
    const char *fmt  = nullptr;
    uint8_t msg_len;
    DataFlash_Class::log_write_fmt *f;
    for (f = _front.log_write_fmts; f; f=f->next) {
        if (f->msg_type == msg_type) {
            fmt = f->fmt;
            msg_len = f->msg_len;
            break;
        }
    }
    if (fmt == nullptr) {
        // this is a bug.
        internal_error();
        return false;
    }
    if (bufferspace_available() < msg_len) {
        return false;
    }
    uint8_t buffer[msg_len];
    uint8_t offset = 0;
    buffer[offset++] = HEAD_BYTE1;
    buffer[offset++] = HEAD_BYTE2;
    buffer[offset++] = msg_type;
    for (uint8_t i=0; i<strlen(fmt); i++) {
        uint8_t charlen = 0;
        switch(fmt[i]) {
        case 'b': {
            int8_t tmp = va_arg(arg_list, int);
            memcpy(&buffer[offset], &tmp, sizeof(int8_t));
            offset += sizeof(int8_t);
            break;
        }
        case 'h':
        case 'c': {
            int16_t tmp = va_arg(arg_list, int);
            memcpy(&buffer[offset], &tmp, sizeof(int16_t));
            offset += sizeof(int16_t);
            break;
        }
        case 'd': {
            double tmp = va_arg(arg_list, double);
            memcpy(&buffer[offset], &tmp, sizeof(double));
            offset += sizeof(double);
            break;
        }
        case 'i':
        case 'L':
        case 'e': {
            int32_t tmp = va_arg(arg_list, int);
            memcpy(&buffer[offset], &tmp, sizeof(int32_t));
            offset += sizeof(int32_t);
            break;
        }
        case 'f': {
            float tmp = va_arg(arg_list, double);
            memcpy(&buffer[offset], &tmp, sizeof(float));
            offset += sizeof(float);
            break;
        }
        case 'n':
            charlen = 4;
            break;
        case 'M':
        case 'B': {
            uint8_t tmp = va_arg(arg_list, int);
            memcpy(&buffer[offset], &tmp, sizeof(uint8_t));
            offset += sizeof(uint8_t);
            break;
        }
        case 'H':
        case 'C': {
            uint16_t tmp = va_arg(arg_list, int);
            memcpy(&buffer[offset], &tmp, sizeof(uint16_t));
            offset += sizeof(uint16_t);
            break;
        }
        case 'I':
        case 'E': {
            uint32_t tmp = va_arg(arg_list, uint32_t);
            memcpy(&buffer[offset], &tmp, sizeof(uint32_t));
            offset += sizeof(uint32_t);
            break;
        }
        case 'N':
            charlen = 16;
            break;
        case 'Z':
            charlen = 64;
            break;
        case 'q': {
            int64_t tmp = va_arg(arg_list, int64_t);
            memcpy(&buffer[offset], &tmp, sizeof(int64_t));
            offset += sizeof(int64_t);
            break;
        }
        case 'Q': {
            uint64_t tmp = va_arg(arg_list, uint64_t);
            memcpy(&buffer[offset], &tmp, sizeof(uint64_t));
            offset += sizeof(uint64_t);
            break;
        }
        }
        if (charlen != 0) {
            char *tmp = va_arg(arg_list, char*);
            memcpy(&buffer[offset], tmp, charlen);
            offset += charlen;
        }
    }

    return WritePrioritisedBlock(buffer, msg_len, is_critical);
}
