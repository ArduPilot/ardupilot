#include "AP_Logger_Backend.h"

#include "LoggerMessageWriter.h"

extern const AP_HAL::HAL& hal;

AP_Logger_Backend::AP_Logger_Backend(AP_Logger &front,
                                     class LoggerMessageWriter_DFLogStart *writer) :
    _front(front),
    _startup_messagewriter(writer)
{
    writer->set_dataflash_backend(this);
}

uint8_t AP_Logger_Backend::num_types() const
{
    return _front._num_types;
}

const struct LogStructure *AP_Logger_Backend::structure(uint8_t num) const
{
    return _front.structure(num);
}

uint8_t AP_Logger_Backend::num_units() const
{
    return _front._num_units;
}

const struct UnitStructure *AP_Logger_Backend::unit(uint8_t num) const
{
    return _front.unit(num);
}

uint8_t AP_Logger_Backend::num_multipliers() const
{
    return _front._num_multipliers;
}

const struct MultiplierStructure *AP_Logger_Backend::multiplier(uint8_t num) const
{
    return _front.multiplier(num);
}

AP_Logger_Backend::vehicle_startup_message_Log_Writer AP_Logger_Backend::vehicle_message_writer() {
    return _front._vehicle_messages;
}

void AP_Logger_Backend::periodic_10Hz(const uint32_t now)
{
}
void AP_Logger_Backend::periodic_1Hz()
{
}
void AP_Logger_Backend::periodic_fullrate()
{
}

void AP_Logger_Backend::periodic_tasks()
{
    uint32_t now = AP_HAL::millis();
    if (now - _last_periodic_1Hz > 1000) {
        periodic_1Hz();
        _last_periodic_1Hz = now;
    }
    if (now - _last_periodic_10Hz > 100) {
        periodic_10Hz(now);
        _last_periodic_10Hz = now;
    }
    periodic_fullrate();
}

void AP_Logger_Backend::start_new_log_reset_variables()
{
    _startup_messagewriter->reset();
    _front.backend_starting_new_log(this);
}

void AP_Logger_Backend::internal_error() {
    _internal_errors++;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    abort();
#endif
}

// this method can be overridden to do extra things with your buffer.
// for example, in AP_Logger_MAVLink we may push messages into the UART.
void AP_Logger_Backend::push_log_blocks() {
    WriteMoreStartupMessages();
}

// returns true if all format messages have been written, and thus it is OK
// for other messages to go out to the log
bool AP_Logger_Backend::WriteBlockCheckStartupMessages()
{
#if APM_BUILD_TYPE(APM_BUILD_Replay)
    return true;
#endif
    if (_startup_messagewriter->fmt_done()) {
        return true;
    }

    if (_writing_startup_messages) {
        // we have been called by a messagewriter, so writing is OK
        return true;
    }

    if (!_startup_messagewriter->finished() &&
        !hal.scheduler->in_main_thread()) {
        // only the main thread may write startup messages out
        return false;
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
void AP_Logger_Backend::WriteMoreStartupMessages()
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


bool AP_Logger_Backend::Log_Write_Emit_FMT(uint8_t msg_type)
{
    // get log structure from front end:
    char ls_name[LS_NAME_SIZE] = {};
    char ls_format[LS_FORMAT_SIZE] = {};
    char ls_labels[LS_LABELS_SIZE] = {};
    char ls_units[LS_UNITS_SIZE] = {};
    char ls_multipliers[LS_MULTIPLIERS_SIZE] = {};
    struct LogStructure logstruct = {
        // these will be overwritten, but need to keep the compiler happy:
        0,
        0,
        ls_name,
        ls_format,
        ls_labels,
        ls_units,
        ls_multipliers
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
    if (!Log_Write_Format_Units(&logstruct)) {
        return false;
    }

    return true;
}

bool AP_Logger_Backend::Log_Write(const uint8_t msg_type, va_list arg_list, bool is_critical)
{
    // stack-allocate a buffer so we can WriteBlock(); this could be
    // 255 bytes!  If we were willing to lose the WriteBlock
    // abstraction we could do WriteBytes() here instead?
    const char *fmt  = nullptr;
    uint8_t msg_len;
    AP_Logger::log_write_fmt *f;
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

bool AP_Logger_Backend::StartNewLogOK() const
{
    if (logging_started()) {
        return false;
    }
    if (_front._log_bitmask == 0) {
        return false;
    }
    if (_front.in_log_download()) {
        return false;
    }
    if (!hal.scheduler->in_main_thread()) {
        return false;
    }
    return true;
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
void AP_Logger_Backend::validate_WritePrioritisedBlock(const void *pBuffer,
                                                       uint16_t size)
{
    // just check the first few packets to avoid too much overhead
    // (finding the structures is expensive)
    static uint16_t count = 0;
    if (count > 65534) {
        return;
    }
    count++;

    // we assume here that we ever WritePrioritisedBlock for a single
    // message.  If this assumption becomes false we can't do these
    // checks.
    if (size < 3) {
        AP_HAL::panic("Short prioritised block");
    }
    if (((uint8_t*)pBuffer)[0] != HEAD_BYTE1 ||
        ((uint8_t*)pBuffer)[1] != HEAD_BYTE2) {
        AP_HAL::panic("Not passed a message");
    }
    const uint8_t type = ((uint8_t*)pBuffer)[2];
    uint8_t type_len;
    const struct LogStructure *s = _front.structure_for_msg_type(type);
    if (s == nullptr) {
        const struct AP_Logger::log_write_fmt *t = _front.log_write_fmt_for_msg_type(type);
        if (t == nullptr) {
            AP_HAL::panic("No structure for msg_type=%u", type);
        }
        type_len = t->msg_len;
    } else {
        type_len = s->msg_len;
    }
    if (type_len != size) {
        char name[5] = {}; // get a null-terminated string
        memcpy(name, s->name, 4);
        AP_HAL::panic("Size mismatch for %u (%s) (expected=%u got=%u)\n",
                      type, name, type_len, size);
    }
}
#endif

bool AP_Logger_Backend::WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    validate_WritePrioritisedBlock(pBuffer, size);
#endif
    if (!ShouldLog(is_critical)) {
        return false;
    }
    if (StartNewLogOK()) {
        start_new_log();
    }
    if (!WritesOK()) {
        return false;
    }
    return _WritePrioritisedBlock(pBuffer, size, is_critical);
}

bool AP_Logger_Backend::ShouldLog(bool is_critical)
{
    if (!_front.WritesEnabled()) {
        return false;
    }
    if (!_initialised) {
        return false;
    }

    if (!_startup_messagewriter->finished() &&
        !hal.scheduler->in_main_thread()) {
        // only the main thread may write startup messages out
        return false;
    }

    if (is_critical && have_logged_armed && !_front._params.file_disarm_rot) {
        // if we have previously logged while armed then we log all
        // critical messages from then on. That fixes a problem where
        // logs show the wrong flight mode if you disarm then arm again
        return true;
    }
    
    if (!_front.vehicle_is_armed() && !_front.log_while_disarmed()) {
        return false;
    }

    if (_front.vehicle_is_armed()) {
        have_logged_armed = true;
    }
    
    return true;
}

bool AP_Logger_Backend::Log_Write_MessageF(const char *fmt, ...)
{
    char msg[65] {}; // sizeof(log_Message.msg) + null-termination

    va_list ap;
    va_start(ap, fmt);
    hal.util->vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);

    return Log_Write_Message(msg);
}
