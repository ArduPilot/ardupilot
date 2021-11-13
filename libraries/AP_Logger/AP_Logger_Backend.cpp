#include "AP_Logger_Backend.h"

#include "LoggerMessageWriter.h"

#include "AP_Common/AP_FWVersion.h"
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Rally/AP_Rally.h>

extern const AP_HAL::HAL& hal;

AP_Logger_Backend::AP_Logger_Backend(AP_Logger &front,
                                     class LoggerMessageWriter_DFLogStart *writer) :
    _front(front),
    _startup_messagewriter(writer)
{
    writer->set_logger_backend(this);
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

AP_Logger_Backend::vehicle_startup_message_Writer AP_Logger_Backend::vehicle_message_writer() const {
    return _front._vehicle_messages;
}

void AP_Logger_Backend::periodic_10Hz(const uint32_t now)
{
}

void AP_Logger_Backend::periodic_1Hz()
{
    if (_rotate_pending && !logging_enabled()) {
        _rotate_pending = false;
        // handle log rotation once we stop logging
        stop_logging_async();
    }
    df_stats_log();
}

void AP_Logger_Backend::periodic_fullrate()
{
    push_log_blocks();
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
    _dropped = 0;
    _startup_messagewriter->reset();
    _front.backend_starting_new_log(this);
    _log_file_size_bytes = 0;
}

// We may need to make sure data is loggable before starting the
// EKF; when allow_start_ekf we should be able to log that data
bool AP_Logger_Backend::allow_start_ekf() const
{
    if (!_startup_messagewriter->fmt_done()) {
        return false;
    }
    // we need to push all startup messages out, or the code in
    // WriteBlockCheckStartupMessages bites us.
    if (!_startup_messagewriter->finished()) {
        return false;
    }
    return true;
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
#if APM_BUILD_TYPE(APM_BUILD_Replay)
    return;
#endif

    if (_startup_messagewriter->finished()) {
        return;
    }

    _writing_startup_messages = true;
    _startup_messagewriter->process();
    _writing_startup_messages = false;
}

/*
 * support for Write():
 */


bool AP_Logger_Backend::Write_Emit_FMT(uint8_t msg_type)
{
#if APM_BUILD_TYPE(APM_BUILD_Replay)
    if (msg_type < REPLAY_LOG_NEW_MSG_MIN || msg_type > REPLAY_LOG_NEW_MSG_MAX) {
        // don't re-emit FMU msgs unless they are in the replay range
        return true;
    }
#endif

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
        INTERNAL_ERROR(AP_InternalError::error_t::logger_missing_logstructure);
        return false;
    }

    if (!Write_Format(&logstruct)) {
        return false;
    }
    if (!Write_Format_Units(&logstruct)) {
        return false;
    }

    return true;
}

bool AP_Logger_Backend::Write(const uint8_t msg_type, va_list arg_list, bool is_critical, bool is_streaming)
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
        INTERNAL_ERROR(AP_InternalError::error_t::logger_logwrite_missingfmt);
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
        case 'a': {
            int16_t *tmp = va_arg(arg_list, int16_t*);
            const uint8_t bytes = 32*2;
            memcpy(&buffer[offset], tmp, bytes);
            offset += bytes;
            break;
        }
        }
        if (charlen != 0) {
            char *tmp = va_arg(arg_list, char*);
            uint8_t len = strnlen(tmp, charlen);
            memcpy(&buffer[offset], tmp, len);
            memset(&buffer[offset+len], 0, charlen-len);
            offset += charlen;
        }
    }

    return WritePrioritisedBlock(buffer, msg_len, is_critical, is_streaming);
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
        if (s->name != nullptr) {
            memcpy(name, s->name, 4);
        } else {
            strncpy(name, "?NM?", ARRAY_SIZE(name));
        }
        AP_HAL::panic("Size mismatch for %u (%s) (expected=%u got=%u)\n",
                      type, name, type_len, size);
    }
}
#endif

bool AP_Logger_Backend::WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical, bool writev_streaming)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !APM_BUILD_TYPE(APM_BUILD_Replay)
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

    if (!is_critical && rate_limiter != nullptr) {
        const uint8_t *msgbuf = (const uint8_t *)pBuffer;
        if (!rate_limiter->should_log(msgbuf[2], writev_streaming)) {
            return false;
        }
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

    if (_front.in_log_download() &&
        _front._last_mavlink_log_transfer_message_handled_ms != 0) {
        if (AP_HAL::millis() - _front._last_mavlink_log_transfer_message_handled_ms < 10000) {
            if (!_front.vehicle_is_armed()) {
                // user is transferring files via mavlink
                return false;
            }
        } else {
            _front._last_mavlink_log_transfer_message_handled_ms = 0;
        }
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

void AP_Logger_Backend::PrepForArming()
{
    if (_rotate_pending) {
        _rotate_pending = false;
        stop_logging();
    }
    if (logging_started()) {
        return;
    }
    PrepForArming_start_logging();
}

bool AP_Logger_Backend::Write_MessageF(const char *fmt, ...)
{
    char msg[65] {}; // sizeof(log_Message.msg) + null-termination

    va_list ap;
    va_start(ap, fmt);
    hal.util->vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);

    return Write_Message(msg);
}

#if HAL_RALLY_ENABLED
// Write rally points
bool AP_Logger_Backend::Write_RallyPoint(uint8_t total,
                                         uint8_t sequence,
                                         const RallyLocation &rally_point)
{
    const struct log_Rally pkt_rally{
        LOG_PACKET_HEADER_INIT(LOG_RALLY_MSG),
        time_us         : AP_HAL::micros64(),
        total           : total,
        sequence        : sequence,
        latitude        : rally_point.lat,
        longitude       : rally_point.lng,
        altitude        : rally_point.alt
    };
    return WriteBlock(&pkt_rally, sizeof(pkt_rally));
}

// Write rally points
bool AP_Logger_Backend::Write_Rally()
{
    // kick off asynchronous write:
    return _startup_messagewriter->writeallrallypoints();
}
#endif

#if HAL_LOGGER_FENCE_ENABLED
// Write a fence point
bool AP_Logger_Backend::Write_FencePoint(uint8_t total, uint8_t sequence, const AC_PolyFenceItem &fence_point)
{
    const struct log_Fence pkt_fence{
        LOG_PACKET_HEADER_INIT(LOG_FENCE_MSG),
        time_us         : AP_HAL::micros64(),
        total           : total,
        sequence        : sequence,
        type            : uint8_t(fence_point.type),
        latitude        : fence_point.loc.x,
        longitude       : fence_point.loc.y,
        vertex_count    : fence_point.vertex_count,
        radius          : fence_point.radius
    };
    return WriteBlock(&pkt_fence, sizeof(pkt_fence));
}

// Write all fence points
bool AP_Logger_Backend::Write_Fence()
{
    // kick off asynchronous write:
    return _startup_messagewriter->writeallfence();
}
#endif // HAL_LOGGER_FENCE_ENABLED


bool AP_Logger_Backend::Write_VER()
{
    const AP_FWVersion &fwver = AP::fwversion();

    log_VER pkt{
        LOG_PACKET_HEADER_INIT(LOG_VER_MSG),
        time_us  : AP_HAL::micros64(),
        board_type : fwver.board_type,
        board_subtype: fwver.board_subtype,
        major: fwver.major,
        minor: fwver.minor,
        patch: fwver.patch,
        fw_type: fwver.fw_type,
        git_hash: fwver.fw_hash,
    };
    strncpy(pkt.fw_string, fwver.fw_string, ARRAY_SIZE(pkt.fw_string)-1);

#ifdef APJ_BOARD_ID
    pkt._APJ_BOARD_ID = APJ_BOARD_ID;
#endif

    return WriteCriticalBlock(&pkt, sizeof(pkt));
}

/*
  convert a list entry number back into a log number (which can then
  be converted into a filename).  A "list entry number" is a sequence
  where the oldest log has a number of 1, the second-from-oldest 2,
  and so on.  Thus the highest list entry number is equal to the
  number of logs.
*/
uint16_t AP_Logger_Backend::log_num_from_list_entry(const uint16_t list_entry)
{
    uint16_t oldest_log = find_oldest_log();
    if (oldest_log == 0) {
        return 0;
    }

    uint32_t log_num = oldest_log + list_entry - 1;
    if (log_num > MAX_LOG_FILES) {
        log_num -= MAX_LOG_FILES;
    }
    return (uint16_t)log_num;
}

// find_oldest_log - find oldest log
// returns 0 if no log was found
uint16_t AP_Logger_Backend::find_oldest_log()
{
    if (_cached_oldest_log != 0) {
        return _cached_oldest_log;
    }

    uint16_t last_log_num = find_last_log();
    if (last_log_num == 0) {
        return 0;
    }

    _cached_oldest_log = last_log_num - get_num_logs() + 1;

    return _cached_oldest_log;
}

void AP_Logger_Backend::vehicle_was_disarmed()
{
    if (_front._params.file_disarm_rot) {
        // rotate our log.  Closing the current one and letting the
        // logging restart naturally based on log_disarmed should do
        // the trick:
        _rotate_pending = true;
    }
}

// this sensor is enabled if we should be logging at the moment
bool AP_Logger_Backend::logging_enabled() const
{
    if (hal.util->get_soft_armed() ||
        _front.log_while_disarmed()) {
        return true;
    }
    return false;
}

void AP_Logger_Backend::Write_AP_Logger_Stats_File(const struct df_stats &_stats)
{
    const struct log_DSF pkt {
        LOG_PACKET_HEADER_INIT(LOG_DF_FILE_STATS),
        time_us         : AP_HAL::micros64(),
        dropped         : _dropped,
        blocks          : _stats.blocks,
        bytes           : _stats.bytes,
        buf_space_min   : _stats.buf_space_min,
        buf_space_max   : _stats.buf_space_max,
        buf_space_avg   : (_stats.blocks) ? (_stats.buf_space_sigma / _stats.blocks) : 0,
    };
    WriteBlock(&pkt, sizeof(pkt));
}

void AP_Logger_Backend::df_stats_gather(const uint16_t bytes_written, uint32_t space_remaining)
{
    if (space_remaining < stats.buf_space_min) {
        stats.buf_space_min = space_remaining;
    }
    if (space_remaining > stats.buf_space_max) {
        stats.buf_space_max = space_remaining;
    }
    stats.buf_space_sigma += space_remaining;
    stats.bytes += bytes_written;
    _log_file_size_bytes += bytes_written;
    stats.blocks++;
}

void AP_Logger_Backend::df_stats_clear() {
    memset(&stats, '\0', sizeof(stats));
    stats.buf_space_min = -1;
}

void AP_Logger_Backend::df_stats_log() {
    Write_AP_Logger_Stats_File(stats);
    df_stats_clear();
}


// class to handle rate limiting of log messages
AP_Logger_RateLimiter::AP_Logger_RateLimiter(const AP_Logger &_front, const AP_Float &_limit_hz)
    : front(_front), rate_limit_hz(_limit_hz)
{
}

/*
  return false if a streaming message should not be sent yet
 */
bool AP_Logger_RateLimiter::should_log_streaming(uint8_t msgid)
{
    if (front._log_pause) {
        return false;
    }
    const uint16_t now = AP_HAL::millis16();
    uint16_t delta_ms = now - last_send_ms[msgid];
    if (delta_ms < 1000.0 / rate_limit_hz.get()) {
        // too soon
        return false;
    }
    last_send_ms[msgid] = now;
    return true;
}

/*
  return true if the message is not a streaming message or the gap
  from the last message is more than the message rate
 */
bool AP_Logger_RateLimiter::should_log(uint8_t msgid, bool writev_streaming)
{
    if (rate_limit_hz <= 0 && !front._log_pause) {
        // no rate limiting if not paused and rate is zero(user changed the parameter)
        return true;
    }
    if (last_send_ms[msgid] == 0 && !writev_streaming) {
        // might be non streaming. check the not_streaming bitmask
        // cache
        if (not_streaming.get(msgid)) {
            return true;
        }
        const auto *mtype = front.structure_for_msg_type(msgid);
        if (mtype == nullptr ||
            mtype->streaming == false) {
            not_streaming.set(msgid);
            return true;
        }
    }

#if !defined(HAL_BUILD_AP_PERIPH)
    // if we've already decided on sending this msgid in this tick then use the
    // same decision again
    const uint16_t sched_ticks = AP::scheduler().ticks();
    if (sched_ticks == last_sched_count[msgid]) {
        return last_return.get(msgid);
    }
    last_sched_count[msgid] = sched_ticks;
#endif

    bool ret = should_log_streaming(msgid);
    if (ret) {
        last_return.set(msgid);
    } else {
        last_return.clear(msgid);
    }
    return ret;
}
