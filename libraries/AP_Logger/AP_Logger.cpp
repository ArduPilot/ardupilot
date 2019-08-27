#include "AP_Logger.h"

#include "AP_Logger_Backend.h"

#include "AP_Logger_File.h"
#include "AP_Logger_SITL.h"
#include "AP_Logger_DataFlash.h"
#include "AP_Logger_MAVLink.h"

#include <AP_InternalError/AP_InternalError.h>
#include <GCS_MAVLink/GCS.h>

AP_Logger *AP_Logger::_singleton;

extern const AP_HAL::HAL& hal;

#ifndef HAL_LOGGING_FILE_BUFSIZE
#define HAL_LOGGING_FILE_BUFSIZE  16
#endif 

#ifndef HAL_LOGGING_MAV_BUFSIZE
#define HAL_LOGGING_MAV_BUFSIZE  8
#endif 

#ifndef HAL_LOGGING_FILE_TIMEOUT
#define HAL_LOGGING_FILE_TIMEOUT 5
#endif 

// by default log for 15 seconds after disarming
#ifndef HAL_LOGGER_ARM_PERSIST
#define HAL_LOGGER_ARM_PERSIST 15
#endif

#ifndef HAL_LOGGING_BACKENDS_DEFAULT
# ifdef HAL_LOGGING_DATAFLASH
#  define HAL_LOGGING_BACKENDS_DEFAULT Backend_Type::BLOCK
# else
#  define HAL_LOGGING_BACKENDS_DEFAULT Backend_Type::FILESYSTEM
# endif
#endif

const AP_Param::GroupInfo AP_Logger::var_info[] = {
    // @Param: _BACKEND_TYPE
    // @DisplayName: AP_Logger Backend Storage type
    // @Description: Bitmap of what Logger backend types to enable. Block-based logging is available on SITL and boards with dataflash chips. Multiple backends can be selected.
    // @Values: 0:None,1:File,2:MAVLink,3:File and MAVLink,4:Block,6:Block and MAVLink
    // @Bitmask: 0:File,1:MAVLink,2:Block
    // @User: Standard
    AP_GROUPINFO("_BACKEND_TYPE",  0, AP_Logger, _params.backend_types,       uint8_t(HAL_LOGGING_BACKENDS_DEFAULT)),

    // @Param: _FILE_BUFSIZE
    // @DisplayName: Maximum AP_Logger File Backend buffer size (in kilobytes)
    // @Description: The AP_Logger_File backend uses a buffer to store data before writing to the block device.  Raising this value may reduce "gaps" in your SD card logging.  This buffer size may be reduced depending on available memory.  PixHawk requires at least 4 kilobytes.  Maximum value available here is 64 kilobytes.
    // @User: Standard
    AP_GROUPINFO("_FILE_BUFSIZE",  1, AP_Logger, _params.file_bufsize,       HAL_LOGGING_FILE_BUFSIZE),

    // @Param: _DISARMED
    // @DisplayName: Enable logging while disarmed
    // @Description: If LOG_DISARMED is set to 1 then logging will be enabled while disarmed. This can make for very large logfiles but can help a lot when tracking down startup issues
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("_DISARMED",  2, AP_Logger, _params.log_disarmed,       0),

    // @Param: _REPLAY
    // @DisplayName: Enable logging of information needed for Replay
    // @Description: If LOG_REPLAY is set to 1 then the EKF2 state estimator will log detailed information needed for diagnosing problems with the Kalman filter. It is suggested that you also raise LOG_FILE_BUFSIZE to give more buffer space for logging and use a high quality microSD card to ensure no sensor data is lost
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("_REPLAY",  3, AP_Logger, _params.log_replay,       0),

    // @Param: _FILE_DSRMROT
    // @DisplayName: Stop logging to current file on disarm
    // @Description: When set, the current log file is closed when the vehicle is disarmed.  If LOG_DISARMED is set then a fresh log will be opened.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("_FILE_DSRMROT",  4, AP_Logger, _params.file_disarm_rot,       0),

    // @Param: _MAV_BUFSIZE
    // @DisplayName: Maximum AP_Logger MAVLink Backend buffer size
    // @Description: Maximum amount of memory to allocate to AP_Logger-over-mavlink
    // @User: Advanced
    // @Units: kB
    AP_GROUPINFO("_MAV_BUFSIZE",  5, AP_Logger, _params.mav_bufsize,       HAL_LOGGING_MAV_BUFSIZE),

    // @Param: _FILE_TIMEOUT
    // @DisplayName: Timeout before giving up on file writes
    // @Description: This controls the amount of time before failing writes to a log file cause the file to be closed and logging stopped.
    // @User: Standard
    // @Units: s
    AP_GROUPINFO("_FILE_TIMEOUT",  6, AP_Logger, _params.file_timeout,     HAL_LOGGING_FILE_TIMEOUT),
    
    AP_GROUPEND
};

#define streq(x, y) (!strcmp(x, y))

AP_Logger::AP_Logger(const AP_Int32 &log_bitmask)
    : _log_bitmask(log_bitmask)
{
    AP_Param::setup_object_defaults(this, var_info);
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Logger must be singleton");
    }

    _singleton = this;
}

void AP_Logger::Init(const struct LogStructure *structures, uint8_t num_types)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Preparing log system");
    if (hal.util->was_watchdog_armed()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Forcing logging for watchdog reset");
        _params.log_disarmed.set(1);
    }
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    validate_structures(structures, num_types);
    dump_structures(structures, num_types);
#endif
    if (_next_backend == LOGGER_MAX_BACKENDS) {
        AP_HAL::panic("Too many backends");
        return;
    }
    _num_types = num_types;
    _structures = structures;

#if defined(HAL_BOARD_LOG_DIRECTORY) && HAVE_FILESYSTEM_SUPPORT
    if (_params.backend_types & uint8_t(Backend_Type::FILESYSTEM)) {
        LoggerMessageWriter_DFLogStart *message_writer =
            new LoggerMessageWriter_DFLogStart();
        if (message_writer != nullptr)  {
            backends[_next_backend] = new AP_Logger_File(*this,
                                                         message_writer,
                                                         HAL_BOARD_LOG_DIRECTORY);
        }
        if (backends[_next_backend] == nullptr) {
            hal.console->printf("Unable to open AP_Logger_File");
        } else {
            _next_backend++;
        }
    }
#endif // HAVE_FILESYSTEM_SUPPORT

#if LOGGER_MAVLINK_SUPPORT
    if (_params.backend_types & uint8_t(Backend_Type::MAVLINK)) {
        if (_next_backend == LOGGER_MAX_BACKENDS) {
            AP_HAL::panic("Too many backends");
            return;
        }
        LoggerMessageWriter_DFLogStart *message_writer =
            new LoggerMessageWriter_DFLogStart();
        if (message_writer != nullptr)  {
            backends[_next_backend] = new AP_Logger_MAVLink(*this,
                                                            message_writer);
        }
        if (backends[_next_backend] == nullptr) {
            hal.console->printf("Unable to open AP_Logger_MAVLink");
        } else {
            _next_backend++;
        }
    }
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_params.backend_types & uint8_t(Backend_Type::BLOCK)) {
        if (_next_backend == LOGGER_MAX_BACKENDS) {
            AP_HAL::panic("Too many backends");
            return;
        }
        LoggerMessageWriter_DFLogStart *message_writer =
            new LoggerMessageWriter_DFLogStart();
        if (message_writer != nullptr)  {
            backends[_next_backend] = new AP_Logger_SITL(*this, message_writer);
        }
        if (backends[_next_backend] == nullptr) {
            hal.console->printf("Unable to open AP_Logger_SITL");
        } else {
            _next_backend++;
        }
    }
#endif

#ifdef HAL_LOGGING_DATAFLASH
    if (_params.backend_types & uint8_t(Backend_Type::BLOCK)) {
        if (_next_backend == LOGGER_MAX_BACKENDS) {
            AP_HAL::panic("Too many backends");
            return;
        }
        LoggerMessageWriter_DFLogStart *message_writer =
            new LoggerMessageWriter_DFLogStart();
        if (message_writer != nullptr)  {
            backends[_next_backend] = new AP_Logger_DataFlash(*this, message_writer);
        }
        if (backends[_next_backend] == nullptr) {
            hal.console->printf("Unable to open AP_Logger_DataFlash");
        } else {
            _next_backend++;
        }
    }
#endif
    
    for (uint8_t i=0; i<_next_backend; i++) {
        backends[i]->Init();
    }

    Prep();

    EnableWrites(true);

    gcs().send_text(MAV_SEVERITY_INFO, "Prepared log system");
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <stdio.h>

#define DEBUG_LOG_STRUCTURES 0

extern const AP_HAL::HAL& hal;
#define Debug(fmt, args ...)  do {::fprintf(stderr, "%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)

/// return the number of commas present in string
static uint8_t count_commas(const char *string)
{
    uint8_t ret = 0;
    for (uint8_t i=0; i<strlen(string); i++) {
        if (string[i] == ',') {
            ret++;
        }
    }
    return ret;
}

/// return a unit name given its ID
const char* AP_Logger::unit_name(const uint8_t unit_id)
{
    for(uint8_t i=0; i<unit_id; i++) {
        if (_units[i].ID == unit_id) {
            return _units[i].unit;
        }
    }
    return NULL;
}

/// return a multiplier value given its ID
double AP_Logger::multiplier_name(const uint8_t multiplier_id)
{
    for(uint8_t i=0; i<multiplier_id; i++) {
        if (_multipliers[i].ID == multiplier_id) {
            return _multipliers[i].multiplier;
        }
    }
    // Should we abort here?
    return 1.0f;
}

/// pretty-print field information from a log structure
void AP_Logger::dump_structure_field(const struct LogStructure *logstructure, const char *label, const uint8_t fieldnum)
{
    ::fprintf(stderr, "  %s (%s)*(%f)\n", label, unit_name(logstructure->units[fieldnum]), multiplier_name(logstructure->multipliers[fieldnum]));
}

/// pretty-print log structures
/// @note structures MUST be well-formed
void AP_Logger::dump_structures(const struct LogStructure *logstructures, const uint8_t num_types)
{
#if DEBUG_LOG_STRUCTURES
    for (uint16_t i=0; i<num_types; i++) {
        const struct LogStructure *logstructure = &logstructures[i];
        ::fprintf(stderr, "%s\n", logstructure->name);
        char label[32] = { };
        uint8_t labeloffset = 0;
        int8_t fieldnum = 0;
        for (uint8_t j=0; j<strlen(logstructure->labels); j++) {
            char labelchar = logstructure->labels[j];
            if (labelchar == '\0') {
                break;
            }
            if (labelchar == ',') {
                dump_structure_field(logstructure, label, fieldnum);
                fieldnum++;
                labeloffset = 0;
                memset(label, '\0', 32);
            } else {
                label[labeloffset++] = labelchar;
            }
        }
        dump_structure_field(logstructure, label, fieldnum);
        ::fprintf(stderr, "\n"); // just add a CR to the output
    }
#endif
}

bool AP_Logger::validate_structure(const struct LogStructure *logstructure, const int16_t offset)
{
    bool passed = true;

#if DEBUG_LOG_STRUCTURES
    Debug("offset=%d ID=%d NAME=%s", offset, logstructure->msg_type, logstructure->name);
#endif

    // fields must be null-terminated
#define CHECK_ENTRY(fieldname,fieldname_s,fieldlen)                     \
    do {                                                                \
        if (strnlen(logstructure->fieldname, fieldlen) > fieldlen-1) {  \
            Debug("  Message " fieldname_s " not NULL-terminated or too long"); \
            passed = false;                                             \
        }                                                               \
    } while (false)
    CHECK_ENTRY(name, "name", LS_NAME_SIZE);
    CHECK_ENTRY(format, "format", LS_FORMAT_SIZE);
    CHECK_ENTRY(labels, "labels", LS_LABELS_SIZE);
    CHECK_ENTRY(units, "units", LS_UNITS_SIZE);
    CHECK_ENTRY(multipliers, "multipliers", LS_MULTIPLIERS_SIZE);
#undef CHECK_ENTRY

    // ensure each message ID is only used once
    if (seen_ids[logstructure->msg_type]) {
        Debug("  ID %d used twice (LogStructure offset=%d)", logstructure->msg_type, offset);
        passed = false;
    }
    seen_ids[logstructure->msg_type] = true;

    // ensure we have enough labels to cover columns
    uint8_t fieldcount = strlen(logstructure->format);
    uint8_t labelcount = count_commas(logstructure->labels)+1;
    if (fieldcount != labelcount) {
        Debug("  fieldcount=%u does not match labelcount=%u",
              fieldcount, labelcount);
        passed = false;
    }

    // check that the structure is of an appropriate length to take fields
    const int16_t msg_len = Write_calc_msg_len(logstructure->format);
    if (msg_len != logstructure->msg_len) {
        Debug("  Calculated message length for (%s) based on format field (%s) does not match structure size (%d != %u)", logstructure->name, logstructure->format, msg_len, logstructure->msg_len);
        passed = false;
    }

    // ensure we have units for each field:
    if (strlen(logstructure->units) != fieldcount) {
        Debug("  fieldcount=%u does not match unitcount=%u",
              (unsigned)fieldcount, (unsigned)strlen(logstructure->units));
        passed = false;
    }

    // ensure we have multipliers for each field
    if (strlen(logstructure->multipliers) != fieldcount) {
        Debug("  fieldcount=%u does not match multipliercount=%u",
              (unsigned)fieldcount, (unsigned)strlen(logstructure->multipliers));
        passed = false;
    }

    // ensure the FMTU messages reference valid units
    for (uint8_t j=0; j<strlen(logstructure->units); j++) {
        char logunit = logstructure->units[j];
        uint8_t k;
        for (k=0; k<_num_units; k++) {
            if (logunit == _units[k].ID) {
                // found this one
                break;
            }
        }
        if (k == _num_units) {
            Debug("  invalid unit=%c", logunit);
            passed = false;
        }
    }

    // ensure the FMTU messages reference valid multipliers
    for (uint8_t j=0; j<strlen(logstructure->multipliers); j++) {
        char logmultiplier = logstructure->multipliers[j];
        uint8_t k;
        for (k=0; k<_num_multipliers; k++) {
            if (logmultiplier == _multipliers[k].ID) {
                // found this one
                break;
            }
        }
        if (k == _num_multipliers) {
            Debug("  invalid multiplier=%c", logmultiplier);
            passed = false;
        }
    }

    // ensure any float has a multiplier of zero
    if (false && passed) {
        for (uint8_t j=0; j<strlen(logstructure->multipliers); j++) {
            const char fmt = logstructure->format[j];
            if (fmt != 'f') {
                continue;
            }
            const char logmultiplier = logstructure->multipliers[j];
            if (logmultiplier == '0' ||
                logmultiplier == '?' ||
                logmultiplier == '-') {
                continue;
            }
            Debug("  %s[%u] float with non-zero multiplier=%c",
                  logstructure->name,
                  j,
                  logmultiplier);
            passed = false;
        }
    }

    return passed;
}

void AP_Logger::validate_structures(const struct LogStructure *logstructures, const uint8_t num_types)
{
    Debug("Validating structures");
    bool passed = true;

    for (uint16_t i=0; i<num_types; i++) {
        const struct LogStructure *logstructure = &logstructures[i];
        passed = validate_structure(logstructure, i) && passed;
    }

    // ensure units are unique:
    for (uint16_t i=0; i<ARRAY_SIZE(log_Units); i++) {
        const struct UnitStructure &a = log_Units[i];
        for (uint16_t j=i+1; j<ARRAY_SIZE(log_Units); j++) {
            const struct UnitStructure &b = log_Units[j];
            if (a.ID == b.ID) {
                Debug("duplicate unit id=%c (%s/%s)", a.ID, a.unit, b.unit);
                passed = false;
            }
            if (streq(a.unit, b.unit)) {
                Debug("duplicate unit=%s (%c/%c)", a.unit, a.ID, b.ID);
                passed = false;
            }
        }
    }

    // ensure multipliers are unique:
    for (uint16_t i=0; i<ARRAY_SIZE(log_Multipliers); i++) {
        const struct MultiplierStructure &a = log_Multipliers[i];
        for (uint16_t j=i+1; j<ARRAY_SIZE(log_Multipliers); j++) {
            const struct MultiplierStructure &b = log_Multipliers[j];
            if (a.ID == b.ID) {
                Debug("duplicate multiplier id=%c (%f/%f)",
                      a.ID, a.multiplier, b.multiplier);
                passed = false;
            }
            if (is_equal(a.multiplier, b.multiplier)) {
                if (a.ID == '?' && b.ID == '0') {
                    // special case
                    continue;
                }
                Debug("duplicate multiplier=%f (%c/%c)",
                      a.multiplier, a.ID, b.ID);
                passed = false;
            }
        }
    }

    if (!passed) {
        Debug("Log structures are invalid");
        abort();
    }
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL

const struct LogStructure *AP_Logger::structure(uint16_t num) const
{
    return &_structures[num];
}

bool AP_Logger::logging_present() const
{
    return _next_backend != 0;
}
bool AP_Logger::logging_enabled() const
{
    if (_next_backend == 0) {
        return false;
    }
    for (uint8_t i=0; i<_next_backend; i++) {
        if (backends[i]->logging_enabled()) {
            return true;
        }
    }
    return false;
}
bool AP_Logger::logging_failed() const
{
    if (_next_backend < 1) {
        // we should not have been called!
        return true;
    }
    for (uint8_t i=0; i<_next_backend; i++) {
        if (backends[i]->logging_failed()) {
            return true;
        }
    }
    return false;
}

void AP_Logger::Write_MessageF(const char *fmt, ...)
{
    char msg[65] {}; // sizeof(log_Message.msg) + null-termination

    va_list ap;
    va_start(ap, fmt);
    hal.util->vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);

    Write_Message(msg);
}

void AP_Logger::backend_starting_new_log(const AP_Logger_Backend *backend)
{
    for (uint8_t i=0; i<_next_backend; i++) {
        if (backends[i] == backend) { // pointer comparison!
            // reset sent masks
            for (struct log_write_fmt *f = log_write_fmts; f; f=f->next) {
                f->sent_mask &= ~(1<<i);
            }
            break;
        }
    }
}

bool AP_Logger::should_log(const uint32_t mask) const
{
    bool armed = vehicle_is_armed();

    if (!(mask & _log_bitmask)) {
        return false;
    }
    if (!armed && !log_while_disarmed()) {
        return false;
    }
    if (in_log_download()) {
        return false;
    }
    if (_next_backend == 0) {
        return false;
    }
    return true;
}

const struct UnitStructure *AP_Logger::unit(uint16_t num) const
{
    return &_units[num];
}

const struct MultiplierStructure *AP_Logger::multiplier(uint16_t num) const
{
    return &log_Multipliers[num];
}

#define FOR_EACH_BACKEND(methodcall)              \
    do {                                          \
        for (uint8_t i=0; i<_next_backend; i++) { \
            backends[i]->methodcall;              \
        }                                         \
    } while (0)

void AP_Logger::PrepForArming()
{
    FOR_EACH_BACKEND(PrepForArming());
}

void AP_Logger::setVehicle_Startup_Writer(vehicle_startup_message_Writer writer)
{
    _vehicle_messages = writer;
}

void AP_Logger::set_vehicle_armed(const bool armed_state)
{
    if (armed_state == _armed) {
        // no change in status
        return;
    }
    _armed = armed_state;

    if (!_armed) {
        // went from armed to disarmed
        FOR_EACH_BACKEND(vehicle_was_disarmed());
    }

}


// start functions pass straight through to backend:
void AP_Logger::WriteBlock(const void *pBuffer, uint16_t size) {
    FOR_EACH_BACKEND(WriteBlock(pBuffer, size));
}

void AP_Logger::WriteCriticalBlock(const void *pBuffer, uint16_t size) {
    FOR_EACH_BACKEND(WriteCriticalBlock(pBuffer, size));
}

void AP_Logger::WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical) {
    FOR_EACH_BACKEND(WritePrioritisedBlock(pBuffer, size, is_critical));
}

// change me to "DoTimeConsumingPreparations"?
void AP_Logger::EraseAll() {
    FOR_EACH_BACKEND(EraseAll());
}
// change me to "LoggingAvailable"?
bool AP_Logger::CardInserted(void) {
    for (uint8_t i=0; i< _next_backend; i++) {
        if (backends[i]->CardInserted()) {
            return true;
        }
    }
    return false;
}

void AP_Logger::Prep() {
    FOR_EACH_BACKEND(Prep());
}

void AP_Logger::StopLogging()
{
    FOR_EACH_BACKEND(stop_logging());
}

uint16_t AP_Logger::find_last_log() const {
    if (_next_backend == 0) {
        return 0;
    }
    return backends[0]->find_last_log();
}
void AP_Logger::get_log_boundaries(uint16_t log_num, uint32_t & start_page, uint32_t & end_page) {
    if (_next_backend == 0) {
        return;
    }
    backends[0]->get_log_boundaries(log_num, start_page, end_page);
}
void AP_Logger::get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc) {
    if (_next_backend == 0) {
        return;
    }
    backends[0]->get_log_info(log_num, size, time_utc);
}
int16_t AP_Logger::get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) {
    if (_next_backend == 0) {
        return 0;
    }
    return backends[0]->get_log_data(log_num, page, offset, len, data);
}
uint16_t AP_Logger::get_num_logs(void) {
    if (_next_backend == 0) {
        return 0;
    }
    return backends[0]->get_num_logs();
}

/* we're started if any of the backends are started */
bool AP_Logger::logging_started(void) {
    for (uint8_t i=0; i< _next_backend; i++) {
        if (backends[i]->logging_started()) {
            return true;
        }
    }
    return false;
}

void AP_Logger::handle_mavlink_msg(GCS_MAVLINK &link, const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS:
        FOR_EACH_BACKEND(remote_log_block_status_msg(link.get_chan(), msg));
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
        FALLTHROUGH;
    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
        FALLTHROUGH;
    case MAVLINK_MSG_ID_LOG_ERASE:
        FALLTHROUGH;
    case MAVLINK_MSG_ID_LOG_REQUEST_END:
        handle_log_message(link, msg);
        break;
    }
}

void AP_Logger::periodic_tasks() {
    handle_log_send();
    FOR_EACH_BACKEND(periodic_tasks());
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    // currently only AP_Logger_File support this:
void AP_Logger::flush(void) {
     FOR_EACH_BACKEND(flush());
}
#endif


void AP_Logger::Write_EntireMission()
{
    FOR_EACH_BACKEND(Write_EntireMission());
}

void AP_Logger::Write_Message(const char *message)
{
    FOR_EACH_BACKEND(Write_Message(message));
}

void AP_Logger::Write_Mode(uint8_t mode, uint8_t reason)
{
    FOR_EACH_BACKEND(Write_Mode(mode, reason));
}

void AP_Logger::Write_Parameter(const char *name, float value)
{
    FOR_EACH_BACKEND(Write_Parameter(name, value));
}

void AP_Logger::Write_Mission_Cmd(const AP_Mission &mission,
                                            const AP_Mission::Mission_Command &cmd)
{
    FOR_EACH_BACKEND(Write_Mission_Cmd(mission, cmd));
}

void AP_Logger::Write_RallyPoint(uint8_t total,
                                 uint8_t sequence,
                                 const RallyLocation &rally_point)
{
    FOR_EACH_BACKEND(Write_RallyPoint(total, sequence, rally_point));
}

void AP_Logger::Write_Rally()
{
    FOR_EACH_BACKEND(Write_Rally());
}

uint32_t AP_Logger::num_dropped() const
{
    if (_next_backend == 0) {
        return 0;
    }
    return backends[0]->num_dropped();
}


// end functions pass straight through to backend

/* Write support */
void AP_Logger::Write(const char *name, const char *labels, const char *fmt, ...)
{
    va_list arg_list;

    va_start(arg_list, fmt);
    WriteV(name, labels, nullptr, nullptr, fmt, arg_list);
    va_end(arg_list);
}

void AP_Logger::Write(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, ...)
{
    va_list arg_list;

    va_start(arg_list, fmt);
    WriteV(name, labels, units, mults, fmt, arg_list);
    va_end(arg_list);
}

void AP_Logger::WriteCritical(const char *name, const char *labels, const char *fmt, ...)
{
    va_list arg_list;

    va_start(arg_list, fmt);
    WriteV(name, labels, nullptr, nullptr, fmt, arg_list, true);
    va_end(arg_list);
}

void AP_Logger::WriteCritical(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, ...)
{
    va_list arg_list;

    va_start(arg_list, fmt);
    WriteV(name, labels, units, mults, fmt, arg_list, true);
    va_end(arg_list);
}

void AP_Logger::WriteV(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, va_list arg_list, bool is_critical)
{
    struct log_write_fmt *f = msg_fmt_for_name(name, labels, units, mults, fmt);
    if (f == nullptr) {
        // unable to map name to a messagetype; could be out of
        // msgtypes, could be out of slots, ...
        AP::internalerror().error(AP_InternalError::error_t::logger_mapfailure);
        return;
    }

    for (uint8_t i=0; i<_next_backend; i++) {
        if (!(f->sent_mask & (1U<<i))) {
            if (!backends[i]->Write_Emit_FMT(f->msg_type)) {
                continue;
            }
            f->sent_mask |= (1U<<i);
        }
        va_list arg_copy;
        va_copy(arg_copy, arg_list);
        backends[i]->Write(f->msg_type, arg_copy, is_critical);
        va_end(arg_copy);
    }
}


#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
void AP_Logger::assert_same_fmt_for_name(const AP_Logger::log_write_fmt *f,
                                               const char *name,
                                               const char *labels,
                                               const char *units,
                                               const char *mults,
                                               const char *fmt) const
{
    bool passed = true;
    if (!streq(f->name, name)) {
        // why exactly were we called?!
        Debug("format names differ (%s) != (%s)", f->name, name);
        passed = false;
    }
    if (!streq(f->labels, labels)) {
        Debug("format labels differ (%s) vs (%s)", f->labels, labels);
        passed = false;
    }
    if ((f->units != nullptr && units == nullptr) ||
        (f->units == nullptr && units != nullptr) ||
        (units !=nullptr && !streq(f->units, units))) {
        Debug("format units differ (%s) vs (%s)",
              (f->units ? f->units : "nullptr"),
              (units ? units : "nullptr"));
        passed = false;
    }
    if ((f->mults != nullptr && mults == nullptr) ||
        (f->mults == nullptr && mults != nullptr) ||
        (mults != nullptr && !streq(f->mults, mults))) {
        Debug("format mults differ (%s) vs (%s)",
              (f->mults ? f->mults : "nullptr"),
              (mults ? mults : "nullptr"));
        passed = false;
    }
    if (!streq(f->fmt, fmt)) {
        Debug("format fmt differ (%s) vs (%s)",
              (f->fmt ? f->fmt : "nullptr"),
              (fmt ? fmt : "nullptr"));
        passed = false;
    }
    if (!passed) {
        Debug("Format definition must be consistent for every call of Write");
        abort();
    }
}
#endif

AP_Logger::log_write_fmt *AP_Logger::msg_fmt_for_name(const char *name, const char *labels, const char *units, const char *mults, const char *fmt)
{
    struct log_write_fmt *f;
    for (f = log_write_fmts; f; f=f->next) {
        if (f->name == name) { // ptr comparison
            // already have an ID for this name:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            assert_same_fmt_for_name(f, name, labels, units, mults, fmt);
#endif
            return f;
        }
    }
    f = (struct log_write_fmt *)calloc(1, sizeof(*f));
    if (f == nullptr) {
        // out of memory
        return nullptr;
    }
    // no message type allocated for this name.  Try to allocate one:
    int16_t msg_type = find_free_msg_type();
    if (msg_type == -1) {
        free(f);
        return nullptr;
    }
    f->msg_type = msg_type;
    f->name = name;
    f->fmt = fmt;
    f->labels = labels;
    f->units = units;
    f->mults = mults;

    int16_t tmp = Write_calc_msg_len(fmt);
    if (tmp == -1) {
        free(f);
        return nullptr;
    }

    f->msg_len = tmp;

    // add to front of list
    f->next = log_write_fmts;
    log_write_fmts = f;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    char ls_name[LS_NAME_SIZE] = {};
    char ls_format[LS_FORMAT_SIZE] = {};
    char ls_labels[LS_LABELS_SIZE] = {};
    char ls_units[LS_UNITS_SIZE] = {};
    char ls_multipliers[LS_MULTIPLIERS_SIZE] = {};
    struct LogStructure ls = {
        f->msg_type,
        f->msg_len,
        ls_name,
        ls_format,
        ls_labels,
        ls_units,
        ls_multipliers
    };
    memcpy((char*)ls_name, f->name, MIN(sizeof(ls_name), strlen(f->name)));
    memcpy((char*)ls_format, f->fmt, MIN(sizeof(ls_format), strlen(f->fmt)));
    memcpy((char*)ls_labels, f->labels, MIN(sizeof(ls_labels), strlen(f->labels)));
    if (f->units != nullptr) {
        memcpy((char*)ls_units, f->units, MIN(sizeof(ls_units), strlen(f->units)));
    } else {
        memset((char*)ls_units, '?', MIN(sizeof(ls_format), strlen(f->fmt)));
    }
    if (f->mults != nullptr) {
        memcpy((char*)ls_multipliers, f->mults, MIN(sizeof(ls_multipliers), strlen(f->mults)));
    } else {
        memset((char*)ls_multipliers, '?', MIN(sizeof(ls_format), strlen(f->fmt)));
    }
    if (!validate_structure(&ls, (int16_t)-1)) {
        Debug("Log structure invalid");
        abort();
    }
#endif

    return f;
}

const struct LogStructure *AP_Logger::structure_for_msg_type(const uint8_t msg_type)
{
    for (uint16_t i=0; i<_num_types;i++) {
        const struct LogStructure *s = structure(i);
        if (s->msg_type == msg_type) {
            // in use
            return s;
        }
    }
    return nullptr;
}

const struct AP_Logger::log_write_fmt *AP_Logger::log_write_fmt_for_msg_type(const uint8_t msg_type) const
{
    struct log_write_fmt *f;
    for (f = log_write_fmts; f; f=f->next) {
        if (f->msg_type == msg_type) {
            return f;
        }
    }
    return nullptr;
}


// returns true if the msg_type is already taken
bool AP_Logger::msg_type_in_use(const uint8_t msg_type) const
{
    // check static list of messages (e.g. from LOG_BASE_STRUCTURES)
    // check the write format types to see if we've used this one
    for (uint16_t i=0; i<_num_types;i++) {
        if (structure(i)->msg_type == msg_type) {
            // in use
            return true;
        }
    }

    struct log_write_fmt *f;
    for (f = log_write_fmts; f; f=f->next) {
        if (f->msg_type == msg_type) {
            return true;
        }
    }
    return false;
}

// find a free message type
int16_t AP_Logger::find_free_msg_type() const
{
    // avoid using 255 here; perhaps we want to use it to extend things later
    for (uint16_t msg_type=254; msg_type>0; msg_type--) { // more likely to be free at end
        if (! msg_type_in_use(msg_type)) {
            return msg_type;
        }
    }
    return -1;
}

/*
 * It is assumed that logstruct's char* variables are valid strings of
 * maximum lengths for those fields (given in LogStructure.h e.g. LS_NAME_SIZE)
 */
bool AP_Logger::fill_log_write_logstructure(struct LogStructure &logstruct, const uint8_t msg_type) const
{
    // find log structure information corresponding to msg_type:
    struct log_write_fmt *f;
    for (f = log_write_fmts; f; f=f->next) {
        if(f->msg_type == msg_type) {
            break;
        }
    }

    if (!f) {
        return false;
    }

    logstruct.msg_type = msg_type;
    strncpy((char*)logstruct.name, f->name, LS_NAME_SIZE);
    strncpy((char*)logstruct.format, f->fmt, LS_FORMAT_SIZE);
    strncpy((char*)logstruct.labels, f->labels, LS_LABELS_SIZE);
    if (f->units != nullptr) {
        strncpy((char*)logstruct.units, f->units, LS_UNITS_SIZE);
    } else {
        memset((char*)logstruct.units, '\0', LS_UNITS_SIZE);
        memset((char*)logstruct.units, '?', MIN(LS_UNITS_SIZE,strlen(logstruct.format)));
    }
    if (f->mults != nullptr) {
        strncpy((char*)logstruct.multipliers, f->mults, LS_MULTIPLIERS_SIZE);
    } else {
        memset((char*)logstruct.multipliers, '\0', LS_MULTIPLIERS_SIZE);
        memset((char*)logstruct.multipliers, '?', MIN(LS_MULTIPLIERS_SIZE, strlen(logstruct.format)));
        // special magic to set units/mults for TimeUS, by far and
        // away the most common first field
        if (!strncmp(logstruct.labels, "TimeUS,", MIN(LS_LABELS_SIZE, strlen("TimeUS,")))) {
            ((char*)(logstruct.units))[0] = 's';
            ((char*)(logstruct.multipliers))[0] = 'F';
        }
    }
    logstruct.msg_len = f->msg_len;
    return true;
}

/* calculate the length of output of a format string.  Note that this
 * returns an int16_t; if it returns -1 then an error has occurred.
 * This was mechanically converted from init_field_types in
 * Tools/Replay/MsgHandler.cpp */
int16_t AP_Logger::Write_calc_msg_len(const char *fmt) const
{
    uint8_t len =  LOG_PACKET_HEADER_LEN;
    for (uint8_t i=0; i<strlen(fmt); i++) {
        switch(fmt[i]) {
        case 'a' : len += sizeof(int16_t[32]); break;
        case 'b' : len += sizeof(int8_t); break;
        case 'c' : len += sizeof(int16_t); break;
        case 'd' : len += sizeof(double); break;
        case 'e' : len += sizeof(int32_t); break;
        case 'f' : len += sizeof(float); break;
        case 'h' : len += sizeof(int16_t); break;
        case 'i' : len += sizeof(int32_t); break;
        case 'n' : len += sizeof(char[4]); break;
        case 'B' : len += sizeof(uint8_t); break;
        case 'C' : len += sizeof(uint16_t); break;
        case 'E' : len += sizeof(uint32_t); break;
        case 'H' : len += sizeof(uint16_t); break;
        case 'I' : len += sizeof(uint32_t); break;
        case 'L' : len += sizeof(int32_t); break;
        case 'M' : len += sizeof(uint8_t); break;
        case 'N' : len += sizeof(char[16]); break;
        case 'Z' : len += sizeof(char[64]); break;
        case 'q' : len += sizeof(int64_t); break;
        case 'Q' : len += sizeof(uint64_t); break;
        default:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("Unknown format specifier (%c)", fmt[i]);
#endif
            return -1;
        }
    }
    return len;
}

/* End of Write support */

#undef FOR_EACH_BACKEND

// Write information about a series of IMU readings to log:
bool AP_Logger::Write_ISBH(const uint16_t seqno,
                                     const AP_InertialSensor::IMU_SENSOR_TYPE sensor_type,
                                     const uint8_t sensor_instance,
                                     const uint16_t mult,
                                     const uint16_t sample_count,
                                     const uint64_t sample_us,
                                     const float sample_rate_hz)
{
    if (_next_backend == 0) {
        return false;
    }
    const struct log_ISBH pkt{
        LOG_PACKET_HEADER_INIT(LOG_ISBH_MSG),
        time_us        : AP_HAL::micros64(),
        seqno          : seqno,
        sensor_type    : (uint8_t)sensor_type,
        instance       : sensor_instance,
        multiplier     : mult,
        sample_count   : sample_count,
        sample_us      : sample_us,
        sample_rate_hz : sample_rate_hz,
    };

    // only the first backend need succeed for us to be successful
    for (uint8_t i=1; i<_next_backend; i++) {
        backends[i]->WriteBlock(&pkt, sizeof(pkt));
    }

    return backends[0]->WriteBlock(&pkt, sizeof(pkt));
}


// Write a series of IMU readings to log:
bool AP_Logger::Write_ISBD(const uint16_t isb_seqno,
                                     const uint16_t seqno,
                                     const int16_t x[32],
                                     const int16_t y[32],
                                     const int16_t z[32])
{
    if (_next_backend == 0) {
        return false;
    }
    struct log_ISBD pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ISBD_MSG),
        time_us    : AP_HAL::micros64(),
        isb_seqno  : isb_seqno,
        seqno      : seqno
    };
    memcpy(pkt.x, x, sizeof(pkt.x));
    memcpy(pkt.y, y, sizeof(pkt.y));
    memcpy(pkt.z, z, sizeof(pkt.z));

    // only the first backend need succeed for us to be successful
    for (uint8_t i=1; i<_next_backend; i++) {
        backends[i]->WriteBlock(&pkt, sizeof(pkt));
    }

    return backends[0]->WriteBlock(&pkt, sizeof(pkt));
}

// Wrote an event packet
void AP_Logger::Write_Event(Log_Event id)
{
    const struct log_Event pkt{
        LOG_PACKET_HEADER_INIT(LOG_EVENT_MSG),
        time_us  : AP_HAL::micros64(),
        id       : id
    };
    WriteCriticalBlock(&pkt, sizeof(pkt));
}

// Write an error packet
void AP_Logger::Write_Error(LogErrorSubsystem sub_system,
                            LogErrorCode error_code)
{
  const struct log_Error pkt{
      LOG_PACKET_HEADER_INIT(LOG_ERROR_MSG),
      time_us       : AP_HAL::micros64(),
      sub_system    : uint8_t(sub_system),
      error_code    : uint8_t(error_code),
  };
  WriteCriticalBlock(&pkt, sizeof(pkt));
}

/*
  return true if we should log while disarmed
 */
bool AP_Logger::log_while_disarmed(void) const
{
    if (_force_log_disarmed) {
        return true;
    }
    if (_params.log_disarmed != 0) {
        return true;
    }

    uint32_t now = AP_HAL::millis();
    uint32_t persist_ms = HAL_LOGGER_ARM_PERSIST*1000U;

    // keep logging for HAL_LOGGER_ARM_PERSIST seconds after disarming
    const uint32_t arm_change_ms = hal.util->get_last_armed_change();
    if (!hal.util->get_soft_armed() && arm_change_ms != 0 && now - arm_change_ms < persist_ms) {
        return true;
    }

    // keep logging for HAL_LOGGER_ARM_PERSIST seconds after an arming failure
    if (_last_arming_failure_ms && now - _last_arming_failure_ms < persist_ms) {
        return true;
    }

    return false;
}

namespace AP {

AP_Logger &logger()
{
    return *AP_Logger::get_singleton();
}

};
