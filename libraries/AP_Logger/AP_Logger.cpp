#include "AP_Logger.h"

#include "AP_Logger_Backend.h"

#include "AP_Logger_File.h"
#include "AP_Logger_SITL.h"
#include "AP_Logger_DataFlash.h"
#include "AP_Logger_MAVLink.h"

#include <AP_InternalError/AP_InternalError.h>
#include <GCS_MAVLink/GCS.h>

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_Motors/AP_Motors.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <AC_AttitudeControl/AC_PosControl.h>
#include <AP_RSSI/AP_RSSI.h>

AP_Logger *AP_Logger::_singleton;

extern const AP_HAL::HAL& hal;

#ifndef HAL_LOGGING_FILE_BUFSIZE
#define HAL_LOGGING_FILE_BUFSIZE  16
#endif 

#ifndef HAL_LOGGING_MAV_BUFSIZE
#define HAL_LOGGING_MAV_BUFSIZE  8
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

#if defined(HAL_BOARD_LOG_DIRECTORY)
 #if HAL_OS_POSIX_IO || HAL_OS_FATFS_IO
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
 #endif
#endif // HAL_BOARD_LOG_DIRECTORY

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
#define Debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)

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
    if (passed) {
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
    if (!(mask & _log_bitmask)) {
        return false;
    }
    if (!vehicle_is_armed() && !log_while_disarmed()) {
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

void AP_Logger::handle_mavlink_msg(GCS_MAVLINK &link, mavlink_message_t* msg)
{
    switch (msg->msgid) {
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

void AP_Logger::WriteV(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, va_list arg_list)
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
        backends[i]->Write(f->msg_type, arg_copy);
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
    struct log_ISBH pkt = {
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
    struct log_Event pkt = {
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
  struct log_Error pkt = {
      LOG_PACKET_HEADER_INIT(LOG_ERROR_MSG),
      time_us       : AP_HAL::micros64(),
      sub_system    : uint8_t(sub_system),
      error_code    : uint8_t(error_code),
  };
  WriteCriticalBlock(&pkt, sizeof(pkt));
}

// Write ESC status messages
//   id starts from 0
//   rpm is eRPM (rpm * 100)
//   voltage is in centi-volts
//   current is in centi-amps
//   temperature is in centi-degrees Celsius
//   current_tot is in centi-amp hours
void AP_Logger::Write_ESC(uint8_t id, uint64_t time_us, int32_t rpm, uint16_t voltage, uint16_t current, int16_t temperature, uint16_t current_tot)
{
    // sanity check id
    if (id >= 8) {
        return;
    }
    struct log_Esc pkt = {
        LOG_PACKET_HEADER_INIT(uint8_t(LOG_ESC1_MSG+id)),
        time_us     : time_us,
        rpm         : rpm,
        voltage     : voltage,
        current     : current,
        temperature : temperature,
        current_tot : current_tot
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write a Yaw PID packet
void AP_Logger::Write_PID(uint8_t msg_type, const PID_Info &info)
{
    struct log_PID pkt = {
        LOG_PACKET_HEADER_INIT(msg_type),
        time_us         : AP_HAL::micros64(),
        desired         : info.desired,
        actual          : info.actual,
        P               : info.P,
        I               : info.I,
        D               : info.D,
        FF              : info.FF
    };
    WriteBlock(&pkt, sizeof(pkt));
}

void AP_Logger::Write_Origin(uint8_t origin_type, const Location &loc)
{
    uint64_t time_us = AP_HAL::micros64();
    struct log_ORGN pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ORGN_MSG),
        time_us     : time_us,
        origin_type : origin_type,
        latitude    : loc.lat,
        longitude   : loc.lng,
        altitude    : loc.alt
    };
    WriteBlock(&pkt, sizeof(pkt));
}

void AP_Logger::Write_RPM(const AP_RPM &rpm_sensor)
{
    struct log_RPM pkt = {
        LOG_PACKET_HEADER_INIT(LOG_RPM_MSG),
        time_us     : AP_HAL::micros64(),
        rpm1        : rpm_sensor.get_rpm(0),
        rpm2        : rpm_sensor.get_rpm(1)
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write a rate packet
void AP_Logger::Write_Rate(const AP_AHRS_View *ahrs,
                                     const AP_Motors &motors,
                                     const AC_AttitudeControl &attitude_control,
                                     const AC_PosControl &pos_control)
{
    const Vector3f &rate_targets = attitude_control.rate_bf_targets();
    const Vector3f &accel_target = pos_control.get_accel_target();
    struct log_Rate pkt_rate = {
        LOG_PACKET_HEADER_INIT(LOG_RATE_MSG),
        time_us         : AP_HAL::micros64(),
        control_roll    : degrees(rate_targets.x),
        roll            : degrees(ahrs->get_gyro().x),
        roll_out        : motors.get_roll(),
        control_pitch   : degrees(rate_targets.y),
        pitch           : degrees(ahrs->get_gyro().y),
        pitch_out       : motors.get_pitch(),
        control_yaw     : degrees(rate_targets.z),
        yaw             : degrees(ahrs->get_gyro().z),
        yaw_out         : motors.get_yaw(),
        control_accel   : (float)accel_target.z,
        accel           : (float)(-(ahrs->get_accel_ef_blended().z + GRAVITY_MSS) * 100.0f),
        accel_out       : motors.get_throttle()
    };
    WriteBlock(&pkt_rate, sizeof(pkt_rate));
}

// Write visual odometry sensor data
void AP_Logger::Write_VisualOdom(float time_delta, const Vector3f &angle_delta, const Vector3f &position_delta, float confidence)
{
    struct log_VisualOdom pkt_visualodom = {
        LOG_PACKET_HEADER_INIT(LOG_VISUALODOM_MSG),
        time_us             : AP_HAL::micros64(),
        time_delta          : time_delta,
        angle_delta_x       : angle_delta.x,
        angle_delta_y       : angle_delta.y,
        angle_delta_z       : angle_delta.z,
        position_delta_x    : position_delta.x,
        position_delta_y    : position_delta.y,
        position_delta_z    : position_delta.z,
        confidence          : confidence
    };
    WriteBlock(&pkt_visualodom, sizeof(log_VisualOdom));
}

// Write AOA and SSA
void AP_Logger::Write_AOA_SSA(AP_AHRS &ahrs)
{
    struct log_AOA_SSA aoa_ssa = {
        LOG_PACKET_HEADER_INIT(LOG_AOA_SSA_MSG),
        time_us         : AP_HAL::micros64(),
        AOA             : ahrs.getAOA(),
        SSA             : ahrs.getSSA()
    };

    WriteBlock(&aoa_ssa, sizeof(aoa_ssa));
}

// Write beacon sensor (position) data
void AP_Logger::Write_Beacon(AP_Beacon &beacon)
{
    if (!beacon.enabled()) {
        return;
    }
    // position
    Vector3f pos;
    float accuracy = 0.0f;
    beacon.get_vehicle_position_ned(pos, accuracy);

    struct log_Beacon pkt_beacon = {
       LOG_PACKET_HEADER_INIT(LOG_BEACON_MSG),
       time_us         : AP_HAL::micros64(),
       health          : (uint8_t)beacon.healthy(),
       count           : (uint8_t)beacon.count(),
       dist0           : beacon.beacon_distance(0),
       dist1           : beacon.beacon_distance(1),
       dist2           : beacon.beacon_distance(2),
       dist3           : beacon.beacon_distance(3),
       posx            : pos.x,
       posy            : pos.y,
       posz            : pos.z
    };
    WriteBlock(&pkt_beacon, sizeof(pkt_beacon));
}

// Write proximity sensor distances
void AP_Logger::Write_Proximity(AP_Proximity &proximity)
{
    // exit immediately if not enabled
    if (proximity.get_status() == AP_Proximity::Proximity_NotConnected) {
        return;
    }

    AP_Proximity::Proximity_Distance_Array dist_array {};
    proximity.get_horizontal_distances(dist_array);

    float dist_up;
    if (!proximity.get_upward_distance(dist_up)) {
        dist_up = 0.0f;
    }

    float close_ang = 0.0f, close_dist = 0.0f;
    proximity.get_closest_object(close_ang, close_dist);

    struct log_Proximity pkt_proximity = {
            LOG_PACKET_HEADER_INIT(LOG_PROXIMITY_MSG),
            time_us         : AP_HAL::micros64(),
            health          : (uint8_t)proximity.get_status(),
            dist0           : dist_array.distance[0],
            dist45          : dist_array.distance[1],
            dist90          : dist_array.distance[2],
            dist135         : dist_array.distance[3],
            dist180         : dist_array.distance[4],
            dist225         : dist_array.distance[5],
            dist270         : dist_array.distance[6],
            dist315         : dist_array.distance[7],
            distup          : dist_up,
            closest_angle   : close_ang,
            closest_dist    : close_dist
    };
    WriteBlock(&pkt_proximity, sizeof(pkt_proximity));
}

void AP_Logger::Write_SRTL(bool active, uint16_t num_points, uint16_t max_points, uint8_t action, const Vector3f& breadcrumb)
{
    struct log_SRTL pkt_srtl = {
        LOG_PACKET_HEADER_INIT(LOG_SRTL_MSG),
        time_us         : AP_HAL::micros64(),
        active          : active,
        num_points      : num_points,
        max_points      : max_points,
        action          : action,
        N               : breadcrumb.x,
        E               : breadcrumb.y,
        D               : breadcrumb.z
    };
    WriteBlock(&pkt_srtl, sizeof(pkt_srtl));
}

void AP_Logger::Write_Power(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    uint8_t safety_and_armed = uint8_t(hal.util->safety_switch_state());
    if (hal.util->get_soft_armed()) {
        // encode armed state in bit 3
        safety_and_armed |= 1U<<2;
    }
    struct log_POWR pkt = {
        LOG_PACKET_HEADER_INIT(LOG_POWR_MSG),
        time_us : AP_HAL::micros64(),
        Vcc     : hal.analogin->board_voltage(),
        Vservo  : hal.analogin->servorail_voltage(),
        flags   : hal.analogin->power_status_flags(),
        safety_and_arm : safety_and_armed
    };
    WriteBlock(&pkt, sizeof(pkt));
#endif
}

// Write an AHRS2 packet
void AP_Logger::Write_AHRS2(AP_AHRS &ahrs)
{
    Vector3f euler;
    struct Location loc;
    Quaternion quat;
    if (!ahrs.get_secondary_attitude(euler) || !ahrs.get_secondary_position(loc) || !ahrs.get_secondary_quaternion(quat)) {
        return;
    }
    struct log_AHRS pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AHR2_MSG),
        time_us : AP_HAL::micros64(),
        roll  : (int16_t)(degrees(euler.x)*100),
        pitch : (int16_t)(degrees(euler.y)*100),
        yaw   : (uint16_t)(wrap_360_cd(degrees(euler.z)*100)),
        alt   : loc.alt*1.0e-2f,
        lat   : loc.lat,
        lng   : loc.lng,
        q1    : quat.q1,
        q2    : quat.q2,
        q3    : quat.q3,
        q4    : quat.q4,
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write a POS packet
void AP_Logger::Write_POS(AP_AHRS &ahrs)
{
    Location loc;
    if (!ahrs.get_position(loc)) {
        return;
    }
    float home, origin;
    ahrs.get_relative_position_D_home(home);
    struct log_POS pkt = {
        LOG_PACKET_HEADER_INIT(LOG_POS_MSG),
        time_us        : AP_HAL::micros64(),
        lat            : loc.lat,
        lng            : loc.lng,
        alt            : loc.alt*1.0e-2f,
        rel_home_alt   : -home,
        rel_origin_alt : ahrs.get_relative_position_D_origin(origin) ? -origin : quiet_nanf(),
    };
    WriteBlock(&pkt, sizeof(pkt));
}

#if AP_AHRS_NAVEKF_AVAILABLE
void AP_Logger::Write_EKF(AP_AHRS_NavEKF &ahrs)
{
    // only log EKF2 if enabled
    if (ahrs.get_NavEKF2().activeCores() > 0) {
        Write_EKF2(ahrs);
    }
    // only log EKF3 if enabled
    if (ahrs.get_NavEKF3().activeCores() > 0) {
        Write_EKF3(ahrs);
    }
}


/*
  write an EKF timing message
 */
void AP_Logger::Write_EKF_Timing(const char *name, uint64_t time_us, const struct ekf_timing &timing)
{
    Write(name,
              "TimeUS,Cnt,IMUMin,IMUMax,EKFMin,EKFMax,AngMin,AngMax,VMin,VMax",
              "QIffffffff",
              time_us,
              timing.count,
              (double)timing.dtIMUavg_min,
              (double)timing.dtIMUavg_max,
              (double)timing.dtEKFavg_min,
              (double)timing.dtEKFavg_max,
              (double)timing.delAngDT_min,
              (double)timing.delAngDT_max,
              (double)timing.delVelDT_min,
              (double)timing.delVelDT_max);
}

void AP_Logger::Write_EKF2(AP_AHRS_NavEKF &ahrs)
{
    uint64_t time_us = AP_HAL::micros64();
    // Write first EKF packet
    Vector3f euler;
    Vector2f posNE;
    float posD;
    Vector3f velNED;
    Vector3f gyroBias;
    float posDownDeriv;
    Location originLLH;
    ahrs.get_NavEKF2().getEulerAngles(0,euler);
    ahrs.get_NavEKF2().getVelNED(0,velNED);
    ahrs.get_NavEKF2().getPosNE(0,posNE);
    ahrs.get_NavEKF2().getPosD(0,posD);
    ahrs.get_NavEKF2().getGyroBias(0,gyroBias);
    posDownDeriv = ahrs.get_NavEKF2().getPosDownDerivative(0);
    if (!ahrs.get_NavEKF2().getOriginLLH(0,originLLH)) {
        originLLH.alt = 0;
    }
    struct log_EKF1 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NKF1_MSG),
        time_us : time_us,
        roll    : (int16_t)(100*degrees(euler.x)), // roll angle (centi-deg, displayed as deg due to format string)
        pitch   : (int16_t)(100*degrees(euler.y)), // pitch angle (centi-deg, displayed as deg due to format string)
        yaw     : (uint16_t)wrap_360_cd(100*degrees(euler.z)), // yaw angle (centi-deg, displayed as deg due to format string)
        velN    : (float)(velNED.x), // velocity North (m/s)
        velE    : (float)(velNED.y), // velocity East (m/s)
        velD    : (float)(velNED.z), // velocity Down (m/s)
        posD_dot : (float)(posDownDeriv), // first derivative of down position
        posN    : (float)(posNE.x), // metres North
        posE    : (float)(posNE.y), // metres East
        posD    : (float)(posD), // metres Down
        gyrX    : (int16_t)(100*degrees(gyroBias.x)), // cd/sec, displayed as deg/sec due to format string
        gyrY    : (int16_t)(100*degrees(gyroBias.y)), // cd/sec, displayed as deg/sec due to format string
        gyrZ    : (int16_t)(100*degrees(gyroBias.z)), // cd/sec, displayed as deg/sec due to format string
        originHgt : originLLH.alt // WGS-84 altitude of EKF origin in cm
    };
    WriteBlock(&pkt, sizeof(pkt));

    // Write second EKF packet
    float azbias = 0;
    Vector3f wind;
    Vector3f magNED;
    Vector3f magXYZ;
    Vector3f gyroScaleFactor;
    uint8_t magIndex = ahrs.get_NavEKF2().getActiveMag(0);
    ahrs.get_NavEKF2().getAccelZBias(0,azbias);
    ahrs.get_NavEKF2().getWind(0,wind);
    ahrs.get_NavEKF2().getMagNED(0,magNED);
    ahrs.get_NavEKF2().getMagXYZ(0,magXYZ);
    ahrs.get_NavEKF2().getGyroScaleErrorPercentage(0,gyroScaleFactor);
    struct log_NKF2 pkt2 = {
        LOG_PACKET_HEADER_INIT(LOG_NKF2_MSG),
        time_us : time_us,
        AZbias  : (int8_t)(100*azbias),
        scaleX  : (int16_t)(100*gyroScaleFactor.x),
        scaleY  : (int16_t)(100*gyroScaleFactor.y),
        scaleZ  : (int16_t)(100*gyroScaleFactor.z),
        windN   : (int16_t)(100*wind.x),
        windE   : (int16_t)(100*wind.y),
        magN    : (int16_t)(magNED.x),
        magE    : (int16_t)(magNED.y),
        magD    : (int16_t)(magNED.z),
        magX    : (int16_t)(magXYZ.x),
        magY    : (int16_t)(magXYZ.y),
        magZ    : (int16_t)(magXYZ.z),
        index   : (uint8_t)(magIndex)
    };
    WriteBlock(&pkt2, sizeof(pkt2));

    // Write third EKF packet
    Vector3f velInnov;
    Vector3f posInnov;
    Vector3f magInnov;
    float tasInnov = 0;
    float yawInnov = 0;
    ahrs.get_NavEKF2().getInnovations(0,velInnov, posInnov, magInnov, tasInnov, yawInnov);
    struct log_NKF3 pkt3 = {
        LOG_PACKET_HEADER_INIT(LOG_NKF3_MSG),
        time_us : time_us,
        innovVN : (int16_t)(100*velInnov.x),
        innovVE : (int16_t)(100*velInnov.y),
        innovVD : (int16_t)(100*velInnov.z),
        innovPN : (int16_t)(100*posInnov.x),
        innovPE : (int16_t)(100*posInnov.y),
        innovPD : (int16_t)(100*posInnov.z),
        innovMX : (int16_t)(magInnov.x),
        innovMY : (int16_t)(magInnov.y),
        innovMZ : (int16_t)(magInnov.z),
        innovYaw : (int16_t)(100*degrees(yawInnov)),
        innovVT : (int16_t)(100*tasInnov)
    };
    WriteBlock(&pkt3, sizeof(pkt3));

    // Write fourth EKF packet
    float velVar = 0;
    float posVar = 0;
    float hgtVar = 0;
    Vector3f magVar;
    float tasVar = 0;
    Vector2f offset;
    uint16_t faultStatus=0;
    uint8_t timeoutStatus=0;
    nav_filter_status solutionStatus {};
    nav_gps_status gpsStatus {};
    ahrs.get_NavEKF2().getVariances(0,velVar, posVar, hgtVar, magVar, tasVar, offset);
    float tempVar = fmaxf(fmaxf(magVar.x,magVar.y),magVar.z);
    ahrs.get_NavEKF2().getFilterFaults(0,faultStatus);
    ahrs.get_NavEKF2().getFilterTimeouts(0,timeoutStatus);
    ahrs.get_NavEKF2().getFilterStatus(0,solutionStatus);
    ahrs.get_NavEKF2().getFilterGpsStatus(0,gpsStatus);
    float tiltError;
    ahrs.get_NavEKF2().getTiltError(0,tiltError);
    int8_t primaryIndex = ahrs.get_NavEKF2().getPrimaryCoreIndex();
    struct log_NKF4 pkt4 = {
        LOG_PACKET_HEADER_INIT(LOG_NKF4_MSG),
        time_us : time_us,
        sqrtvarV : (int16_t)(100*velVar),
        sqrtvarP : (int16_t)(100*posVar),
        sqrtvarH : (int16_t)(100*hgtVar),
        sqrtvarM : (int16_t)(100*tempVar),
        sqrtvarVT : (int16_t)(100*tasVar),
        tiltErr : (float)tiltError,
        offsetNorth : (int8_t)(offset.x),
        offsetEast : (int8_t)(offset.y),
        faults : (uint16_t)(faultStatus),
        timeouts : (uint8_t)(timeoutStatus),
        solution : (uint16_t)(solutionStatus.value),
        gps : (uint16_t)(gpsStatus.value),
        primary : (int8_t)primaryIndex
    };
    WriteBlock(&pkt4, sizeof(pkt4));

    // Write fifth EKF packet - take data from the primary instance
    float normInnov=0; // normalised innovation variance ratio for optical flow observations fused by the main nav filter
    float gndOffset=0; // estimated vertical position of the terrain relative to the nav filter zero datum
    float flowInnovX=0, flowInnovY=0; // optical flow LOS rate vector innovations from the main nav filter
    float auxFlowInnov=0; // optical flow LOS rate innovation from terrain offset estimator
    float HAGL=0; // height above ground level
    float rngInnov=0; // range finder innovations
    float range=0; // measured range
    float gndOffsetErr=0; // filter ground offset state error
    Vector3f predictorErrors; // output predictor angle, velocity and position tracking error
    ahrs.get_NavEKF2().getFlowDebug(-1,normInnov, gndOffset, flowInnovX, flowInnovY, auxFlowInnov, HAGL, rngInnov, range, gndOffsetErr);
    ahrs.get_NavEKF2().getOutputTrackingError(-1,predictorErrors);
    struct log_NKF5 pkt5 = {
        LOG_PACKET_HEADER_INIT(LOG_NKF5_MSG),
        time_us : time_us,
        normInnov : (uint8_t)(MIN(100*normInnov,255)),
        FIX : (int16_t)(1000*flowInnovX),
        FIY : (int16_t)(1000*flowInnovY),
        AFI : (int16_t)(1000*auxFlowInnov),
        HAGL : (int16_t)(100*HAGL),
        offset : (int16_t)(100*gndOffset),
        RI : (int16_t)(100*rngInnov),
        meaRng : (uint16_t)(100*range),
        errHAGL : (uint16_t)(100*gndOffsetErr),
        angErr : (float)predictorErrors.x,
        velErr : (float)predictorErrors.y,
        posErr : (float)predictorErrors.z
     };
    WriteBlock(&pkt5, sizeof(pkt5));

    // log quaternion
    Quaternion quat;
    ahrs.get_NavEKF2().getQuaternion(0, quat);
    struct log_Quaternion pktq1 = {
        LOG_PACKET_HEADER_INIT(LOG_NKQ1_MSG),
        time_us : time_us,
        q1 : quat.q1,
        q2 : quat.q2,
        q3 : quat.q3,
        q4 : quat.q4
    };
    WriteBlock(&pktq1, sizeof(pktq1));

    // log innovations for the second IMU if enabled
    if (ahrs.get_NavEKF2().activeCores() >= 2) {
        // Write 6th EKF packet
        ahrs.get_NavEKF2().getEulerAngles(1,euler);
        ahrs.get_NavEKF2().getVelNED(1,velNED);
        ahrs.get_NavEKF2().getPosNE(1,posNE);
        ahrs.get_NavEKF2().getPosD(1,posD);
        ahrs.get_NavEKF2().getGyroBias(1,gyroBias);
        posDownDeriv = ahrs.get_NavEKF2().getPosDownDerivative(1);
        if (!ahrs.get_NavEKF2().getOriginLLH(1,originLLH)) {
            originLLH.alt = 0;
        }
        struct log_EKF1 pkt6 = {
            LOG_PACKET_HEADER_INIT(LOG_NKF6_MSG),
            time_us : time_us,
            roll    : (int16_t)(100*degrees(euler.x)), // roll angle (centi-deg, displayed as deg due to format string)
            pitch   : (int16_t)(100*degrees(euler.y)), // pitch angle (centi-deg, displayed as deg due to format string)
            yaw     : (uint16_t)wrap_360_cd(100*degrees(euler.z)), // yaw angle (centi-deg, displayed as deg due to format string)
            velN    : (float)(velNED.x), // velocity North (m/s)
            velE    : (float)(velNED.y), // velocity East (m/s)
            velD    : (float)(velNED.z), // velocity Down (m/s)
            posD_dot : (float)(posDownDeriv), // first derivative of down position
            posN    : (float)(posNE.x), // metres North
            posE    : (float)(posNE.y), // metres East
            posD    : (float)(posD), // metres Down
            gyrX    : (int16_t)(100*degrees(gyroBias.x)), // cd/sec, displayed as deg/sec due to format string
            gyrY    : (int16_t)(100*degrees(gyroBias.y)), // cd/sec, displayed as deg/sec due to format string
            gyrZ    : (int16_t)(100*degrees(gyroBias.z)), // cd/sec, displayed as deg/sec due to format string
            originHgt : originLLH.alt // WGS-84 altitude of EKF origin in cm
        };
        WriteBlock(&pkt6, sizeof(pkt6));

        // Write 7th EKF packet
        ahrs.get_NavEKF2().getAccelZBias(1,azbias);
        ahrs.get_NavEKF2().getWind(1,wind);
        ahrs.get_NavEKF2().getMagNED(1,magNED);
        ahrs.get_NavEKF2().getMagXYZ(1,magXYZ);
        ahrs.get_NavEKF2().getGyroScaleErrorPercentage(1,gyroScaleFactor);
        magIndex = ahrs.get_NavEKF2().getActiveMag(1);
        struct log_NKF2 pkt7 = {
            LOG_PACKET_HEADER_INIT(LOG_NKF7_MSG),
            time_us : time_us,
            AZbias  : (int8_t)(100*azbias),
            scaleX  : (int16_t)(100*gyroScaleFactor.x),
            scaleY  : (int16_t)(100*gyroScaleFactor.y),
            scaleZ  : (int16_t)(100*gyroScaleFactor.z),
            windN   : (int16_t)(100*wind.x),
            windE   : (int16_t)(100*wind.y),
            magN    : (int16_t)(magNED.x),
            magE    : (int16_t)(magNED.y),
            magD    : (int16_t)(magNED.z),
            magX    : (int16_t)(magXYZ.x),
            magY    : (int16_t)(magXYZ.y),
            magZ    : (int16_t)(magXYZ.z),
            index   : (uint8_t)(magIndex)
        };
        WriteBlock(&pkt7, sizeof(pkt7));

        // Write 8th EKF packet
        ahrs.get_NavEKF2().getInnovations(1,velInnov, posInnov, magInnov, tasInnov, yawInnov);
        struct log_NKF3 pkt8 = {
            LOG_PACKET_HEADER_INIT(LOG_NKF8_MSG),
            time_us : time_us,
            innovVN : (int16_t)(100*velInnov.x),
            innovVE : (int16_t)(100*velInnov.y),
            innovVD : (int16_t)(100*velInnov.z),
            innovPN : (int16_t)(100*posInnov.x),
            innovPE : (int16_t)(100*posInnov.y),
            innovPD : (int16_t)(100*posInnov.z),
            innovMX : (int16_t)(magInnov.x),
            innovMY : (int16_t)(magInnov.y),
            innovMZ : (int16_t)(magInnov.z),
            innovYaw : (int16_t)(100*degrees(yawInnov)),
            innovVT : (int16_t)(100*tasInnov)
        };
        WriteBlock(&pkt8, sizeof(pkt8));

        // Write 9th EKF packet
        ahrs.get_NavEKF2().getVariances(1,velVar, posVar, hgtVar, magVar, tasVar, offset);
        tempVar = fmaxf(fmaxf(magVar.x,magVar.y),magVar.z);
        ahrs.get_NavEKF2().getFilterFaults(1,faultStatus);
        ahrs.get_NavEKF2().getFilterTimeouts(1,timeoutStatus);
        ahrs.get_NavEKF2().getFilterStatus(1,solutionStatus);
        ahrs.get_NavEKF2().getFilterGpsStatus(1,gpsStatus);
        ahrs.get_NavEKF2().getTiltError(1,tiltError);
        struct log_NKF4 pkt9 = {
            LOG_PACKET_HEADER_INIT(LOG_NKF9_MSG),
            time_us : time_us,
            sqrtvarV : (int16_t)(100*velVar),
            sqrtvarP : (int16_t)(100*posVar),
            sqrtvarH : (int16_t)(100*hgtVar),
            sqrtvarM : (int16_t)(100*tempVar),
            sqrtvarVT : (int16_t)(100*tasVar),
            tiltErr : (float)tiltError,
            offsetNorth : (int8_t)(offset.x),
            offsetEast : (int8_t)(offset.y),
            faults : (uint16_t)(faultStatus),
            timeouts : (uint8_t)(timeoutStatus),
            solution : (uint16_t)(solutionStatus.value),
            gps : (uint16_t)(gpsStatus.value),
            primary : (int8_t)primaryIndex
        };
        WriteBlock(&pkt9, sizeof(pkt9));

        ahrs.get_NavEKF2().getQuaternion(1, quat);
        struct log_Quaternion pktq2 = {
            LOG_PACKET_HEADER_INIT(LOG_NKQ2_MSG),
            time_us : time_us,
            q1 : quat.q1,
            q2 : quat.q2,
            q3 : quat.q3,
            q4 : quat.q4
        };
        WriteBlock(&pktq2, sizeof(pktq2));
    }

    // write range beacon fusion debug packet if the range value is non-zero
    if (ahrs.get_beacon() != nullptr) {
        uint8_t ID;
        float rng;
        float innovVar;
        float innov;
        float testRatio;
        Vector3f beaconPosNED;
        float bcnPosOffsetHigh;
        float bcnPosOffsetLow;
        if (ahrs.get_NavEKF2().getRangeBeaconDebug(-1, ID, rng, innov, innovVar, testRatio, beaconPosNED, bcnPosOffsetHigh, bcnPosOffsetLow)) {
            if (rng > 0.0f) {
                struct log_RngBcnDebug pkt10 = {
                    LOG_PACKET_HEADER_INIT(LOG_NKF10_MSG),
                    time_us : time_us,
                    ID : (uint8_t)ID,
                    rng : (int16_t)(100*rng),
                    innov : (int16_t)(100*innov),
                    sqrtInnovVar : (uint16_t)(100*safe_sqrt(innovVar)),
                    testRatio : (uint16_t)(100*constrain_float(testRatio,0.0f,650.0f)),
                    beaconPosN : (int16_t)(100*beaconPosNED.x),
                    beaconPosE : (int16_t)(100*beaconPosNED.y),
                    beaconPosD : (int16_t)(100*beaconPosNED.z),
                    offsetHigh : (int16_t)(100*bcnPosOffsetHigh),
                    offsetLow : (int16_t)(100*bcnPosOffsetLow),
                    posN : 0,
                    posE : 0,
                    posD : 0
                };
                WriteBlock(&pkt10, sizeof(pkt10));
            }
        }
    }

    // log EKF timing statistics every 5s
    static uint32_t lastTimingLogTime_ms = 0;
    if (AP_HAL::millis() - lastTimingLogTime_ms > 5000) {
        lastTimingLogTime_ms = AP_HAL::millis();
        struct ekf_timing timing;
        for (uint8_t i=0; i<ahrs.get_NavEKF2().activeCores(); i++) {
            ahrs.get_NavEKF2().getTimingStatistics(i, timing);
            Write_EKF_Timing(i==0?"NKT1":"NKT2", time_us, timing);
        }
    }
}


void AP_Logger::Write_EKF3(AP_AHRS_NavEKF &ahrs)
{
    uint64_t time_us = AP_HAL::micros64();
	// Write first EKF packet
    Vector3f euler;
    Vector2f posNE;
    float posD;
    Vector3f velNED;
    Vector3f gyroBias;
    float posDownDeriv;
    Location originLLH;
    ahrs.get_NavEKF3().getEulerAngles(0,euler);
    ahrs.get_NavEKF3().getVelNED(0,velNED);
    ahrs.get_NavEKF3().getPosNE(0,posNE);
    ahrs.get_NavEKF3().getPosD(0,posD);
    ahrs.get_NavEKF3().getGyroBias(0,gyroBias);
    posDownDeriv = ahrs.get_NavEKF3().getPosDownDerivative(0);
    if (!ahrs.get_NavEKF3().getOriginLLH(0,originLLH)) {
        originLLH.alt = 0;
    }
    struct log_EKF1 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_XKF1_MSG),
        time_us : time_us,
        roll    : (int16_t)(100*degrees(euler.x)), // roll angle (centi-deg, displayed as deg due to format string)
        pitch   : (int16_t)(100*degrees(euler.y)), // pitch angle (centi-deg, displayed as deg due to format string)
        yaw     : (uint16_t)wrap_360_cd(100*degrees(euler.z)), // yaw angle (centi-deg, displayed as deg due to format string)
        velN    : (float)(velNED.x), // velocity North (m/s)
        velE    : (float)(velNED.y), // velocity East (m/s)
        velD    : (float)(velNED.z), // velocity Down (m/s)
        posD_dot : (float)(posDownDeriv), // first derivative of down position
        posN    : (float)(posNE.x), // metres North
        posE    : (float)(posNE.y), // metres East
        posD    : (float)(posD), // metres Down
        gyrX    : (int16_t)(100*degrees(gyroBias.x)), // cd/sec, displayed as deg/sec due to format string
        gyrY    : (int16_t)(100*degrees(gyroBias.y)), // cd/sec, displayed as deg/sec due to format string
        gyrZ    : (int16_t)(100*degrees(gyroBias.z)), // cd/sec, displayed as deg/sec due to format string
        originHgt : originLLH.alt // WGS-84 altitude of EKF origin in cm
    };
    WriteBlock(&pkt, sizeof(pkt));

    // Write second EKF packet
    Vector3f accelBias;
    Vector3f wind;
    Vector3f magNED;
    Vector3f magXYZ;
    uint8_t magIndex = ahrs.get_NavEKF3().getActiveMag(0);
    ahrs.get_NavEKF3().getAccelBias(0,accelBias);
    ahrs.get_NavEKF3().getWind(0,wind);
    ahrs.get_NavEKF3().getMagNED(0,magNED);
    ahrs.get_NavEKF3().getMagXYZ(0,magXYZ);
    struct log_NKF2a pkt2 = {
        LOG_PACKET_HEADER_INIT(LOG_XKF2_MSG),
        time_us : time_us,
        accBiasX  : (int16_t)(100*accelBias.x),
        accBiasY  : (int16_t)(100*accelBias.y),
        accBiasZ  : (int16_t)(100*accelBias.z),
        windN   : (int16_t)(100*wind.x),
        windE   : (int16_t)(100*wind.y),
        magN    : (int16_t)(magNED.x),
        magE    : (int16_t)(magNED.y),
        magD    : (int16_t)(magNED.z),
        magX    : (int16_t)(magXYZ.x),
        magY    : (int16_t)(magXYZ.y),
        magZ    : (int16_t)(magXYZ.z),
        index   : (uint8_t)(magIndex)
    };
    WriteBlock(&pkt2, sizeof(pkt2));

    // Write third EKF packet
    Vector3f velInnov;
    Vector3f posInnov;
    Vector3f magInnov;
    float tasInnov = 0;
    float yawInnov = 0;
    ahrs.get_NavEKF3().getInnovations(0,velInnov, posInnov, magInnov, tasInnov, yawInnov);
    struct log_NKF3 pkt3 = {
        LOG_PACKET_HEADER_INIT(LOG_XKF3_MSG),
        time_us : time_us,
        innovVN : (int16_t)(100*velInnov.x),
        innovVE : (int16_t)(100*velInnov.y),
        innovVD : (int16_t)(100*velInnov.z),
        innovPN : (int16_t)(100*posInnov.x),
        innovPE : (int16_t)(100*posInnov.y),
        innovPD : (int16_t)(100*posInnov.z),
        innovMX : (int16_t)(magInnov.x),
        innovMY : (int16_t)(magInnov.y),
        innovMZ : (int16_t)(magInnov.z),
        innovYaw : (int16_t)(100*degrees(yawInnov)),
        innovVT : (int16_t)(100*tasInnov)
    };
    WriteBlock(&pkt3, sizeof(pkt3));

    // Write fourth EKF packet
    float velVar = 0;
    float posVar = 0;
    float hgtVar = 0;
    Vector3f magVar;
    float tasVar = 0;
    Vector2f offset;
    uint16_t faultStatus=0;
    uint8_t timeoutStatus=0;
    nav_filter_status solutionStatus {};
    nav_gps_status gpsStatus {};
    ahrs.get_NavEKF3().getVariances(0,velVar, posVar, hgtVar, magVar, tasVar, offset);
    float tempVar = fmaxf(fmaxf(magVar.x,magVar.y),magVar.z);
    ahrs.get_NavEKF3().getFilterFaults(0,faultStatus);
    ahrs.get_NavEKF3().getFilterTimeouts(0,timeoutStatus);
    ahrs.get_NavEKF3().getFilterStatus(0,solutionStatus);
    ahrs.get_NavEKF3().getFilterGpsStatus(0,gpsStatus);
    float tiltError;
    ahrs.get_NavEKF3().getTiltError(0,tiltError);
    uint8_t primaryIndex = ahrs.get_NavEKF3().getPrimaryCoreIndex();
    struct log_NKF4 pkt4 = {
        LOG_PACKET_HEADER_INIT(LOG_XKF4_MSG),
        time_us : time_us,
        sqrtvarV : (int16_t)(100*velVar),
        sqrtvarP : (int16_t)(100*posVar),
        sqrtvarH : (int16_t)(100*hgtVar),
        sqrtvarM : (int16_t)(100*tempVar),
        sqrtvarVT : (int16_t)(100*tasVar),
        tiltErr : (float)tiltError,
        offsetNorth : (int8_t)(offset.x),
        offsetEast : (int8_t)(offset.y),
        faults : (uint16_t)(faultStatus),
        timeouts : (uint8_t)(timeoutStatus),
        solution : (uint16_t)(solutionStatus.value),
        gps : (uint16_t)(gpsStatus.value),
        primary : (int8_t)primaryIndex
    };
    WriteBlock(&pkt4, sizeof(pkt4));

    // Write fifth EKF packet - take data from the primary instance
    float normInnov=0; // normalised innovation variance ratio for optical flow observations fused by the main nav filter
    float gndOffset=0; // estimated vertical position of the terrain relative to the nav filter zero datum
    float flowInnovX=0, flowInnovY=0; // optical flow LOS rate vector innovations from the main nav filter
    float auxFlowInnov=0; // optical flow LOS rate innovation from terrain offset estimator
    float HAGL=0; // height above ground level
    float rngInnov=0; // range finder innovations
    float range=0; // measured range
    float gndOffsetErr=0; // filter ground offset state error
    Vector3f predictorErrors; // output predictor angle, velocity and position tracking error
    ahrs.get_NavEKF3().getFlowDebug(-1,normInnov, gndOffset, flowInnovX, flowInnovY, auxFlowInnov, HAGL, rngInnov, range, gndOffsetErr);
    ahrs.get_NavEKF3().getOutputTrackingError(-1,predictorErrors);
    struct log_NKF5 pkt5 = {
        LOG_PACKET_HEADER_INIT(LOG_XKF5_MSG),
        time_us : time_us,
        normInnov : (uint8_t)(MIN(100*normInnov,255)),
        FIX : (int16_t)(1000*flowInnovX),
        FIY : (int16_t)(1000*flowInnovY),
        AFI : (int16_t)(1000*auxFlowInnov),
        HAGL : (int16_t)(100*HAGL),
        offset : (int16_t)(100*gndOffset),
        RI : (int16_t)(100*rngInnov),
        meaRng : (uint16_t)(100*range),
        errHAGL : (uint16_t)(100*gndOffsetErr),
        angErr : (float)predictorErrors.x,
        velErr : (float)predictorErrors.y,
        posErr : (float)predictorErrors.z
     };
    WriteBlock(&pkt5, sizeof(pkt5));

    // log quaternion
    Quaternion quat;
    ahrs.get_NavEKF3().getQuaternion(0, quat);
    struct log_Quaternion pktq1 = {
        LOG_PACKET_HEADER_INIT(LOG_XKQ1_MSG),
        time_us : time_us,
        q1 : quat.q1,
        q2 : quat.q2,
        q3 : quat.q3,
        q4 : quat.q4
    };
    WriteBlock(&pktq1, sizeof(pktq1));
    
    // log innovations for the second IMU if enabled
    if (ahrs.get_NavEKF3().activeCores() >= 2) {
        // Write 6th EKF packet
        ahrs.get_NavEKF3().getEulerAngles(1,euler);
        ahrs.get_NavEKF3().getVelNED(1,velNED);
        ahrs.get_NavEKF3().getPosNE(1,posNE);
        ahrs.get_NavEKF3().getPosD(1,posD);
        ahrs.get_NavEKF3().getGyroBias(1,gyroBias);
        posDownDeriv = ahrs.get_NavEKF3().getPosDownDerivative(1);
        if (!ahrs.get_NavEKF3().getOriginLLH(1,originLLH)) {
            originLLH.alt = 0;
        }
        struct log_EKF1 pkt6 = {
            LOG_PACKET_HEADER_INIT(LOG_XKF6_MSG),
            time_us : time_us,
            roll    : (int16_t)(100*degrees(euler.x)), // roll angle (centi-deg, displayed as deg due to format string)
            pitch   : (int16_t)(100*degrees(euler.y)), // pitch angle (centi-deg, displayed as deg due to format string)
            yaw     : (uint16_t)wrap_360_cd(100*degrees(euler.z)), // yaw angle (centi-deg, displayed as deg due to format string)
            velN    : (float)(velNED.x), // velocity North (m/s)
            velE    : (float)(velNED.y), // velocity East (m/s)
            velD    : (float)(velNED.z), // velocity Down (m/s)
            posD_dot : (float)(posDownDeriv), // first derivative of down position
            posN    : (float)(posNE.x), // metres North
            posE    : (float)(posNE.y), // metres East
            posD    : (float)(posD), // metres Down
            gyrX    : (int16_t)(100*degrees(gyroBias.x)), // cd/sec, displayed as deg/sec due to format string
            gyrY    : (int16_t)(100*degrees(gyroBias.y)), // cd/sec, displayed as deg/sec due to format string
            gyrZ    : (int16_t)(100*degrees(gyroBias.z)), // cd/sec, displayed as deg/sec due to format string
            originHgt : originLLH.alt // WGS-84 altitude of EKF origin in cm
        };
        WriteBlock(&pkt6, sizeof(pkt6));

        // Write 7th EKF packet
        ahrs.get_NavEKF3().getAccelBias(1,accelBias);
        ahrs.get_NavEKF3().getWind(1,wind);
        ahrs.get_NavEKF3().getMagNED(1,magNED);
        ahrs.get_NavEKF3().getMagXYZ(1,magXYZ);
        magIndex = ahrs.get_NavEKF3().getActiveMag(1);
        struct log_NKF2a pkt7 = {
            LOG_PACKET_HEADER_INIT(LOG_XKF7_MSG),
            time_us : time_us,
            accBiasX  : (int16_t)(100*accelBias.x),
            accBiasY  : (int16_t)(100*accelBias.y),
            accBiasZ  : (int16_t)(100*accelBias.z),
            windN   : (int16_t)(100*wind.x),
            windE   : (int16_t)(100*wind.y),
            magN    : (int16_t)(magNED.x),
            magE    : (int16_t)(magNED.y),
            magD    : (int16_t)(magNED.z),
            magX    : (int16_t)(magXYZ.x),
            magY    : (int16_t)(magXYZ.y),
            magZ    : (int16_t)(magXYZ.z),
            index   : (uint8_t)(magIndex)
        };
        WriteBlock(&pkt7, sizeof(pkt7));

        // Write 8th EKF packet
        ahrs.get_NavEKF3().getInnovations(1,velInnov, posInnov, magInnov, tasInnov, yawInnov);
        struct log_NKF3 pkt8 = {
            LOG_PACKET_HEADER_INIT(LOG_XKF8_MSG),
            time_us : time_us,
            innovVN : (int16_t)(100*velInnov.x),
            innovVE : (int16_t)(100*velInnov.y),
            innovVD : (int16_t)(100*velInnov.z),
            innovPN : (int16_t)(100*posInnov.x),
            innovPE : (int16_t)(100*posInnov.y),
            innovPD : (int16_t)(100*posInnov.z),
            innovMX : (int16_t)(magInnov.x),
            innovMY : (int16_t)(magInnov.y),
            innovMZ : (int16_t)(magInnov.z),
            innovYaw : (int16_t)(100*degrees(yawInnov)),
            innovVT : (int16_t)(100*tasInnov)
        };
        WriteBlock(&pkt8, sizeof(pkt8));

        // Write 9th EKF packet
        ahrs.get_NavEKF3().getVariances(1,velVar, posVar, hgtVar, magVar, tasVar, offset);
        tempVar = fmaxf(fmaxf(magVar.x,magVar.y),magVar.z);
        ahrs.get_NavEKF3().getFilterFaults(1,faultStatus);
        ahrs.get_NavEKF3().getFilterTimeouts(1,timeoutStatus);
        ahrs.get_NavEKF3().getFilterStatus(1,solutionStatus);
        ahrs.get_NavEKF3().getFilterGpsStatus(1,gpsStatus);
        ahrs.get_NavEKF3().getTiltError(1,tiltError);
        struct log_NKF4 pkt9 = {
            LOG_PACKET_HEADER_INIT(LOG_XKF9_MSG),
            time_us : time_us,
            sqrtvarV : (int16_t)(100*velVar),
            sqrtvarP : (int16_t)(100*posVar),
            sqrtvarH : (int16_t)(100*hgtVar),
            sqrtvarM : (int16_t)(100*tempVar),
            sqrtvarVT : (int16_t)(100*tasVar),
            tiltErr : (float)tiltError,
            offsetNorth : (int8_t)(offset.x),
            offsetEast : (int8_t)(offset.y),
            faults : (uint16_t)(faultStatus),
            timeouts : (uint8_t)(timeoutStatus),
            solution : (uint16_t)(solutionStatus.value),
            gps : (uint16_t)(gpsStatus.value),
            primary : (int8_t)primaryIndex
        };
        WriteBlock(&pkt9, sizeof(pkt9));

        // log quaternion
        ahrs.get_NavEKF3().getQuaternion(1, quat);
        struct log_Quaternion pktq2 = {
            LOG_PACKET_HEADER_INIT(LOG_XKQ2_MSG),
            time_us : time_us,
            q1 : quat.q1,
            q2 : quat.q2,
            q3 : quat.q3,
            q4 : quat.q4
        };
        WriteBlock(&pktq2, sizeof(pktq2));        
    }
    // write range beacon fusion debug packet if the range value is non-zero
    uint8_t ID;
    float rng;
    float innovVar;
    float innov;
    float testRatio;
    Vector3f beaconPosNED;
    float bcnPosOffsetHigh;
    float bcnPosOffsetLow;
    Vector3f posNED;
     if (ahrs.get_NavEKF3().getRangeBeaconDebug(-1, ID, rng, innov, innovVar, testRatio, beaconPosNED, bcnPosOffsetHigh, bcnPosOffsetLow, posNED)) {
        if (rng > 0.0f) {
            struct log_RngBcnDebug pkt10 = {
                LOG_PACKET_HEADER_INIT(LOG_XKF10_MSG),
                time_us : time_us,
                ID : (uint8_t)ID,
                rng : (int16_t)(100*rng),
                innov : (int16_t)(100*innov),
                sqrtInnovVar : (uint16_t)(100*sqrtf(innovVar)),
                testRatio : (uint16_t)(100*constrain_float(testRatio,0.0f,650.0f)),
                beaconPosN : (int16_t)(100*beaconPosNED.x),
                beaconPosE : (int16_t)(100*beaconPosNED.y),
                beaconPosD : (int16_t)(100*beaconPosNED.z),
                offsetHigh : (int16_t)(100*bcnPosOffsetHigh),
                offsetLow : (int16_t)(100*bcnPosOffsetLow),
                posN : (int16_t)(100*posNED.x),
                posE : (int16_t)(100*posNED.y),
                posD : (int16_t)(100*posNED.z)

             };
            WriteBlock(&pkt10, sizeof(pkt10));
        }
    }
    // write debug data for body frame odometry fusion
    Vector3f velBodyInnov,velBodyInnovVar;
    static uint32_t lastUpdateTime_ms = 0;
    uint32_t updateTime_ms = ahrs.get_NavEKF3().getBodyFrameOdomDebug(-1, velBodyInnov, velBodyInnovVar);
    if (updateTime_ms > lastUpdateTime_ms) {
        struct log_ekfBodyOdomDebug pkt11 = {
            LOG_PACKET_HEADER_INIT(LOG_XKFD_MSG),
            time_us : time_us,
            velInnovX : velBodyInnov.x,
            velInnovY : velBodyInnov.y,
            velInnovZ : velBodyInnov.z,
            velInnovVarX : velBodyInnovVar.x,
            velInnovVarY : velBodyInnovVar.y,
            velInnovVarZ : velBodyInnovVar.z
         };
        WriteBlock(&pkt11, sizeof(pkt11));
        lastUpdateTime_ms = updateTime_ms;
    }

    // log state variances every 0.49s
    static uint32_t lastEkfStateVarLogTime_ms = 0;
    if (AP_HAL::millis() - lastEkfStateVarLogTime_ms > 490) {
        lastEkfStateVarLogTime_ms = AP_HAL::millis();
        float stateVar[24];
        ahrs.get_NavEKF3().getStateVariances(-1, stateVar);
        struct log_ekfStateVar pktv1 = {
            LOG_PACKET_HEADER_INIT(LOG_XKV1_MSG),
            time_us : time_us,
            v00 : stateVar[0],
            v01 : stateVar[1],
            v02 : stateVar[2],
            v03 : stateVar[3],
            v04 : stateVar[4],
            v05 : stateVar[5],
            v06 : stateVar[6],
            v07 : stateVar[7],
            v08 : stateVar[8],
            v09 : stateVar[9],
            v10 : stateVar[10],
            v11 : stateVar[11]
        };
        WriteBlock(&pktv1, sizeof(pktv1));
        struct log_ekfStateVar pktv2 = {
            LOG_PACKET_HEADER_INIT(LOG_XKV2_MSG),
            time_us : time_us,
            v00 : stateVar[12],
            v01 : stateVar[13],
            v02 : stateVar[14],
            v03 : stateVar[15],
            v04 : stateVar[16],
            v05 : stateVar[17],
            v06 : stateVar[18],
            v07 : stateVar[19],
            v08 : stateVar[20],
            v09 : stateVar[21],
            v10 : stateVar[22],
            v11 : stateVar[23]
        };
        WriteBlock(&pktv2, sizeof(pktv2));
    }


    // log EKF timing statistics every 5s
    static uint32_t lastTimingLogTime_ms = 0;
    if (AP_HAL::millis() - lastTimingLogTime_ms > 5000) {
        lastTimingLogTime_ms = AP_HAL::millis();
        struct ekf_timing timing;
        for (uint8_t i=0; i<ahrs.get_NavEKF3().activeCores(); i++) {
            ahrs.get_NavEKF3().getTimingStatistics(i, timing);
            Write_EKF_Timing(i==0?"XKT1":"XKT2", time_us, timing);
        }
    }
}
#endif

void AP_Logger::Write_Radio(const mavlink_radio_t &packet)
{
    struct log_Radio pkt = {
        LOG_PACKET_HEADER_INIT(LOG_RADIO_MSG),
        time_us      : AP_HAL::micros64(),
        rssi         : packet.rssi,
        remrssi      : packet.remrssi,
        txbuf        : packet.txbuf,
        noise        : packet.noise,
        remnoise     : packet.remnoise,
        rxerrors     : packet.rxerrors,
        fixed        : packet.fixed
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write a Camera packet
void AP_Logger::Write_CameraInfo(enum LogMessages msg, const AP_AHRS &ahrs, const Location &current_loc, uint64_t timestamp_us)
{
    int32_t altitude, altitude_rel, altitude_gps;
    if (current_loc.relative_alt) {
        altitude = current_loc.alt+ahrs.get_home().alt;
        altitude_rel = current_loc.alt;
    } else {
        altitude = current_loc.alt;
        altitude_rel = current_loc.alt - ahrs.get_home().alt;
    }
    const AP_GPS &gps = AP::gps();
    if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        altitude_gps = gps.location().alt;
    } else {
        altitude_gps = 0;
    }

    struct log_Camera pkt = {
        LOG_PACKET_HEADER_INIT(static_cast<uint8_t>(msg)),
        time_us     : timestamp_us?timestamp_us:AP_HAL::micros64(),
        gps_time    : gps.time_week_ms(),
        gps_week    : gps.time_week(),
        latitude    : current_loc.lat,
        longitude   : current_loc.lng,
        altitude    : altitude,
        altitude_rel: altitude_rel,
        altitude_gps: altitude_gps,
        roll        : (int16_t)ahrs.roll_sensor,
        pitch       : (int16_t)ahrs.pitch_sensor,
        yaw         : (uint16_t)ahrs.yaw_sensor
    };
    WriteCriticalBlock(&pkt, sizeof(pkt));
}

// Write a Camera packet
void AP_Logger::Write_Camera(const AP_AHRS &ahrs, const Location &current_loc, uint64_t timestamp_us)
{
    Write_CameraInfo(LOG_CAMERA_MSG, ahrs, current_loc, timestamp_us);
}

// Write a Trigger packet
void AP_Logger::Write_Trigger(const AP_AHRS &ahrs, const Location &current_loc)
{
    Write_CameraInfo(LOG_TRIGGER_MSG, ahrs, current_loc, 0);
}

// Write an attitude packet
void AP_Logger::Write_Attitude(AP_AHRS &ahrs, const Vector3f &targets)
{
    struct log_Attitude pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ATTITUDE_MSG),
        time_us         : AP_HAL::micros64(),
        control_roll    : (int16_t)targets.x,
        roll            : (int16_t)ahrs.roll_sensor,
        control_pitch   : (int16_t)targets.y,
        pitch           : (int16_t)ahrs.pitch_sensor,
        control_yaw     : (uint16_t)wrap_360_cd(targets.z),
        yaw             : (uint16_t)wrap_360_cd(ahrs.yaw_sensor),
        error_rp        : (uint16_t)(ahrs.get_error_rp() * 100),
        error_yaw       : (uint16_t)(ahrs.get_error_yaw() * 100)
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write an attitude packet
void AP_Logger::Write_AttitudeView(AP_AHRS_View &ahrs, const Vector3f &targets)
{
    struct log_Attitude pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ATTITUDE_MSG),
        time_us         : AP_HAL::micros64(),
        control_roll    : (int16_t)targets.x,
        roll            : (int16_t)ahrs.roll_sensor,
        control_pitch   : (int16_t)targets.y,
        pitch           : (int16_t)ahrs.pitch_sensor,
        control_yaw     : (uint16_t)wrap_360_cd(targets.z),
        yaw             : (uint16_t)wrap_360_cd(ahrs.yaw_sensor),
        error_rp        : (uint16_t)(ahrs.get_error_rp() * 100),
        error_yaw       : (uint16_t)(ahrs.get_error_yaw() * 100)
    };
    WriteBlock(&pkt, sizeof(pkt));
}

void AP_Logger::Write_Current_instance(const uint64_t time_us,
                                                 const uint8_t battery_instance,
                                                 const enum LogMessages type,
                                                 const enum LogMessages celltype)
{
    AP_BattMonitor &battery = AP::battery();
    float temp;
    bool has_temp = battery.get_temperature(temp, battery_instance);
    struct log_Current pkt = {
        LOG_PACKET_HEADER_INIT(type),
        time_us             : time_us,
        voltage             : battery.voltage(battery_instance),
        voltage_resting     : battery.voltage_resting_estimate(battery_instance),
        current_amps        : battery.current_amps(battery_instance),
        current_total       : battery.consumed_mah(battery_instance),
        consumed_wh         : battery.consumed_wh(battery_instance),
        temperature         : (int16_t)(has_temp ? (temp * 100) : 0),
        resistance          : battery.get_resistance(battery_instance)
    };
    WriteBlock(&pkt, sizeof(pkt));

    // individual cell voltages
    if (battery.has_cell_voltages(battery_instance)) {
        const AP_BattMonitor::cells &cells = battery.get_cell_voltages(battery_instance);
        struct log_Current_Cells cell_pkt = {
            LOG_PACKET_HEADER_INIT(celltype),
            time_us             : time_us,
            voltage             : battery.voltage(battery_instance)
        };
        for (uint8_t i = 0; i < ARRAY_SIZE(cells.cells); i++) {
            cell_pkt.cell_voltages[i] = cells.cells[i] + 1;
        }
        WriteBlock(&cell_pkt, sizeof(cell_pkt));

        // check battery structure can hold all cells
        static_assert(ARRAY_SIZE(cells.cells) == (sizeof(cell_pkt.cell_voltages) / sizeof(cell_pkt.cell_voltages[0])),
                      "Battery cell number doesn't match in library and log structure");
    }
}

// Write an Current data packet
void AP_Logger::Write_Current()
{
    // Big painful assert to ensure that logging won't produce suprising results when the
    // number of battery monitors changes, does have the built in expectation that
    // LOG_COMPASS_MSG follows the last LOG_CURRENT_CELLSx_MSG
    static_assert(((LOG_CURRENT_MSG + AP_BATT_MONITOR_MAX_INSTANCES) == LOG_CURRENT_CELLS_MSG) &&
                  ((LOG_CURRENT_CELLS_MSG + AP_BATT_MONITOR_MAX_INSTANCES) == LOG_COMPASS_MSG),
                  "The number of batt monitors has changed without updating the log "
                  "table entries. Please add new enums for LOG_CURRENT_MSG, LOG_CURRENT_CELLS_MSG "
                  "directly following the highest indexed fields. Don't forget to update the log "
                  "description table as well.");

    const uint64_t time_us = AP_HAL::micros64();
    const uint8_t num_instances = AP::battery().num_instances();
    for (uint8_t i = 0; i < num_instances; i++) {
        Write_Current_instance(time_us,
                                   i,
                                   (LogMessages)((uint8_t)LOG_CURRENT_MSG + i),
                                   (LogMessages)((uint8_t)LOG_CURRENT_CELLS_MSG + i));
    }
}

void AP_Logger::Write_Compass_instance(const uint64_t time_us, const uint8_t mag_instance, const enum LogMessages type)
{
    const Compass &compass = AP::compass();

    const Vector3f &mag_field = compass.get_field(mag_instance);
    const Vector3f &mag_offsets = compass.get_offsets(mag_instance);
    const Vector3f &mag_motor_offsets = compass.get_motor_offsets(mag_instance);
    struct log_Compass pkt = {
        LOG_PACKET_HEADER_INIT(type),
        time_us         : time_us,
        mag_x           : (int16_t)mag_field.x,
        mag_y           : (int16_t)mag_field.y,
        mag_z           : (int16_t)mag_field.z,
        offset_x        : (int16_t)mag_offsets.x,
        offset_y        : (int16_t)mag_offsets.y,
        offset_z        : (int16_t)mag_offsets.z,
        motor_offset_x  : (int16_t)mag_motor_offsets.x,
        motor_offset_y  : (int16_t)mag_motor_offsets.y,
        motor_offset_z  : (int16_t)mag_motor_offsets.z,
        health          : (uint8_t)compass.healthy(mag_instance),
        SUS             : compass.last_update_usec(mag_instance)
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write a Compass packet
void AP_Logger::Write_Compass(uint64_t time_us)
{
    if (time_us == 0) {
        time_us = AP_HAL::micros64();
    }
    const Compass &compass = AP::compass();
    if (compass.get_count() > 0) {
        Write_Compass_instance(time_us, 0, LOG_COMPASS_MSG);
    }

    if (compass.get_count() > 1) {
        Write_Compass_instance(time_us, 1, LOG_COMPASS2_MSG);
    }

    if (compass.get_count() > 2) {
        Write_Compass_instance(time_us, 2, LOG_COMPASS3_MSG);
    }
}


// Write an GPS packet
void AP_Logger::Write_GPS(uint8_t i, uint64_t time_us)
{
    const AP_GPS &gps = AP::gps();
    if (time_us == 0) {
        time_us = AP_HAL::micros64();
    }
    const struct Location &loc = gps.location(i);
    struct log_GPS pkt = {
        LOG_PACKET_HEADER_INIT((uint8_t)(LOG_GPS_MSG+i)),
        time_us       : time_us,
        status        : (uint8_t)gps.status(i),
        gps_week_ms   : gps.time_week_ms(i),
        gps_week      : gps.time_week(i),
        num_sats      : gps.num_sats(i),
        hdop          : gps.get_hdop(i),
        latitude      : loc.lat,
        longitude     : loc.lng,
        altitude      : loc.alt,
        ground_speed  : gps.ground_speed(i),
        ground_course : gps.ground_course(i),
        vel_z         : gps.velocity(i).z,
        used          : (uint8_t)(gps.primary_sensor() == i)
    };
    WriteBlock(&pkt, sizeof(pkt));

    /* write auxiliary accuracy information as well */
    float hacc = 0, vacc = 0, sacc = 0;
    gps.horizontal_accuracy(i, hacc);
    gps.vertical_accuracy(i, vacc);
    gps.speed_accuracy(i, sacc);
    struct log_GPA pkt2 = {
        LOG_PACKET_HEADER_INIT((uint8_t)(LOG_GPA_MSG+i)),
        time_us       : time_us,
        vdop          : gps.get_vdop(i),
        hacc          : (uint16_t)MIN((hacc*100), UINT16_MAX),
        vacc          : (uint16_t)MIN((vacc*100), UINT16_MAX),
        sacc          : (uint16_t)MIN((sacc*100), UINT16_MAX),
        have_vv       : (uint8_t)gps.have_vertical_velocity(i),
        sample_ms     : gps.last_message_time_ms(i),
        delta_ms      : gps.last_message_delta_time_ms(i)
    };
    WriteBlock(&pkt2, sizeof(pkt2));
}


// Write an RCIN packet
void AP_Logger::Write_RCIN(void)
{
    uint16_t values[14] = {};
    rc().get_radio_in(values, ARRAY_SIZE(values));
    struct log_RCIN pkt = {
        LOG_PACKET_HEADER_INIT(LOG_RCIN_MSG),
        time_us       : AP_HAL::micros64(),
        chan1         : values[0],
        chan2         : values[1],
        chan3         : values[2],
        chan4         : values[3],
        chan5         : values[4],
        chan6         : values[5],
        chan7         : values[6],
        chan8         : values[7],
        chan9         : values[8],
        chan10        : values[9],
        chan11        : values[10],
        chan12        : values[11],
        chan13        : values[12],
        chan14        : values[13]
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write an SERVO packet
void AP_Logger::Write_RCOUT(void)
{
    struct log_RCOUT pkt = {
        LOG_PACKET_HEADER_INIT(LOG_RCOUT_MSG),
        time_us       : AP_HAL::micros64(),
        chan1         : hal.rcout->read(0),
        chan2         : hal.rcout->read(1),
        chan3         : hal.rcout->read(2),
        chan4         : hal.rcout->read(3),
        chan5         : hal.rcout->read(4),
        chan6         : hal.rcout->read(5),
        chan7         : hal.rcout->read(6),
        chan8         : hal.rcout->read(7),
        chan9         : hal.rcout->read(8),
        chan10        : hal.rcout->read(9),
        chan11        : hal.rcout->read(10),
        chan12        : hal.rcout->read(11),
        chan13        : hal.rcout->read(12),
        chan14        : hal.rcout->read(13)
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write an RSSI packet
void AP_Logger::Write_RSSI()
{
    AP_RSSI *rssi = AP::rssi();
    if (rssi == nullptr) {
        return;
    }

    struct log_RSSI pkt = {
        LOG_PACKET_HEADER_INIT(LOG_RSSI_MSG),
        time_us       : AP_HAL::micros64(),
        RXRSSI        : rssi->read_receiver_rssi()
    };
    WriteBlock(&pkt, sizeof(pkt));
}

void AP_Logger::Write_Baro_instance(uint64_t time_us, uint8_t baro_instance, enum LogMessages type)
{
    AP_Baro &baro = AP::baro();
    float climbrate = baro.get_climb_rate();
    float drift_offset = baro.get_baro_drift_offset();
    float ground_temp = baro.get_ground_temperature();
    struct log_BARO pkt = {
        LOG_PACKET_HEADER_INIT(type),
        time_us       : time_us,
        altitude      : baro.get_altitude(baro_instance),
        pressure      : baro.get_pressure(baro_instance),
        temperature   : (int16_t)(baro.get_temperature(baro_instance) * 100 + 0.5f),
        climbrate     : climbrate,
        sample_time_ms: baro.get_last_update(baro_instance),
        drift_offset  : drift_offset,
        ground_temp   : ground_temp,
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write a BARO packet
void AP_Logger::Write_Baro(uint64_t time_us)
{
    if (time_us == 0) {
        time_us = AP_HAL::micros64();
    }
    const AP_Baro &baro = AP::baro();
    Write_Baro_instance(time_us, 0, LOG_BARO_MSG);
    if (baro.num_instances() > 1 && baro.healthy(1)) {
        Write_Baro_instance(time_us, 1, LOG_BAR2_MSG);
    }
    if (baro.num_instances() > 2 && baro.healthy(2)) {
        Write_Baro_instance(time_us, 2, LOG_BAR3_MSG);
    }
}

void AP_Logger::Write_IMU_instance(const uint64_t time_us, const uint8_t imu_instance, const enum LogMessages type)
{
    const AP_InertialSensor &ins = AP::ins();
    const Vector3f &gyro = ins.get_gyro(imu_instance);
    const Vector3f &accel = ins.get_accel(imu_instance);
    struct log_IMU pkt = {
        LOG_PACKET_HEADER_INIT(type),
        time_us : time_us,
        gyro_x  : gyro.x,
        gyro_y  : gyro.y,
        gyro_z  : gyro.z,
        accel_x : accel.x,
        accel_y : accel.y,
        accel_z : accel.z,
        gyro_error  : ins.get_gyro_error_count(imu_instance),
        accel_error : ins.get_accel_error_count(imu_instance),
        temperature : ins.get_temperature(imu_instance),
        gyro_health : (uint8_t)ins.get_gyro_health(imu_instance),
        accel_health : (uint8_t)ins.get_accel_health(imu_instance),
        gyro_rate : ins.get_gyro_rate_hz(imu_instance),
        accel_rate : ins.get_accel_rate_hz(imu_instance),
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write an raw accel/gyro data packet
void AP_Logger::Write_IMU()
{
    uint64_t time_us = AP_HAL::micros64();

    const AP_InertialSensor &ins = AP::ins();

    Write_IMU_instance(time_us, 0, LOG_IMU_MSG);
    if (ins.get_gyro_count() < 2 && ins.get_accel_count() < 2) {
        return;
    }

    Write_IMU_instance(time_us, 1, LOG_IMU2_MSG);

    if (ins.get_gyro_count() < 3 && ins.get_accel_count() < 3) {
        return;
    }

    Write_IMU_instance(time_us, 2, LOG_IMU3_MSG);
}

// Write an accel/gyro delta time data packet
void AP_Logger::Write_IMUDT_instance(const uint64_t time_us, const uint8_t imu_instance, const enum LogMessages type)
{
    const AP_InertialSensor &ins = AP::ins();
    float delta_t = ins.get_delta_time();
    float delta_vel_t = ins.get_delta_velocity_dt(imu_instance);
    float delta_ang_t = ins.get_delta_angle_dt(imu_instance);
    Vector3f delta_angle, delta_velocity;
    ins.get_delta_angle(imu_instance, delta_angle);
    ins.get_delta_velocity(imu_instance, delta_velocity);

    struct log_IMUDT pkt = {
        LOG_PACKET_HEADER_INIT(type),
        time_us : time_us,
        delta_time   : delta_t,
        delta_vel_dt : delta_vel_t,
        delta_ang_dt : delta_ang_t,
        delta_ang_x  : delta_angle.x,
        delta_ang_y  : delta_angle.y,
        delta_ang_z  : delta_angle.z,
        delta_vel_x  : delta_velocity.x,
        delta_vel_y  : delta_velocity.y,
        delta_vel_z  : delta_velocity.z
    };
    WriteBlock(&pkt, sizeof(pkt));
}

void AP_Logger::Write_IMUDT(uint64_t time_us, uint8_t imu_mask)
{
    const AP_InertialSensor &ins = AP::ins();
    if (imu_mask & 1) {
        Write_IMUDT_instance(time_us, 0, LOG_IMUDT_MSG);
    }
    if ((ins.get_gyro_count() < 2 && ins.get_accel_count() < 2) || !ins.use_gyro(1)) {
        return;
    }

    if (imu_mask & 2) {
        Write_IMUDT_instance(time_us, 1, LOG_IMUDT2_MSG);
    }

    if ((ins.get_gyro_count() < 3 && ins.get_accel_count() < 3) || !ins.use_gyro(2)) {
        return;
    }

    if (imu_mask & 4) {
        Write_IMUDT_instance(time_us, 2, LOG_IMUDT3_MSG);
    }
}

void AP_Logger::Write_Vibration()
{
    uint64_t time_us = AP_HAL::micros64();
    const AP_InertialSensor &ins = AP::ins();
    const Vector3f vibration = ins.get_vibration_levels();
    struct log_Vibe pkt = {
        LOG_PACKET_HEADER_INIT(LOG_VIBE_MSG),
        time_us     : time_us,
        vibe_x      : vibration.x,
        vibe_y      : vibration.y,
        vibe_z      : vibration.z,
        clipping_0  : ins.get_accel_clip_count(0),
        clipping_1  : ins.get_accel_clip_count(1),
        clipping_2  : ins.get_accel_clip_count(2)
    };
    WriteBlock(&pkt, sizeof(pkt));
}

namespace AP {

AP_Logger &logger()
{
    return *AP_Logger::get_singleton();
}

};
