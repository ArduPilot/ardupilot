#include "AP_Logger.h"

#if HAL_LOGGING_ENABLED

#include "AP_Logger_Backend.h"

#include "AP_Logger_File.h"
#include "AP_Logger_DataFlash.h"
#include "AP_Logger_W25N01GV.h"
#include "AP_Logger_MAVLink.h"

#include <AP_InternalError/AP_InternalError.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Rally/AP_Rally.h>

AP_Logger *AP_Logger::_singleton;

extern const AP_HAL::HAL& hal;

#ifndef HAL_LOGGING_FILE_BUFSIZE
#if HAL_MEM_CLASS >= HAL_MEM_CLASS_1000
#define HAL_LOGGING_FILE_BUFSIZE  200
#elif HAL_MEM_CLASS >= HAL_MEM_CLASS_500
#define HAL_LOGGING_FILE_BUFSIZE  80
#elif HAL_MEM_CLASS >= HAL_MEM_CLASS_300
#define HAL_LOGGING_FILE_BUFSIZE  50
#else
#define HAL_LOGGING_FILE_BUFSIZE  16
#endif
#endif

#ifndef HAL_LOGGING_DATAFLASH_DRIVER
#define HAL_LOGGING_DATAFLASH_DRIVER AP_Logger_DataFlash
#endif

#ifndef HAL_LOGGING_STACK_SIZE
#define HAL_LOGGING_STACK_SIZE 1580
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
# if HAL_LOGGING_FILESYSTEM_ENABLED && (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#  define HAL_LOGGING_BACKENDS_DEFAULT Backend_Type::FILESYSTEM
# elif HAL_LOGGING_DATAFLASH_ENABLED
#  define HAL_LOGGING_BACKENDS_DEFAULT Backend_Type::BLOCK
# elif HAL_LOGGING_FILESYSTEM_ENABLED
#  define HAL_LOGGING_BACKENDS_DEFAULT Backend_Type::FILESYSTEM
# elif HAL_LOGGING_MAVLINK_ENABLED
#  define HAL_LOGGING_BACKENDS_DEFAULT Backend_Type::MAVLINK
# else
#  define HAL_LOGGING_BACKENDS_DEFAULT 0
# endif
#endif

// when adding new msgs we start at a different index in replay
#if APM_BUILD_TYPE(APM_BUILD_Replay)
#define LOGGING_FIRST_DYNAMIC_MSGID REPLAY_LOG_NEW_MSG_MAX
#else
#define LOGGING_FIRST_DYNAMIC_MSGID 254
#endif


const AP_Param::GroupInfo AP_Logger::var_info[] = {
    // @Param: _BACKEND_TYPE
    // @DisplayName: AP_Logger Backend Storage type
    // @Description: Bitmap of what Logger backend types to enable. Block-based logging is available on SITL and boards with dataflash chips. Multiple backends can be selected.
    // @Bitmask: 0:File,1:MAVLink,2:Block
    // @User: Standard
    AP_GROUPINFO("_BACKEND_TYPE",  0, AP_Logger, _params.backend_types,       uint8_t(HAL_LOGGING_BACKENDS_DEFAULT)),

    // @Param: _FILE_BUFSIZE
    // @DisplayName: Maximum AP_Logger File and Block Backend buffer size (in kilobytes)
    // @Description: The File and Block backends use a buffer to store data before writing to the block device.  Raising this value may reduce "gaps" in your SD card logging.  This buffer size may be reduced depending on available memory.  PixHawk requires at least 4 kilobytes.  Maximum value available here is 64 kilobytes.
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
    // @Description: When set, the current log file is closed when the vehicle is disarmed.  If LOG_DISARMED is set then a fresh log will be opened. Applies to the File and Block logging backends.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("_FILE_DSRMROT",  4, AP_Logger, _params.file_disarm_rot,       0),

#if HAL_LOGGING_MAVLINK_ENABLED
    // @Param: _MAV_BUFSIZE
    // @DisplayName: Maximum AP_Logger MAVLink Backend buffer size
    // @Description: Maximum amount of memory to allocate to AP_Logger-over-mavlink
    // @User: Advanced
    // @Units: kB
    AP_GROUPINFO("_MAV_BUFSIZE",  5, AP_Logger, _params.mav_bufsize,       HAL_LOGGING_MAV_BUFSIZE),
#endif

    // @Param: _FILE_TIMEOUT
    // @DisplayName: Timeout before giving up on file writes
    // @Description: This controls the amount of time before failing writes to a log file cause the file to be closed and logging stopped.
    // @User: Standard
    // @Units: s
    AP_GROUPINFO("_FILE_TIMEOUT",  6, AP_Logger, _params.file_timeout,     HAL_LOGGING_FILE_TIMEOUT),

    // @Param: _FILE_MB_FREE
    // @DisplayName: Old logs on the SD card will be deleted to maintain this amount of free space
    // @Description: Set this such that the free space is larger than your largest typical flight log
    // @Units: MB
    // @Range: 10 1000
    // @User: Standard
    AP_GROUPINFO("_FILE_MB_FREE",  7, AP_Logger, _params.min_MB_free, 500),

    // @Param: _FILE_RATEMAX
    // @DisplayName: Maximum logging rate for file backend
    // @Description: This sets the maximum rate that streaming log messages will be logged to the file backend. A value of zero means that rate limiting is disabled.
    // @Units: Hz
    // @Range: 0 1000
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("_FILE_RATEMAX",  8, AP_Logger, _params.file_ratemax, 0),

#if HAL_LOGGING_MAVLINK_ENABLED
    // @Param: _MAV_RATEMAX
    // @DisplayName: Maximum logging rate for mavlink backend
    // @Description: This sets the maximum rate that streaming log messages will be logged to the mavlink backend. A value of zero means that rate limiting is disabled.
    // @Units: Hz
    // @Range: 0 1000
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("_MAV_RATEMAX",  9, AP_Logger, _params.mav_ratemax, 0),
#endif

#if HAL_LOGGING_BLOCK_ENABLED
    // @Param: _BLK_RATEMAX
    // @DisplayName: Maximum logging rate for block backend
    // @Description: This sets the maximum rate that streaming log messages will be logged to the mavlink backend. A value of zero means that rate limiting is disabled.
    // @Units: Hz
    // @Range: 0 1000
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("_BLK_RATEMAX", 10, AP_Logger, _params.blk_ratemax, 0),
#endif
    
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
    // convert from 8 bit to 16 bit LOG_FILE_BUFSIZE
    _params.file_bufsize.convert_parameter_width(AP_PARAM_INT8);

    if (hal.util->was_watchdog_armed()) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Forcing logging for watchdog reset");
        _params.log_disarmed.set(1);
    }
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    validate_structures(structures, num_types);
    dump_structures(structures, num_types);
#endif
    _num_types = num_types;
    _structures = structures;

    // the "main" logging type needs to come before mavlink so that
    // index 0 is correct
    static const struct {
        Backend_Type type;
        AP_Logger_Backend* (*probe_fn)(AP_Logger&, LoggerMessageWriter_DFLogStart*);
    } backend_configs[] {
#if HAL_LOGGING_FILESYSTEM_ENABLED
        { Backend_Type::FILESYSTEM, AP_Logger_File::probe },
#endif
#if HAL_LOGGING_DATAFLASH_ENABLED
        { Backend_Type::BLOCK, HAL_LOGGING_DATAFLASH_DRIVER::probe },
#endif
#if HAL_LOGGING_MAVLINK_ENABLED
        { Backend_Type::MAVLINK, AP_Logger_MAVLink::probe },
#endif
};

    for (const auto &backend_config : backend_configs) {
        if ((_params.backend_types & uint8_t(backend_config.type)) == 0) {
            continue;
        }
        if (_next_backend == LOGGER_MAX_BACKENDS) {
            AP_BoardConfig::config_error("Too many backends");
            return;
        }
        LoggerMessageWriter_DFLogStart *message_writer =
            new LoggerMessageWriter_DFLogStart();
        if (message_writer == nullptr)  {
            AP_BoardConfig::allocation_error("message writer");
        }
        backends[_next_backend] = backend_config.probe_fn(*this, message_writer);
        if (backends[_next_backend] == nullptr) {
            AP_BoardConfig::allocation_error("logger backend");
        }
        _next_backend++;
    }

    for (uint8_t i=0; i<_next_backend; i++) {
        backends[i]->Init();
    }

    start_io_thread();

    EnableWrites(true);
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
    for (uint8_t i=0; i<unit_id; i++) {
        if (_units[i].ID == unit_id) {
            return _units[i].unit;
        }
    }
    return nullptr;
}

/// return a multiplier value given its ID
double AP_Logger::multiplier_name(const uint8_t multiplier_id)
{
    for (uint8_t i=0; i<multiplier_id; i++) {
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

bool AP_Logger::labels_string_is_good(const char *labels) const
{
    bool passed = true;
    if (strlen(labels) >= LS_LABELS_SIZE) {
        Debug("Labels string too long (%u > %u)", unsigned(strlen(labels)), unsigned(LS_LABELS_SIZE));
        passed = false;
    }
    // This goes through and slices labels up into substrings by
    // changing commas to nulls - keeping references to each string in
    // label_offsets.
    char *label_offsets[LS_LABELS_SIZE];
    uint8_t label_offsets_offset = 0;
    char labels_copy[LS_LABELS_SIZE+1] {};
    strncpy(labels_copy, labels, LS_LABELS_SIZE);
    if (labels_copy[0] == ',') {
        Debug("Leading comma in (%s)", labels);
        passed = false;
    }
    label_offsets[label_offsets_offset++] = labels_copy;
    const uint8_t len = strnlen(labels_copy, LS_LABELS_SIZE);
    for (uint8_t i=0; i<len; i++) {
        if (labels_copy[i] == ',') {
            if (labels_copy[i+1] == '\0') {
                Debug("Trailing comma in (%s)", labels);
                passed = false;
                continue;
            }
            labels_copy[i] = '\0';
            label_offsets[label_offsets_offset++] = &labels_copy[i+1];
        }
    }
    for (uint8_t i=0; i<label_offsets_offset-1; i++) {
        for (uint8_t j=i+1; j<label_offsets_offset; j++) {
            if (!strcmp(label_offsets[i], label_offsets[j])) {
                Debug("Duplicate label (%s) in (%s)", label_offsets[i], labels);
                passed = false;
            }
        }
    }
    return passed;
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
            Debug("  Message %s." fieldname_s " not NULL-terminated or too long", logstructure->name); \
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
        Debug("  %s fieldcount=%u does not match labelcount=%u",
              logstructure->name, fieldcount, labelcount);
        passed = false;
    }

    if (!labels_string_is_good(logstructure->labels)) {
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
        Debug("  %s fieldcount=%u does not match unitcount=%u",
              logstructure->name, (unsigned)fieldcount, (unsigned)strlen(logstructure->units));
        passed = false;
    }

    // ensure we have multipliers for each field
    if (strlen(logstructure->multipliers) != fieldcount) {
        Debug("  %s fieldcount=%u does not match multipliercount=%u",
              logstructure->name, (unsigned)fieldcount, (unsigned)strlen(logstructure->multipliers));
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
        AP_BoardConfig::config_error("See console: Log structures invalid");
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
    _log_start_count++;

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

/*
  return true if in log download which should prevent logging
 */
bool AP_Logger::in_log_download() const
{
    if (uint8_t(_params.backend_types) & uint8_t(Backend_Type::BLOCK)) {
        // when we have a BLOCK backend then listing completely prevents logging
        return transfer_activity != TransferActivity::IDLE;
    }
    // for other backends listing does not interfere with logging
    return transfer_activity == TransferActivity::SENDING;
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

    if (_armed) {
         // went from disarmed to armed
#if HAL_LOGGER_FILE_CONTENTS_ENABLED
        // get a set of @SYS files logged:
        file_content_prepare_for_arming = true;
#endif
    } else {
        // went from armed to disarmed
        FOR_EACH_BACKEND(vehicle_was_disarmed());
    }
}

#if APM_BUILD_TYPE(APM_BUILD_Replay)
/*
  remember formats for replay. This allows WriteV() to work within
  replay
*/
void AP_Logger::save_format_Replay(const void *pBuffer)
{
    if (((uint8_t *)pBuffer)[2] == LOG_FORMAT_MSG) {
        struct log_Format *fmt = (struct log_Format *)pBuffer;
        struct log_write_fmt *f = new log_write_fmt;
        f->msg_type = fmt->type;
        f->msg_len = fmt->length;
        f->name = strndup(fmt->name, sizeof(fmt->name));
        f->fmt = strndup(fmt->format, sizeof(fmt->format));
        f->labels = strndup(fmt->labels, sizeof(fmt->labels));
        f->next = log_write_fmts;
        log_write_fmts = f;
    }
}
#endif


// start functions pass straight through to backend:
void AP_Logger::WriteBlock(const void *pBuffer, uint16_t size) {
#if APM_BUILD_TYPE(APM_BUILD_Replay)
    save_format_Replay(pBuffer);
#endif
    FOR_EACH_BACKEND(WriteBlock(pBuffer, size));
}

// only the first backend write need succeed for us to be successful
bool AP_Logger::WriteBlock_first_succeed(const void *pBuffer, uint16_t size) 
{
    if (_next_backend == 0) {
        return false;
    }
    
    for (uint8_t i=1; i<_next_backend; i++) {
        backends[i]->WriteBlock(pBuffer, size);
    }

    return backends[0]->WriteBlock(pBuffer, size);
}

// write a replay block. This differs from other as it returns false if a backend doesn't
// have space for the msg
bool AP_Logger::WriteReplayBlock(uint8_t msg_id, const void *pBuffer, uint16_t size) {
    bool ret = true;
    if (log_replay()) {
        uint8_t buf[3+size];
        buf[0] = HEAD_BYTE1;
        buf[1] = HEAD_BYTE2;
        buf[2] = msg_id;
        memcpy(&buf[3], pBuffer, size);
        for (uint8_t i=0; i<_next_backend; i++) {
            if (!backends[i]->WritePrioritisedBlock(buf, sizeof(buf), true)) {
                ret = false;
            }
        }
    }
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // things will almost certainly go sour.  However, if we are not
    // logging while disarmed then the EKF can be started and trying
    // to log things even 'though the backends might be saying "no".
    if (!ret && log_while_disarmed()) {
        AP_HAL::panic("Failed to log replay block");
    }
#endif
    return ret;
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
        FOR_EACH_BACKEND(remote_log_block_status_msg(link, msg));
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
#ifndef HAL_BUILD_AP_PERIPH
    handle_log_send();
#endif
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

void AP_Logger::Write_Mode(uint8_t mode, const ModeReason reason)
{
    FOR_EACH_BACKEND(Write_Mode(mode, reason));
}

void AP_Logger::Write_Parameter(const char *name, float value)
{
    FOR_EACH_BACKEND(Write_Parameter(name, value, quiet_nanf()));
}

void AP_Logger::Write_Mission_Cmd(const AP_Mission &mission,
                                            const AP_Mission::Mission_Command &cmd)
{
    FOR_EACH_BACKEND(Write_Mission_Cmd(mission, cmd));
}

#if HAL_RALLY_ENABLED
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
#endif

#if HAL_LOGGER_FENCE_ENABLED
void AP_Logger::Write_Fence()
{
    FOR_EACH_BACKEND(Write_Fence());
}
#endif

// output a FMT message for each backend if not already done so
void AP_Logger::Safe_Write_Emit_FMT(log_write_fmt *f)
{
    for (uint8_t i=0; i<_next_backend; i++) {
        if (!(f->sent_mask & (1U<<i))) {
            if (!backends[i]->Write_Emit_FMT(f->msg_type)) {
                continue;
            }
            f->sent_mask |= (1U<<i);
        }
    }
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

void AP_Logger::WriteStreaming(const char *name, const char *labels, const char *fmt, ...)
{
    va_list arg_list;

    va_start(arg_list, fmt);
    WriteV(name, labels, nullptr, nullptr, fmt, arg_list, false, true);
    va_end(arg_list);
}

void AP_Logger::WriteStreaming(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, ...)
{
    va_list arg_list;

    va_start(arg_list, fmt);
    WriteV(name, labels, units, mults, fmt, arg_list, false, true);
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

void AP_Logger::WriteV(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, va_list arg_list,
                       bool is_critical, bool is_streaming)
{
    // WriteV is not safe in replay as we can re-use IDs
    const bool direct_comp = APM_BUILD_TYPE(APM_BUILD_Replay);
    struct log_write_fmt *f = msg_fmt_for_name(name, labels, units, mults, fmt, direct_comp);
    if (f == nullptr) {
        // unable to map name to a messagetype; could be out of
        // msgtypes, could be out of slots, ...
#if !APM_BUILD_TYPE(APM_BUILD_Replay)
        INTERNAL_ERROR(AP_InternalError::error_t::logger_mapfailure);
#endif
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
        backends[i]->Write(f->msg_type, arg_copy, is_critical, is_streaming);
        va_end(arg_copy);
    }
}

/*
  when we are doing replay logging we want to delay start of the EKF
  until after the headers are out so that on replay all parameter
  values are available
 */
bool AP_Logger::allow_start_ekf() const
{
    if (!log_replay() || !log_while_disarmed()) {
        return true;
    }

    for (uint8_t i=0; i<_next_backend; i++) {
        if (!backends[i]->allow_start_ekf()) {
            return false;
        }
    }

    return true;
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
bool AP_Logger::assert_same_fmt_for_name(const AP_Logger::log_write_fmt *f,
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
    if (!streq(f->fmt, fmt)) {
        Debug("format fmt differ (%s) vs (%s)",
              (f->fmt ? f->fmt : "nullptr"),
              (fmt ? fmt : "nullptr"));
        passed = false;
    }
#if !APM_BUILD_TYPE(APM_BUILD_Replay)
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
    if (!passed) {
        AP_BoardConfig::config_error("See console: Format definition must be consistent for every call of Write");
    }
#endif
    return passed;
}
#endif

AP_Logger::log_write_fmt *AP_Logger::msg_fmt_for_name(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, const bool direct_comp)
{
    WITH_SEMAPHORE(log_write_fmts_sem);
    struct log_write_fmt *f;
    for (f = log_write_fmts; f; f=f->next) {
        if (!direct_comp) {
            if (f->name == name) { // ptr comparison
                // already have an ID for this name:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                if (!assert_same_fmt_for_name(f, name, labels, units, mults, fmt)) {
                    return nullptr;
                }
#endif
                return f;
            }
        } else {
            // direct comparison used from scripting where pointer is not maintained
            if (strcmp(f->name,name) == 0) {
                // already have an ID for this name:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                if (!assert_same_fmt_for_name(f, name, labels, units, mults, fmt)) {
                    return nullptr;
                }
#endif
                return f;
            }
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
        AP_BoardConfig::config_error("See console: Log structure invalid");
    }
#endif

    return f;
}

const struct LogStructure *AP_Logger::structure_for_msg_type(const uint8_t msg_type) const
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
    // check static list of messages (e.g. from LOG_COMMON_STRUCTURES)
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
    const uint8_t start = LOGGING_FIRST_DYNAMIC_MSGID;
    // avoid using 255 here; perhaps we want to use it to extend things later
    for (uint16_t msg_type=start; msg_type>0; msg_type--) { // more likely to be free at end
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
        if (f->msg_type == msg_type) {
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

/*
  see if we need to save a crash dump. Returns true if either no crash
  dump available or we have saved it to sdcard. This is called
  continuously until success to account for late mount of the microSD
 */
bool AP_Logger::check_crash_dump_save(void)
{
    int fd = AP::FS().open("@SYS/crash_dump.bin", O_RDONLY);
    if (fd == -1) {
        // we don't have a crash dump file. The @SYS filesystem
        // returns -1 for open on empty files
        return true;
    }
    int fd2 = AP::FS().open("APM/crash_dump.bin", O_WRONLY|O_CREAT|O_TRUNC);
    if (fd2 == -1) {
        // sdcard not available yet, try again later
        AP::FS().close(fd);
        return false;
    }
    uint8_t buf[128];
    int32_t n;
    while ((n = AP::FS().read(fd, buf, sizeof(buf))) > 0) {
        AP::FS().write(fd2, buf, n);
    }
    AP::FS().close(fd2);
    AP::FS().close(fd);
    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Saved crash_dump.bin");
    return true;
}

// thread for processing IO - in general IO involves a long blocking DMA write to an SPI device
// and the thread will sleep while this completes preventing other tasks from running, it therefore
// is necessary to run the IO in it's own thread
void AP_Logger::io_thread(void)
{
    uint32_t last_run_us = AP_HAL::micros();
    uint32_t last_stack_us = last_run_us;
    uint32_t last_crash_check_us = last_run_us;
    bool done_crash_dump_save = false;

    while (true) {
        uint32_t now = AP_HAL::micros();

        uint32_t delay = 250U; // always have some delay
        if (now - last_run_us < 1000) {
            delay = MAX(1000 - (now - last_run_us), delay);
        }
        hal.scheduler->delay_microseconds(delay);

        last_run_us = AP_HAL::micros();

        FOR_EACH_BACKEND(io_timer());

        if (now - last_stack_us > 100000U) {
            last_stack_us = now;
            hal.util->log_stack_info();
        }

        // check for saving a crash dump file every 5s
        if (!done_crash_dump_save &&
            now - last_crash_check_us > 5000000U) {
            last_crash_check_us = now;
            done_crash_dump_save = check_crash_dump_save();
        }
#if HAL_LOGGER_FILE_CONTENTS_ENABLED
        file_content_update();
#endif
    }
}

// start the update thread
void AP_Logger::start_io_thread(void)
{
    WITH_SEMAPHORE(_log_send_sem);

    if (_io_thread_started) {
        return;
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Logger::io_thread, void), "log_io", HAL_LOGGING_STACK_SIZE, AP_HAL::Scheduler::PRIORITY_IO, 1)) {
        AP_HAL::panic("Failed to start Logger IO thread");
    }

    _io_thread_started = true;
    return;
}

/* End of Write support */

#undef FOR_EACH_BACKEND

// Wrote an event packet
void AP_Logger::Write_Event(LogEvent id)
{
    const struct log_Event pkt{
        LOG_PACKET_HEADER_INIT(LOG_EVENT_MSG),
        time_us  : AP_HAL::micros64(),
        id       : (uint8_t)id
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
    if (_force_long_log_persist) {
        // log for 10x longer than default
        persist_ms *= 10U;
    }

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

#if HAL_LOGGER_FILE_CONTENTS_ENABLED
void AP_Logger::prepare_at_arming_sys_file_logging()
{
    // free existing content:
    at_arm_file_content.reset();

    /*
      log files useful for diagnostics on arming. We log on arming as
      with LOG_DISARMED we don't want to log the statistics at boot or
      we wouldn't get a realistic idea of key system values
      Note that some of these files may not exist, in that case they
      are ignored
     */
    static const char *log_content_filenames[] = {
        "@SYS/uarts.txt",
        "@SYS/dma.txt",
        "@SYS/memory.txt",
        "@SYS/threads.txt",
        "@SYS/timers.txt",
        "@ROMFS/hwdef.dat",
        "@SYS/storage.bin",
        "@SYS/crash_dump.bin",
    };
    for (const auto *name : log_content_filenames) {
        log_file_content(at_arm_file_content, name);
    }
}

void AP_Logger::FileContent::reset()
{
    WITH_SEMAPHORE(sem);
    file_list *next = nullptr;
    for (auto *c = head; c != nullptr; c = next) {
        next = c->next;
        delete [] c->filename;
        delete c;
    }
    head = nullptr;
    tail = nullptr;
    if (fd != -1) {
        AP::FS().close(fd);
        fd = -1;
    }
    counter = 0;
    fast = false;
    offset = 0;
}

// removes victim from FileContent ***and delete()s it***
void AP_Logger::FileContent::remove_and_free(file_list *victim)
{
    WITH_SEMAPHORE(sem);

    file_list *prev = nullptr;
    for (auto *c = head; c != nullptr; prev = c, c = c->next) {
        if (c != victim) {
            continue;
        }

        // found the item to remove; remove it and return
        if (prev == nullptr) {
            head = victim->next;
        } else {
            prev->next = victim->next;
        }
        delete [] victim->filename;
        delete victim;
        return;
    }
}


/*
  log the content of a file in FILE log messages
 */
void AP_Logger::log_file_content(const char *filename)
{
    log_file_content(normal_file_content, filename);
}

void AP_Logger::log_file_content(FileContent &file_content, const char *filename)
{
    WITH_SEMAPHORE(file_content.sem);
    auto *file = new file_list;
    if (file == nullptr) {
        return;
    }
    // make copy to allow original to go out of scope
    const size_t len = strlen(filename)+1;
    char * tmp_filename = new char[len];
    if (tmp_filename == nullptr) {
        delete file;
        return;
    }
    strncpy(tmp_filename, filename, len);
    file->filename = tmp_filename;
    // Remove directory if whole file name will not fit
    const char * name = strrchr(file->filename, '/');
    if ((len-1 > sizeof(file->log_filename)) && (name != nullptr)) {
        strncpy_noterm(file->log_filename, name+1, sizeof(file->log_filename));
    } else {
        strncpy_noterm(file->log_filename, file->filename, sizeof(file->log_filename));
    }
    if (file_content.head == nullptr) {
        file_content.tail = file_content.head = file;
        file_content.fd = -1;
    } else {
        file_content.tail->next = file;
        file_content.tail = file;
    }
}

/*
  periodic call to log file content
 */
void AP_Logger::file_content_update(void)
{
    if (file_content_prepare_for_arming) {
        file_content_prepare_for_arming = false;
        prepare_at_arming_sys_file_logging();
    }

    file_content_update(at_arm_file_content);
    file_content_update(normal_file_content);
}

void AP_Logger::file_content_update(FileContent &file_content)
{
    auto *file = file_content.head;
    if (file == nullptr) {
        return;
    }

    /* this function is called at around 100Hz on average (tested on
       400Hz copter). We don't want to saturate the logging with file
       data, so we reduce the frequency of 64 byte file writes by a
       factor of 10. For the file crash_dump.bin we dump 10x faster so
       we get it in a reasonable time (full dump of 450k in about 1
       minute)
    */
    file_content.counter++;
    const uint8_t frequency = file_content.fast?1:10;
    if (file_content.counter % frequency != 0) {
        return;
    }

    if (file_content.fd == -1) {
        // open a new file
        file_content.fd  = AP::FS().open(file->filename, O_RDONLY);
        file_content.fast = strncmp(file->filename, "@SYS/crash_dump", 15) == 0;
        if (file_content.fd == -1) {
            file_content.remove_and_free(file);
            return;
        }
        file_content.offset = 0;
        if (file_content.fast) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Logging %s", file->filename);
        }
    }

    struct log_File pkt {
        LOG_PACKET_HEADER_INIT(LOG_FILE_MSG),
    };
    memcpy(pkt.filename, file->log_filename, sizeof(pkt.filename));
    const auto length = AP::FS().read(file_content.fd, pkt.data, sizeof(pkt.data));
    if (length <= 0) {
        AP::FS().close(file_content.fd);
        file_content.fd = -1;
        file_content.remove_and_free(file);
        return;
    }
    pkt.offset = file_content.offset;
    pkt.length = length;
    if (WriteBlock_first_succeed(&pkt, sizeof(pkt))) {
        file_content.offset += length;
    } else {
        // seek back ready for another try
        AP::FS().lseek(file_content.fd, file_content.offset, SEEK_SET);
    }
}
#endif // HAL_LOGGER_FILE_CONTENTS_ENABLED

namespace AP {

AP_Logger &logger()
{
    return *AP_Logger::get_singleton();
}

};

#endif // HAL_LOGGING_ENABLED
