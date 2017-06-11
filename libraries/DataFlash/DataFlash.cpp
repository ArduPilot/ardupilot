#include "DataFlash.h"

#include "DataFlash_Backend.h"

#include "DataFlash_File.h"
#include "DataFlash_MAVLink.h"

DataFlash_Class *DataFlash_Class::_instance;

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo DataFlash_Class::var_info[] = {
    // @Param: _BACKEND_TYPE
    // @DisplayName: DataFlash Backend Storage type
    // @Description: 0 for None, 1 for File, 2 for dataflash mavlink, 3 for both file and dataflash
    // @Values: 0:None,1:File,2:MAVLink,3:BothFileAndMAVLink
    // @User: Standard
    AP_GROUPINFO("_BACKEND_TYPE",  0, DataFlash_Class, _params.backend_types,       DATAFLASH_BACKEND_FILE),

    // @Param: _FILE_BUFSIZE
    // @DisplayName: Maximum DataFlash File Backend buffer size (in kilobytes)
    // @Description: The DataFlash_File backend uses a buffer to store data before writing to the block device.  Raising this value may reduce "gaps" in your SD card logging.  This buffer size may be reduced depending on available memory.  PixHawk requires at least 4 kilobytes.  Maximum value available here is 64 kilobytes.
    // @User: Standard
    AP_GROUPINFO("_FILE_BUFSIZE",  1, DataFlash_Class, _params.file_bufsize,       16),

    // @Param: _DISARMED
    // @DisplayName: Enable logging while disarmed
    // @Description: If LOG_DISARMED is set to 1 then logging will be enabled while disarmed. This can make for very large logfiles but can help a lot when tracking down startup issues
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("_DISARMED",  2, DataFlash_Class, _params.log_disarmed,       0),

    // @Param: _REPLAY
    // @DisplayName: Enable logging of information needed for Replay
    // @Description: If LOG_REPLAY is set to 1 then the EKF2 state estimator will log detailed information needed for diagnosing problems with the Kalman filter. It is suggested that you also raise LOG_FILE_BUFSIZE to give more buffer space for logging and use a high quality microSD card to ensure no sensor data is lost
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("_REPLAY",  3, DataFlash_Class, _params.log_replay,       0),

    // @Param: _FILE_DSRMROT
    // @DisplayName: Stop logging to current file on disarm
    // @Description: When set, the current log file is closed when the vehicle is disarmed.  If LOG_DISARMED is set then a fresh log will be opened.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("_FILE_DSRMROT",  4, DataFlash_Class, _params.file_disarm_rot,       0),

    AP_GROUPEND
};

void DataFlash_Class::Init(const struct LogStructure *structures, uint8_t num_types)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    validate_structures(structures, num_types);
    dump_structures(structures, num_types);
#endif
    if (_next_backend == DATAFLASH_MAX_BACKENDS) {
        AP_HAL::panic("Too many backends");
        return;
    }
    _num_types = num_types;
    _structures = structures;

#if defined(HAL_BOARD_LOG_DIRECTORY)
    if (_params.backend_types == DATAFLASH_BACKEND_FILE ||
        _params.backend_types == DATAFLASH_BACKEND_BOTH) {
        DFMessageWriter_DFLogStart *message_writer =
            new DFMessageWriter_DFLogStart(_firmware_string);
        if (message_writer != nullptr)  {
#if HAL_OS_POSIX_IO
            backends[_next_backend] = new DataFlash_File(*this,
                                                         message_writer,
                                                         HAL_BOARD_LOG_DIRECTORY);
#endif
        }
        if (backends[_next_backend] == nullptr) {
            hal.console->printf("Unable to open DataFlash_File");
        } else {
            _next_backend++;
        }
    }
#endif

#if DATAFLASH_MAVLINK_SUPPORT
    if (_params.backend_types == DATAFLASH_BACKEND_MAVLINK ||
        _params.backend_types == DATAFLASH_BACKEND_BOTH) {
        if (_next_backend == DATAFLASH_MAX_BACKENDS) {
            AP_HAL::panic("Too many backends");
            return;
        }
        DFMessageWriter_DFLogStart *message_writer =
            new DFMessageWriter_DFLogStart(_firmware_string);
        if (message_writer != nullptr)  {
            backends[_next_backend] = new DataFlash_MAVLink(*this,
                                                            message_writer);
        }
        if (backends[_next_backend] == nullptr) {
            hal.console->printf("Unable to open DataFlash_MAVLink");
        } else {
            _next_backend++;
        }
    }
#endif

    for (uint8_t i=0; i<_next_backend; i++) {
        backends[i]->Init();
    }
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

/// pretty-print field information from a log structure
void DataFlash_Class::dump_structure_field(const struct LogStructure *structure, const char *label, const uint8_t fieldnum)
{
    ::fprintf(stderr, "  %s\n", label);
}

/// pretty-print log structures
/// @note structures MUST be well-formed
void DataFlash_Class::dump_structures(const struct LogStructure *structures, const uint8_t num_types)
{
#if DEBUG_LOG_STRUCTURES
    for (uint16_t i=0; i<num_types; i++) {
        const struct LogStructure *structure = &structures[i];
        ::fprintf(stderr, "%s\n", structure->name);
        char label[32] = { };
        uint8_t labeloffset = 0;
        int8_t fieldnum = 0;
        for (uint8_t j=0; j<strlen(structure->labels); j++) {
            char labelchar = structure->labels[j];
            if (labelchar == '\0') {
                break;
            }
            if (labelchar == ',') {
                dump_structure_field(structure, label, fieldnum);
                fieldnum++;
                labeloffset = 0;
                memset(label, '\0', 32);
            } else {
                label[labeloffset++] = labelchar;
            }
        }
        dump_structure_field(structure, label, fieldnum);
        ::fprintf(stderr, "\n"); // just add a CR to the output
    }
#endif
}

void DataFlash_Class::validate_structures(const struct LogStructure *structures, const uint8_t num_types)
{
    Debug("Validating structures");
    bool passed = true;

    bool seen_ids[256] = { };
    for (uint16_t i=0; i<num_types; i++) {
        const struct LogStructure *logstructure = &structures[i];

#if DEBUG_LOG_STRUCTURES
        Debug("offset=%d ID=%d NAME=%s\n", i, logstructure->msg_type, logstructure->name);
#endif

        // names must be null-terminated
        if (logstructure->name[4] != '\0') {
            Debug("Message name not NULL-terminated");
            passed = false;
        }

        // ensure each message ID is only used once
        if (seen_ids[logstructure->msg_type]) {
            Debug("ID %d used twice (LogStructure offset=%d)", logstructure->msg_type, i);
            passed = false;
        }
        seen_ids[logstructure->msg_type] = true;

        // ensure we have enough labels to cover columns
        uint8_t fieldcount = strlen(logstructure->format);
        uint8_t labelcount = count_commas(logstructure->labels)+1;
        if (fieldcount != labelcount) {
            Debug("fieldcount=%u does not match labelcount=%u",
                  fieldcount, labelcount);
            passed = false;
        }

        // check that the structure is of an appropriate length to take fields
        const int16_t msg_len = Log_Write_calc_msg_len(logstructure->format);
        if (msg_len != logstructure->msg_len) {
            Debug("Calculated message length for (%s) based on format field (%s) does not match structure size (%d != %u)", logstructure->name, logstructure->format, msg_len, logstructure->msg_len);
            passed = false;
        }
    }
    if (!passed) {
        Debug("Log structures are invalid");
        abort();
    }
}

#else

void DataFlash_Class::dump_structure_field(const struct LogStructure *_structure, const char *label, const uint8_t fieldnum)
{
}

void DataFlash_Class::dump_structures(const struct LogStructure *structures, const uint8_t num_types)
{
}

void DataFlash_Class::validate_structures(const struct LogStructure *structures, const uint8_t num_types)
{
    return;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL

const struct LogStructure *DataFlash_Class::structure(uint16_t num) const
{
    return &_structures[num];
}

bool DataFlash_Class::logging_present() const
{
    return _next_backend != 0;
}
bool DataFlash_Class::logging_enabled() const
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
bool DataFlash_Class::logging_failed() const
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

void DataFlash_Class::Log_Write_MessageF(const char *fmt, ...)
{
    char msg[64] {};

    va_list ap;
    va_start(ap, fmt);
    hal.util->vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);

    Log_Write_Message(msg);
}

void DataFlash_Class::backend_starting_new_log(const DataFlash_Backend *backend)
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

// start any backend which hasn't started; this is only called from
// the vehicle code
void DataFlash_Class::StartUnstartedLogging(void)
{
    for (uint8_t i=0; i<_next_backend; i++) {
        if (!backends[i]->logging_started()) {
            backends[i]->start_new_log();
        }
    }
}

#define FOR_EACH_BACKEND(methodcall)              \
    do {                                          \
        for (uint8_t i=0; i<_next_backend; i++) { \
            backends[i]->methodcall;              \
        }                                         \
    } while (0)

void DataFlash_Class::setVehicle_Startup_Log_Writer(vehicle_startup_message_Log_Writer writer)
{
    _vehicle_messages = writer;
}

void DataFlash_Class::set_vehicle_armed(const bool armed_state)
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


void DataFlash_Class::set_mission(const AP_Mission *mission) {
    FOR_EACH_BACKEND(set_mission(mission));
}

// start functions pass straight through to backend:
void DataFlash_Class::WriteBlock(const void *pBuffer, uint16_t size) {
    FOR_EACH_BACKEND(WriteBlock(pBuffer, size));
}

void DataFlash_Class::WriteCriticalBlock(const void *pBuffer, uint16_t size) {
    FOR_EACH_BACKEND(WriteCriticalBlock(pBuffer, size));
}

void DataFlash_Class::WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical) {
    FOR_EACH_BACKEND(WritePrioritisedBlock(pBuffer, size, is_critical));
}

// change me to "DoTimeConsumingPreparations"?
void DataFlash_Class::EraseAll() {
    FOR_EACH_BACKEND(EraseAll());
}
// change me to "LoggingAvailable"?
bool DataFlash_Class::CardInserted(void) {
    for (uint8_t i=0; i< _next_backend; i++) {
        if (backends[i]->CardInserted()) {
            return true;
        }
    }
    return false;
}

bool DataFlash_Class::NeedPrep() {
    for (uint8_t i=0; i< _next_backend; i++) {
        if (backends[i]->NeedPrep()) {
            return true;
        }
    }
    return false;
}

void DataFlash_Class::Prep() {
    FOR_EACH_BACKEND(Prep());
}

void DataFlash_Class::StopLogging()
{
    FOR_EACH_BACKEND(stop_logging());
}

uint16_t DataFlash_Class::find_last_log() const {
    if (_next_backend == 0) {
        return 0;
    }
    return backends[0]->find_last_log();
}
void DataFlash_Class::get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page) {
    if (_next_backend == 0) {
        return;
    }
    backends[0]->get_log_boundaries(log_num, start_page, end_page);
}
void DataFlash_Class::get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc) {
    if (_next_backend == 0) {
        return;
    }
    backends[0]->get_log_info(log_num, size, time_utc);
}
int16_t DataFlash_Class::get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) {
    if (_next_backend == 0) {
        return 0;
    }
    return backends[0]->get_log_data(log_num, page, offset, len, data);
}
uint16_t DataFlash_Class::get_num_logs(void) {
    if (_next_backend == 0) {
        return 0;
    }
    return backends[0]->get_num_logs();
}

void DataFlash_Class::LogReadProcess(uint16_t log_num,
                                     uint16_t start_page, uint16_t end_page,
                                     print_mode_fn printMode,
                                     AP_HAL::BetterStream *port) {
    if (_next_backend == 0) {
        // how were we called?!
        return;
    }
    backends[0]->LogReadProcess(log_num, start_page, end_page, printMode, port);
}
void DataFlash_Class::DumpPageInfo(AP_HAL::BetterStream *port) {
    if (_next_backend == 0) {
        // how were we called?!
        return;
    }
    backends[0]->DumpPageInfo(port);
}
void DataFlash_Class::ShowDeviceInfo(AP_HAL::BetterStream *port) {
    if (_next_backend == 0) {
        // how were we called?!
        return;
    }
    backends[0]->ShowDeviceInfo(port);
}
void DataFlash_Class::ListAvailableLogs(AP_HAL::BetterStream *port) {
    if (_next_backend == 0) {
        // how were we called?!
        return;
    }
    backends[0]->ListAvailableLogs(port);
}

/* we're started if any of the backends are started */
bool DataFlash_Class::logging_started(void) {
    for (uint8_t i=0; i< _next_backend; i++) {
        if (backends[i]->logging_started()) {
            return true;
        }
    }
    return false;
}

void DataFlash_Class::EnableWrites(bool enable) {
    FOR_EACH_BACKEND(EnableWrites(enable));
}

// for DataFlash_MAVLink
void DataFlash_Class::remote_log_block_status_msg(mavlink_channel_t chan,
                                                  mavlink_message_t* msg) {
    FOR_EACH_BACKEND(remote_log_block_status_msg(chan, msg));
}
// end for DataFlash_MAVLink

void DataFlash_Class::periodic_tasks() {
     FOR_EACH_BACKEND(periodic_tasks());
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    // currently only DataFlash_File support this:
void DataFlash_Class::flush(void) {
     FOR_EACH_BACKEND(flush());
}
#endif


void DataFlash_Class::Log_Write_EntireMission(const AP_Mission &mission)
{
    FOR_EACH_BACKEND(Log_Write_EntireMission(mission));
}

void DataFlash_Class::Log_Write_Message(const char *message)
{
    FOR_EACH_BACKEND(Log_Write_Message(message));
}

void DataFlash_Class::Log_Write_Mode(uint8_t mode, uint8_t reason)
{
    FOR_EACH_BACKEND(Log_Write_Mode(mode, reason));
}

void DataFlash_Class::Log_Write_Parameter(const char *name, float value)
{
    FOR_EACH_BACKEND(Log_Write_Parameter(name, value));
}

void DataFlash_Class::Log_Write_Mission_Cmd(const AP_Mission &mission,
                                            const AP_Mission::Mission_Command &cmd)
{
    FOR_EACH_BACKEND(Log_Write_Mission_Cmd(mission, cmd));
}

uint32_t DataFlash_Class::num_dropped() const
{
    if (_next_backend == 0) {
        return 0;
    }
    return backends[0]->num_dropped();
}


// end functions pass straight through to backend

void DataFlash_Class::internal_error() const {
//    _internal_errors++;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    abort();
#endif
}

/* Log_Write support */
void DataFlash_Class::Log_Write(const char *name, const char *labels, const char *fmt, ...)
{
    va_list arg_list;
    
    struct log_write_fmt *f = msg_fmt_for_name(name, labels, fmt);
    if (f == nullptr) {
        // unable to map name to a messagetype; could be out of
        // msgtypes, could be out of slots, ...
        internal_error();
        return;
    }

    for (uint8_t i=0; i<_next_backend; i++) {
        if (!(f->sent_mask & (1U<<i))) {
            if (!backends[i]->Log_Write_Emit_FMT(f->msg_type)) {
                continue;
            }
            f->sent_mask |= (1U<<i);
        }
        va_start(arg_list, fmt);
        backends[i]->Log_Write(f->msg_type, arg_list);
        va_end(arg_list);
    }
}


DataFlash_Class::log_write_fmt *DataFlash_Class::msg_fmt_for_name(const char *name, const char *labels, const char *fmt)
{
    struct log_write_fmt *f;
    for (f = log_write_fmts; f; f=f->next) {
        if (f->name == name) { // ptr comparison
            // already have an ID for this name:
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

    int16_t tmp = Log_Write_calc_msg_len(fmt);
    if (tmp == -1) {
        free(f);
        return nullptr;
    }

    f->msg_len = tmp;

    // add to front of list
    f->next = log_write_fmts;
    log_write_fmts = f;

    return f;
}

// returns true if the msg_type is already taken
bool DataFlash_Class::msg_type_in_use(const uint8_t msg_type) const
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
int16_t DataFlash_Class::find_free_msg_type() const
{
    // avoid using 255 here; perhaps we want to use it to extend things later
    for (uint16_t msg_type=254; msg_type>0; msg_type--) { // more likely to be free at end
        if (! msg_type_in_use(msg_type)) {
            return msg_type;
        }
    }
    return -1;
}

bool DataFlash_Class::fill_log_write_logstructure(struct LogStructure &logstruct, const uint8_t msg_type) const
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
    strncpy((char*)logstruct.name, f->name, sizeof(logstruct.name)); /* cast away the "const" (*gulp*) */
    strncpy((char*)logstruct.format, f->fmt, sizeof(logstruct.format));
    strncpy((char*)logstruct.labels, f->labels, sizeof(logstruct.labels));
    logstruct.msg_len = f->msg_len;
    return true;
}

/* calculate the length of output of a format string.  Note that this
 * returns an int16_t; if it returns -1 then an error has occurred.
 * This was mechanically converted from init_field_types in
 * Tools/Replay/MsgHandler.cpp */
int16_t DataFlash_Class::Log_Write_calc_msg_len(const char *fmt) const
{
    uint8_t len =  LOG_PACKET_HEADER_LEN;
    for (uint8_t i=0; i<strlen(fmt); i++) {
        switch(fmt[i]) {
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
        default: return -1;
        }
    }
    return len;
}

/* End of Log_Write support */

#undef FOR_EACH_BACKEND
