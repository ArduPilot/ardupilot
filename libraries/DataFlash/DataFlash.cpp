#include "DataFlash.h"

#include "DataFlash_Backend.h"

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

    AP_GROUPEND
};

const struct LogStructure *DataFlash_Class::structure(uint16_t num) const
{
    return &_structures[num];
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

    int16_t msg_type = msg_type_for_name(name, labels, fmt);
    if (msg_type == -1) {
        // unable to map name to a messagetype; could be out of
        // msgtypes, could be out of slots, ...
        internal_error();
        return;
    }

    for (uint8_t i=0; i<_next_backend; i++) {
        va_start(arg_list, fmt);
        if (backends[i]->Log_Write_Emit_FMT(msg_type)) {
            backends[i]->Log_Write(msg_type, arg_list);
        } else {
            // backend failed to write format (and has never written it)
        }
        va_end(arg_list);
    }
}


int16_t DataFlash_Class::msg_type_for_name(const char *name, const char *labels, const char *fmt)
{
    for (uint8_t i=0; i<_log_write_fmt_count;i++) {
        if (log_write_fmts[i].name == name) { // ptr comparison
            // already have an ID for this name:
            return log_write_fmts[i].msg_type;
        }
    }
    if (_log_write_fmt_count >= ARRAY_SIZE(log_write_fmts)) {
        // out of slots to remember formats
        return -1;
    }
    // no message type allocated for this name.  Try to allocate one:
    int16_t msg_type = find_free_msg_type();
    if (msg_type == -1) {
        return -1;
    }
    log_write_fmts[_log_write_fmt_count].msg_type = msg_type;
    log_write_fmts[_log_write_fmt_count].name = name;
    log_write_fmts[_log_write_fmt_count].fmt = fmt;
    log_write_fmts[_log_write_fmt_count].labels = labels;
    int16_t tmp = Log_Write_calc_msg_len(fmt);
    if (tmp == -1) {
        return -1;
    }
    log_write_fmts[_log_write_fmt_count].msg_len = tmp;
    _log_write_fmt_count++;

    return msg_type;
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
    // check the write messages we've already put out:
    for (uint8_t i=0; i<_log_write_fmt_count; i++) {
        if (log_write_fmts[i].msg_type == msg_type) {
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
    uint8_t i;
    for (i=0; i<_log_write_fmt_count; i++) {
        if(log_write_fmts[i].msg_type == msg_type) {
            break;
        }
    }
    if (i >= _log_write_fmt_count) {
        // asked to fill for a type we don't recognise
        internal_error();
        return false;
    }
    logstruct.msg_type = msg_type;
    strncpy((char*)logstruct.name, log_write_fmts[i].name, sizeof(logstruct.name)); /* cast away the "const" (*gulp*) */
    strncpy((char*)logstruct.format, log_write_fmts[i].fmt, sizeof(logstruct.format));
    strncpy((char*)logstruct.labels, log_write_fmts[i].labels, sizeof(logstruct.labels));
    logstruct.msg_len = log_write_fmts[i].msg_len;
    return true;
}

/* calculate the length of output of a format string.  Note that this
 * returns an int16_t; if it returns -1 then an error has occured.
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
