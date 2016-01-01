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

void DataFlash_Class::Log_Write_Mode(uint8_t mode)
{
    FOR_EACH_BACKEND(Log_Write_Mode(mode));
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


// end functions pass straight through to backend

#undef FOR_EACH_BACKEND
