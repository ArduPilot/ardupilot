#include <DataFlash.h>
#include "DataFlash_Backend.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo DataFlash_Class::var_info[] PROGMEM = {
    // @Param: _BACKEND_TYPES
    // @DisplayName: DataFlash Backend Storage type
    // @Description: 0 for None, 1 for File, 2 for dataflash mavlink, 3 for both file and dataflash
    // @Values: 0:None,1:File,2:MAVLink,3:BothFileAndMAVLink
    // @User: Standard
    AP_GROUPINFO("_BACKEND_TYPE",  0, DataFlash_Class, _params.backend_types,       DATAFLASH_BACKEND_FILE),
    AP_GROUPEND
};


#define FOR_EACH_BACKEND(methodcall)              \
    do {                                          \
        for (uint8_t i=0; i<_next_backend; i++) { \
            backends[i]->methodcall;              \
        }                                         \
    } while (0)
    

void DataFlash_Class::WriteBlock(const void *pBuffer, uint16_t size) {
    FOR_EACH_BACKEND(WriteBlock(pBuffer, size));
}

// change me to "DoTimeConsumingPreparations"?
void DataFlash_Class::EraseAll() {
    FOR_EACH_BACKEND(EraseAll());
}
void DataFlash_Class::Prep_MinSpace() {
    FOR_EACH_BACKEND(Prep_MinSpace());
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
// remove me in favour of calling "DoTimeConsumingPreparations" all the time?
bool DataFlash_Class::NeedErase(void) {
    for (uint8_t i=0; i< _next_backend; i++) {
        if (backends[i]->NeedErase()) {
            return true;
        }
    }
    return false;
}

// log retrieval stuff always operates on the first backend for now:
uint16_t DataFlash_Class::find_last_log(void) {
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
    return backends[0]->get_num_logs();
}
void DataFlash_Class::Log_Fill_Format(const struct LogStructure *s, struct log_Format &pkt) {
    if (_next_backend == 0) {
        // how were we called?!
        return;
    }
    backends[0]->Log_Fill_Format(s, pkt);
}

#ifndef DATAFLASH_NO_CLI
void DataFlash_Class::LogReadProcess(uint16_t log_num,
                                     uint16_t start_page, uint16_t end_page,
                                     void (*print_mode)(AP_HAL::BetterStream *port, uint8_t mode),
                                     AP_HAL::BetterStream *port) {
    if (_next_backend == 0) {
        // how were we called?!
        return;
    }
    backends[0]->LogReadProcess(log_num, start_page, end_page, print_mode, port);
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
#endif // DATAFLASH_NO_CLI

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
void DataFlash_Class::periodic_tasks() {
    FOR_EACH_BACKEND(periodic_tasks());
}
// end for DataFlash_MAVLink

#include "DataFlash_MAVLink.h"
    
/*
 * write statistics about a DF MAV object to DataFlash
 */
void DataFlash_Class::Log_Write_Format(const struct LogStructure *s)
{
    FOR_EACH_BACKEND(Log_Write_Format(s));
}

void DataFlash_Class::Log_Write_Message(const char *message)
{
    FOR_EACH_BACKEND(Log_Write_Message(message));
}

void DataFlash_Class::Log_Write_Message_P(const prog_char_t *message)
{
    FOR_EACH_BACKEND(Log_Write_Message_P(message));
}

void DataFlash_Class::Log_Write_Mode(uint8_t mode)
{
    FOR_EACH_BACKEND(Log_Write_Mode(mode));
}

void DataFlash_Class::Log_Write_Parameter(const char *name, float value)
{
    FOR_EACH_BACKEND(Log_Write_Parameter(name, value));
}

// end functions pass straight through to backend

#undef FOR_EACH_BACKEND
