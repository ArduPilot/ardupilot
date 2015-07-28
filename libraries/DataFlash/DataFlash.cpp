#include <DataFlash.h>
#include "DataFlash_Backend.h"

extern const AP_HAL::HAL& hal;

// start functions pass straight through to backend:
bool DataFlash_Class::WriteBlock(const void *pBuffer, uint16_t size) {
    return backend->WriteBlock(pBuffer, size);
}
uint16_t DataFlash_Class::start_new_log() {
    return backend->start_new_log();
}

// change me to "DoTimeConsumingPreparations"?
void DataFlash_Class::EraseAll() {
    backend->EraseAll();
}
void DataFlash_Class::Prep_MinSpace() {
    backend->Prep_MinSpace();
}
// change me to "LoggingAvailable"?
bool DataFlash_Class::CardInserted(void) {
    return backend->CardInserted();
}
// remove me in favour of calling "DoTimeConsumingPreparations" all the time?
bool DataFlash_Class::NeedErase(void) {
    return backend->NeedErase();
}

uint16_t DataFlash_Class::find_last_log(void) {
    return backend->find_last_log();
}
void DataFlash_Class::get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page) {
    backend->get_log_boundaries(log_num, start_page, end_page);
}
void DataFlash_Class::get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc) {
    backend->get_log_info(log_num, size, time_utc);
}
int16_t DataFlash_Class::get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) {
    return backend->get_log_data(log_num, page, offset, len, data);
}
uint16_t DataFlash_Class::get_num_logs(void) {
    return backend->get_num_logs();
}
void DataFlash_Class::Log_Fill_Format(const struct LogStructure *s, struct log_Format &pkt) {
    backend->Log_Fill_Format(s, pkt);
}

#ifndef DATAFLASH_NO_CLI
void DataFlash_Class::LogReadProcess(uint16_t log_num,
                                     uint16_t start_page, uint16_t end_page,
                                     void (*print_mode)(AP_HAL::BetterStream *port, uint8_t mode),
                                     AP_HAL::BetterStream *port) {
    backend->LogReadProcess(log_num, start_page, end_page, print_mode, port);
}
void DataFlash_Class::DumpPageInfo(AP_HAL::BetterStream *port) {
    backend->DumpPageInfo(port);
}
void DataFlash_Class::ShowDeviceInfo(AP_HAL::BetterStream *port) {
    backend->ShowDeviceInfo(port);
}
void DataFlash_Class::ListAvailableLogs(AP_HAL::BetterStream *port) {
    backend->ListAvailableLogs(port);
}
#endif // DATAFLASH_NO_CLI

bool DataFlash_Class::logging_started(void) {
    return backend->logging_started();
}

void DataFlash_Class::EnableWrites(bool enable) {
    backend->EnableWrites(enable);
}

// for DataFlash_MAVLink
void DataFlash_Class::remote_log_block_status_msg(mavlink_message_t* msg) {
    backend->remote_log_block_status_msg(msg);
}
void DataFlash_Class::periodic_tasks() {
    backend->periodic_tasks();
}
// end for DataFlash_MAVLink

#include "DataFlash_MAVLink.h"
/*
 * write statistics about a DF MAV object to DataFlash
 */
void DataFlash_Class::Log_Write_DF_MAV(DataFlash_MAVLink &df)
{
    if (df.stats.collection_count == 0) {
        return;
    }
    struct log_DF_MAV_Stats pkt = {
        LOG_PACKET_HEADER_INIT(LOG_DF_MAV_STATS),
        timestamp         : hal.scheduler->millis(),
        dropped           : df.stats.dropped,
        internal_errors   : df.stats.internal_errors,
        state_free_avg    : (uint8_t)(df.stats.state_free/df.stats.collection_count),
        state_free_min    : df.stats.state_free_min,
        state_free_max    : df.stats.state_free_max,
        state_pending_avg : (uint8_t)(df.stats.state_pending/df.stats.collection_count),
        state_pending_min : df.stats.state_pending_min,
        state_pending_max : df.stats.state_pending_max,
        state_sent_avg    : (uint8_t)(df.stats.state_sent/df.stats.collection_count),
        state_sent_min    : df.stats.state_sent_min,
        state_sent_max    : df.stats.state_sent_max,
        state_retry_avg   : (uint8_t)(df.stats.state_retry/df.stats.collection_count),
        state_retry_min    : df.stats.state_retry_min,
        state_retry_max    : df.stats.state_retry_max
    };
    WriteBlock(&pkt,sizeof(pkt));
}

// end functions pass straight through to backend
