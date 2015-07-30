#include <DataFlash.h>


const AP_Param::GroupInfo DataFlash_Class::var_info[] PROGMEM = {
    // @Param: _TYPE
    // @DisplayName: Log storage type
    // @Description: 8 byte value indicating backend to use (where applicable)
    // @Values: 0:Default,1:MAVLink
    // @User: Standard
    AP_GROUPINFO("_TYPE", 0, DataFlash_Class, params.type, BACKEND_TYPE_DEFAULT_VALUE),
};

// start functions pass straight through to backend:
void DataFlash_Class::WriteBlock(const void *pBuffer, uint16_t size) {
    backend->WriteBlock(pBuffer, size);
}
void DataFlash_Class::WriteCriticalBlock(const void *pBuffer, uint16_t size) {
    backend->WriteCriticalBlock(pBuffer, size);
}
uint16_t DataFlash_Class::start_new_log() {
    return backend->start_new_log();
}

// change me to "DoTimeConsumingPreparations"?
void DataFlash_Class::EraseAll() {
    backend->EraseAll();
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
                                     print_mode_fn printMode,
                                     AP_HAL::BetterStream *port) {
    backend->LogReadProcess(log_num, start_page, end_page, printMode, port);
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

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    // currently only DataFlash_File support this:
void DataFlash_Class::flush(void) {
    backend->flush();
}
#endif

// for DataFlash_MAVLink
void DataFlash_Class::handle_ack(mavlink_channel_t chan, uint32_t block_num) {
    backend->handle_ack(chan, block_num);
}
void DataFlash_Class::handle_retry(uint32_t block_num) {
    backend->handle_retry(block_num);
}
void DataFlash_Class::remote_log_block_status_msg(mavlink_channel_t chan,
                                                  mavlink_message_t* msg) {
    backend->remote_log_block_status_msg(chan, msg);
}
// end for DataFlash_MAVLink


// end functions pass straight through to backend
