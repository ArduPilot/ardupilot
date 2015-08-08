#ifndef DATAFLASH_BACKEND_H
#define DATAFLASH_BACKEND_H

#if HAL_CPU_CLASS < HAL_CPU_CLASS_75
#define DATAFLASH_NO_CLI
#endif

#include "DataFlash.h"

class DataFlash_Backend
{
public:
    FUNCTOR_TYPEDEF(print_mode_fn, void, AP_HAL::BetterStream*, uint8_t);

    DataFlash_Backend(DataFlash_Class &front) :
        _front(front),
        _writing_startup_messages(false),
        _internal_errors(0),
        _dropped(0),
        _last_periodic_1Hz(0),
        _last_periodic_10Hz(0)
        { }

    void internal_error();

    virtual bool CardInserted(void) = 0;

    // erase handling
    virtual void EraseAll() = 0;

    virtual bool NeedPrep() = 0;
    virtual void Prep() = 0;

    /* Write a block of data at current offset */
    bool WriteBlock(const void *pBuffer, uint16_t size) {
        return WritePrioritisedBlock(pBuffer, size, false);
    }

    bool WriteCriticalBlock(const void *pBuffer, uint16_t size) {
        return WritePrioritisedBlock(pBuffer, size, true);
    }

    virtual bool WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical) = 0;

    // high level interface
    virtual uint16_t find_last_log(void) = 0;
    virtual void get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page) = 0;
    virtual void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc) = 0;
    virtual int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) = 0;
    virtual uint16_t get_num_logs(void) = 0;
#ifndef DATAFLASH_NO_CLI
    virtual void LogReadProcess(uint16_t log_num,
                                uint16_t start_page, uint16_t end_page,
                                print_mode_fn printMode,
                                AP_HAL::BetterStream *port) = 0;
    virtual void DumpPageInfo(AP_HAL::BetterStream *port) = 0;
    virtual void ShowDeviceInfo(AP_HAL::BetterStream *port) = 0;
    virtual void ListAvailableLogs(AP_HAL::BetterStream *port) = 0;
#endif // DATAFLASH_NO_CLI

    void EnableWrites(bool enable) { _writes_enabled = enable; }
    bool logging_started(void) const { return log_write_started; }

    // initialisation this really shouldn't take structure and
    // num_types, however the CLI LogReadProcess function requires it.
    // That function needs to be split.
    virtual void Init(const struct LogStructure *structure, uint8_t num_types) {
        _writes_enabled = true;
        _num_types = num_types;
        _structures = structure;
    }

    virtual uint16_t bufferspace_available() = 0;

    virtual uint16_t start_new_log(void) = 0;
    bool log_write_started;

    void Log_Fill_Format(const struct LogStructure *structure, struct log_Format &pkt);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    // currently only DataFlash_File support this:
    virtual void flush(void) { }
#endif

    virtual void periodic_tasks();

    virtual void WroteStartupFormat() { }
    virtual void WroteStartupParam() { }

protected:
    DataFlash_Class &_front;

    virtual void periodic_10Hz(const uint32_t now);
    virtual void periodic_1Hz(const uint32_t now);
    virtual void periodic_fullrate(const uint32_t now);

    /*
    read and print a log entry using the format strings from the given structure
    */
    void _print_log_entry(uint8_t msg_type,
                          print_mode_fn print_mode,
                          AP_HAL::BetterStream *port);

    const struct LogStructure *_structures;
    uint8_t _num_types = 0;
    bool _writes_enabled = false;

    /*
      read a block
    */
    virtual bool ReadBlock(void *pkt, uint16_t size) = 0;

    virtual bool WriteBlockCheckStartupMessages();
    virtual void WriteMoreStartupMessages();
    virtual void push_log_blocks();
    bool _writing_startup_messages = false;

    uint32_t _internal_errors;
    uint32_t _dropped;
private:
    uint32_t _last_periodic_1Hz;
    uint32_t _last_periodic_10Hz;
};

#endif
