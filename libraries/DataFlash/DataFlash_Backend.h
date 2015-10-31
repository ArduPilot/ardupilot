#ifndef DATAFLASH_BACKEND_H
#define DATAFLASH_BACKEND_H

#if HAL_CPU_CLASS < HAL_CPU_CLASS_75
#define DATAFLASH_NO_CLI
#endif

#include <AP_HAL.h>
#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_GPS.h>
#include <AP_InertialSensor.h>
#include <AP_Baro.h>
#include <AP_AHRS.h>
#include <AP_Vehicle.h>
#include "../AP_Airspeed/AP_Airspeed.h"
#include "../AP_BattMonitor/AP_BattMonitor.h"
#include <stdint.h>
#include <GCS_MAVLink.h> // for mavlink_msg_t

#include "DFMessageWriter.h"
#include "LogStructure.h"

class DataFlash_MAVLink;

class DataFlash_Backend
{
    friend class DFMessageWriter_DFLogStart; // for access to _num_types etc
    friend class DFMessageWriter; // for access to _num_types etc

public:

    DataFlash_Backend(const struct LogStructure *structure, uint8_t num_types,
                      class DFMessageWriter *writer);

    void internal_error();
    virtual bool CardInserted(void) = 0;

    // erase handling
    virtual bool NeedErase(void) = 0;
    virtual void EraseAll() = 0;
    virtual void Prep_MinSpace(void) { }

    /* Write a block of data at current offset */
    virtual bool WriteBlock(const void *pBuffer, uint16_t size) = 0;

    // high level interface
    virtual uint16_t find_last_log(void) = 0;
    virtual void get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page) = 0;
    virtual void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc) = 0;
    virtual int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) = 0;
    virtual uint16_t get_num_logs(void) = 0;
#ifndef DATAFLASH_NO_CLI
    virtual void LogReadProcess(uint16_t log_num,
                                uint16_t start_page, uint16_t end_page,
                                void (*print_mode)(AP_HAL::BetterStream *port, uint8_t mode),
                                AP_HAL::BetterStream *port) = 0;
    virtual void DumpPageInfo(AP_HAL::BetterStream *port) = 0;
    virtual void ShowDeviceInfo(AP_HAL::BetterStream *port) = 0;
    virtual void ListAvailableLogs(AP_HAL::BetterStream *port) = 0;
#endif // DATAFLASH_NO_CLI

    void EnableWrites(bool enable) { _writes_enabled = enable; }
    bool logging_started(void) const { return _logging_started; }

    // initialisation this really shouldn't take structure and
    // num_types, however the CLI LogReadProcess function requires it.
    // That function needs to be split.
    virtual void Init(const struct LogStructure *structure, uint8_t num_types) {
        _writes_enabled = true;
        _num_types = num_types;
        _structures = structure;
    }

    void WriteMorePrefaceMessages();
    virtual uint16_t bufferspace_available() = 0;
    virtual void push_log_blocks(void) = 0;
    
    virtual uint16_t start_new_log(void) = 0;
    bool _logging_started;

    // FIXME: move me to a notional LogStructure.cpp?
    void Log_Fill_Format(const struct LogStructure *structure, struct log_Format &pkt);

    // for Dataflash_MAVlink
    virtual void remote_log_block_status_msg(mavlink_channel_t chan,
                                             mavlink_message_t* msg) { }
    // end for Dataflash_MAVlink

    virtual void periodic_tasks();

    void Log_Write_DF_MAV(DataFlash_MAVLink &df);
    bool Log_Write_Format(const struct LogStructure *structure);
    bool Log_Write_Message(const char *message);
    bool Log_Write_Message_P(const prog_char_t *message);
    bool Log_Write_Mode(uint8_t mode);;
    bool Log_Write_Parameter(const char *name, float value);
    bool Log_Write_Parameter(const AP_Param *ap,
                             const AP_Param::ParamToken &token,
                             enum ap_var_type type);

protected:
    uint32_t dropped;
    uint8_t internal_errors; // uint8_t - wishful thinking?

    virtual void periodic_10Hz(const uint32_t now);
    virtual void periodic_1Hz(const uint32_t now);
    virtual void periodic_fullrate(const uint32_t now);

    /*
    read and print a log entry using the format strings from the given structure
    */
    void _print_log_entry(uint8_t msg_type,
                          void (*print_mode)(AP_HAL::BetterStream *port, uint8_t mode),
                          AP_HAL::BetterStream *port);

    virtual bool WriteBlockCheckPrefaceMessages();

    const struct LogStructure *_structures;
    uint8_t _num_types;
    bool _writes_enabled;
    bool _writing_preface_messages;

    /*
      read a block
    */
    virtual void ReadBlock(void *pkt, uint16_t size) = 0;

    DFMessageWriter *_startup_messagewriter;

private:
    
    uint32_t _last_periodic_1Hz;
    uint32_t _last_periodic_10Hz;
};

#endif
