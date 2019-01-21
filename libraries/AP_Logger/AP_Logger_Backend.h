#pragma once

#include "AP_Logger.h"

class LoggerMessageWriter_DFLogStart;

class AP_Logger_Backend
{

public:
    FUNCTOR_TYPEDEF(vehicle_startup_message_Writer, void);

    AP_Logger_Backend(AP_Logger &front,
                      class LoggerMessageWriter_DFLogStart *writer);

    vehicle_startup_message_Writer vehicle_message_writer();

    void internal_error();

    virtual bool CardInserted(void) const = 0;

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

    bool WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical);

    // high level interface
    virtual uint16_t find_last_log() = 0;
    virtual void get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page) = 0;
    virtual void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc) = 0;
    virtual int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) = 0;
    virtual uint16_t get_num_logs() = 0;

    virtual bool logging_started(void) const = 0;

    virtual void Init() { }

    virtual uint32_t bufferspace_available() = 0;

    virtual void PrepForArming() { }

    virtual uint16_t start_new_log(void) = 0;

    /* stop logging - close output files etc etc.
     *
     * note that this doesn't stop logging from starting up again
     * immediately - e.g. AP_Logger_MAVLink might get another start
     * packet from a client.
     */
    virtual void stop_logging(void) = 0;

    void Fill_Format(const struct LogStructure *structure, struct log_Format &pkt);
    void Fill_Format_Units(const struct LogStructure *s, struct log_Format_Units &pkt);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    // currently only AP_Logger_File support this:
    virtual void flush(void) { }
#endif

     // for Dataflash_MAVlink
    virtual void remote_log_block_status_msg(mavlink_channel_t chan,
                                             mavlink_message_t* msg) { }
    // end for Dataflash_MAVlink

   virtual void periodic_tasks();

    uint8_t num_types() const;
    const struct LogStructure *structure(uint8_t structure) const;

    uint8_t num_units() const;
    const struct UnitStructure *unit(uint8_t unit) const;

    uint8_t num_multipliers() const;
    const struct MultiplierStructure *multiplier(uint8_t multiplier) const;

    void Write_EntireMission(const AP_Mission &mission);
    bool Write_Format(const struct LogStructure *structure);
    bool Write_Message(const char *message);
    bool Write_MessageF(const char *fmt, ...);
    bool Write_Mission_Cmd(const AP_Mission &mission,
                               const AP_Mission::Mission_Command &cmd);
    bool Write_Mode(uint8_t mode, uint8_t reason = 0);
    bool Write_Parameter(const char *name, float value);
    bool Write_Parameter(const AP_Param *ap,
                             const AP_Param::ParamToken &token,
                             enum ap_var_type type);

    uint32_t num_dropped(void) const {
        return _dropped;
    }

    /*
     * Write support
     */
    // write a FMT message out (if it hasn't been done already).
    // Returns true if the FMT message has ever been written.
    bool Write_Emit_FMT(uint8_t msg_type);

    // write a log message out to the log of msg_type type, with
    // values contained in arg_list:
    bool Write(uint8_t msg_type, va_list arg_list, bool is_critical=false);

    // these methods are used when reporting system status over mavlink
    virtual bool logging_enabled() const = 0;
    virtual bool logging_failed() const = 0;

    virtual void vehicle_was_disarmed() { };

    bool Write_Unit(const struct UnitStructure *s);
    bool Write_Multiplier(const struct MultiplierStructure *s);
    bool Write_Format_Units(const struct LogStructure *structure);


protected:

    AP_Logger &_front;

    virtual void periodic_10Hz(const uint32_t now);
    virtual void periodic_1Hz();
    virtual void periodic_fullrate();

    bool ShouldLog(bool is_critical);
    virtual bool WritesOK() const = 0;
    virtual bool StartNewLogOK() const;

    /*
      read a block
    */
    virtual bool WriteBlockCheckStartupMessages();
    virtual void WriteMoreStartupMessages();
    virtual void push_log_blocks();

    LoggerMessageWriter_DFLogStart *_startup_messagewriter;
    bool _writing_startup_messages;

    uint8_t _internal_errors;
    uint32_t _dropped;

    // must be called when a new log is being started:
    virtual void start_new_log_reset_variables();

    virtual bool _WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical) = 0;

    bool _initialised;

private:

    uint32_t _last_periodic_1Hz;
    uint32_t _last_periodic_10Hz;
    bool have_logged_armed;

    void validate_WritePrioritisedBlock(const void *pBuffer, uint16_t size);
};
