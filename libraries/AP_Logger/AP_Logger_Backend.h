#pragma once

#include "AP_Logger.h"

class LoggerMessageWriter_DFLogStart;

#define MAX_LOG_FILES 500

class AP_Logger_Backend
{

public:
    FUNCTOR_TYPEDEF(vehicle_startup_message_Writer, void);

    AP_Logger_Backend(AP_Logger &front,
                      class LoggerMessageWriter_DFLogStart *writer);

    vehicle_startup_message_Writer vehicle_message_writer();

    virtual bool CardInserted(void) const = 0;

    // erase handling
    virtual void EraseAll() = 0;

    virtual void Prep() = 0;

    /* Write a block of data at current offset */
    bool WriteBlock(const void *pBuffer, uint16_t size) {
        return WritePrioritisedBlock(pBuffer, size, false);
    }

    bool WriteCriticalBlock(const void *pBuffer, uint16_t size) {
        return WritePrioritisedBlock(pBuffer, size, true);
    }

    bool WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical);

    // high level interface, indexed by the position in the list of logs
    virtual uint16_t find_last_log() = 0;
    virtual void get_log_boundaries(uint16_t list_entry, uint32_t & start_page, uint32_t & end_page) = 0;
    virtual void get_log_info(uint16_t list_entry, uint32_t &size, uint32_t &time_utc) = 0;
    virtual int16_t get_log_data(uint16_t list_entry, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) = 0;
    virtual uint16_t get_num_logs() = 0;
    virtual uint16_t find_oldest_log();

    virtual bool logging_started(void) const = 0;

    virtual void Init() = 0;

    virtual uint32_t bufferspace_available() = 0;

    virtual void PrepForArming();

    virtual void start_new_log() { }

    /* stop logging - close output files etc etc.
     *
     * note that this doesn't stop logging from starting up again
     * immediately - e.g. AP_Logger_MAVLink might get another start
     * packet from a client.
     */
    virtual void stop_logging(void) = 0;
    // asynchronously stop logging, status can be determined through logging_started()
    virtual void stop_logging_async(void) { stop_logging(); }

    void Fill_Format(const struct LogStructure *structure, struct log_Format &pkt);
    void Fill_Format_Units(const struct LogStructure *s, struct log_Format_Units &pkt);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    // currently only AP_Logger_File support this:
    virtual void flush(void) { }
#endif

     // for Logger_MAVlink
    virtual void remote_log_block_status_msg(const GCS_MAVLINK &link,
                                             const mavlink_message_t &msg) { }
    // end for Logger_MAVlink

   virtual void periodic_tasks();

    uint8_t num_types() const;
    const struct LogStructure *structure(uint8_t structure) const;

    uint8_t num_units() const;
    const struct UnitStructure *unit(uint8_t unit) const;

    uint8_t num_multipliers() const;
    const struct MultiplierStructure *multiplier(uint8_t multiplier) const;

    bool Write_EntireMission();
    bool Write_RallyPoint(uint8_t total,
                          uint8_t sequence,
                          const RallyLocation &rally_point);
    bool Write_Rally();
    bool Write_Format(const struct LogStructure *structure);
    bool Write_Message(const char *message);
    bool Write_MessageF(const char *fmt, ...);
    bool Write_Mission_Cmd(const AP_Mission &mission,
                               const AP_Mission::Mission_Command &cmd);
    bool Write_Mode(uint8_t mode, const ModeReason reason = ModeReason::UNKNOWN);
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
    virtual bool logging_enabled() const;
    virtual bool logging_failed() const = 0;

    virtual void vehicle_was_disarmed();

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

    uint16_t _cached_oldest_log;

    uint32_t _dropped;
    uint32_t _log_file_size_bytes;
    // should we rotate when we next stop logging
    bool _rotate_pending;

    // must be called when a new log is being started:
    virtual void start_new_log_reset_variables();
    // convert between log numbering in storage and normalized numbering
    uint16_t log_num_from_list_entry(const uint16_t list_entry);

    uint32_t critical_message_reserved_space(uint32_t bufsize) const {
        // possibly make this a proportional to buffer size?
        uint32_t ret = 1024;
        if (ret > bufsize) {
            // in this case you will only get critical messages
            ret = bufsize;
        }
        return ret;
    };
    uint32_t non_messagewriter_message_reserved_space(uint32_t bufsize) const {
        // possibly make this a proportional to buffer size?
        uint32_t ret = 1024;
        if (ret >= bufsize) {
            // need to allow messages out from the messagewriters.  In
            // this case while you have a messagewriter you won't get
            // any other messages.  This should be a corner case!
            ret = 0;
        }
        return ret;
    };

    virtual bool _WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical) = 0;

    bool _initialised;

    void df_stats_gather(uint16_t bytes_written, uint32_t space_remaining);
    void df_stats_log();
    void df_stats_clear();

private:
    // statistics support
    struct df_stats {
        uint16_t blocks;
        uint32_t bytes;
        uint32_t buf_space_min;
        uint32_t buf_space_max;
        uint32_t buf_space_sigma;
    };
    struct df_stats stats;

    uint32_t _last_periodic_1Hz;
    uint32_t _last_periodic_10Hz;
    bool have_logged_armed;

    void Write_AP_Logger_Stats_File(const struct df_stats &_stats);
    void validate_WritePrioritisedBlock(const void *pBuffer, uint16_t size);
};
