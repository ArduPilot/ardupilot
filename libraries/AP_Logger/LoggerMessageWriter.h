#pragma once

#include "AP_Logger_Backend.h"

class LoggerMessageWriter {
public:

    virtual void reset() = 0;
    virtual void process() = 0;
    virtual bool finished() { return _finished; }

    virtual void set_logger_backend(class AP_Logger_Backend *backend) {
        _logger_backend = backend;
    }

protected:
    bool _finished = false;
    AP_Logger_Backend *_logger_backend = nullptr;
};


class LoggerMessageWriter_WriteSysInfo : public LoggerMessageWriter {
public:

    void reset() override;
    void process() override;

private:
    enum write_sysinfo_blockwriter_stage : uint8_t {
        ws_blockwriter_stage_formats = 0,
        ws_blockwriter_stage_firmware_string,
        ws_blockwriter_stage_git_versions,
        ws_blockwriter_stage_system_id
    };
    write_sysinfo_blockwriter_stage stage;
};

class LoggerMessageWriter_WriteEntireMission : public LoggerMessageWriter {
public:

    void reset() override;
    void process() override;

private:
    enum entire_mission_blockwriter_stage {
        em_blockwriter_stage_write_new_mission_message = 0,
        em_blockwriter_stage_write_mission_items,
        em_blockwriter_stage_done
    };

    uint16_t _mission_number_to_send;
    entire_mission_blockwriter_stage stage;
};

class LoggerMessageWriter_WriteAllRallyPoints : public LoggerMessageWriter {
public:

    void reset() override;
    void process() override;

private:
    enum all_rally_points_blockwriter_stage {
        ar_blockwriter_stage_write_new_rally_message = 0,
        ar_blockwriter_stage_write_all_rally_points,
        ar_blockwriter_stage_done
    };

    uint16_t _rally_number_to_send;
    all_rally_points_blockwriter_stage stage;
};

class LoggerMessageWriter_DFLogStart : public LoggerMessageWriter {
public:
    LoggerMessageWriter_DFLogStart() :
        _writesysinfo(),
        _writeentiremission(),
        _writeallrallypoints()
        {
        }

    virtual void set_logger_backend(class AP_Logger_Backend *backend) override {
        LoggerMessageWriter::set_logger_backend(backend);
        _writesysinfo.set_logger_backend(backend);
        _writeentiremission.set_logger_backend(backend);
        _writeallrallypoints.set_logger_backend(backend);
    }

    void reset() override;
    void process() override;
    bool fmt_done() { return _fmt_done; }

private:

    enum log_start_blockwriter_stage {
        ls_blockwriter_stage_formats = 0,
        ls_blockwriter_stage_units,
        ls_blockwriter_stage_multipliers,
        ls_blockwriter_stage_format_units,
        ls_blockwriter_stage_parms,
        ls_blockwriter_stage_sysinfo,
        ls_blockwriter_stage_write_entire_mission,
        ls_blockwriter_stage_write_all_rally_points,
        ls_blockwriter_stage_vehicle_messages,
        ls_blockwriter_stage_done,
    };

    bool _fmt_done;

    log_start_blockwriter_stage stage;

    uint16_t next_format_to_send;

    uint8_t _next_unit_to_send;
    uint8_t _next_format_unit_to_send;
    uint8_t _next_multiplier_to_send;

    AP_Param::ParamToken token;
    AP_Param *ap;
    enum ap_var_type type;


    LoggerMessageWriter_WriteSysInfo _writesysinfo;
    LoggerMessageWriter_WriteEntireMission _writeentiremission;
    LoggerMessageWriter_WriteAllRallyPoints _writeallrallypoints;
};
