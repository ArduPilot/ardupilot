#pragma once

#include "AP_Logger_Backend.h"

#include <AP_Mission/AP_Mission.h>
#include <AP_Rally/AP_Rally.h>

class LoggerMessageWriter {
public:

    virtual void reset() = 0;
    virtual void process() = 0;
    virtual bool finished() { return _finished; }

    virtual void set_dataflash_backend(class AP_Logger_Backend *backend) {
        _dataflash_backend = backend;
    }

protected:
    bool _finished = false;
    AP_Logger_Backend *_dataflash_backend = nullptr;
};


class LoggerMessageWriter_WriteSysInfo : public LoggerMessageWriter {
public:

    void reset() override;
    void process() override;

private:
    enum write_sysinfo_blockwriter_stage {
        ws_blockwriter_stage_init,
        ws_blockwriter_stage_firmware_string,
        ws_blockwriter_stage_git_versions,
        ws_blockwriter_stage_system_id
    };
    write_sysinfo_blockwriter_stage stage = ws_blockwriter_stage_init;
};

class LoggerMessageWriter_WriteEntireMission : public LoggerMessageWriter {
public:

    void reset() override;
    void process() override;

private:
    enum entire_mission_blockwriter_stage {
        em_blockwriter_stage_init,
        em_blockwriter_stage_write_new_mission_message,
        em_blockwriter_stage_write_mission_items,
        em_blockwriter_stage_done
    };

    uint16_t _mission_number_to_send = 0;
    entire_mission_blockwriter_stage stage = em_blockwriter_stage_init;
};

class LoggerMessageWriter_WriteAllRallyPoints : public LoggerMessageWriter {
public:

    void reset() override;
    void process() override;

private:
    enum all_rally_points_blockwriter_stage {
        ar_blockwriter_stage_init,
        ar_blockwriter_stage_write_new_rally_message,
        ar_blockwriter_stage_write_all_rally_points,
        ar_blockwriter_stage_done
    };

    uint16_t _rally_number_to_send = 0;
    all_rally_points_blockwriter_stage stage = ar_blockwriter_stage_init;
};

class LoggerMessageWriter_DFLogStart : public LoggerMessageWriter {
public:
    LoggerMessageWriter_DFLogStart() :
        _writesysinfo(),
        _writeentiremission()
        {
        }

    virtual void set_dataflash_backend(class AP_Logger_Backend *backend) override {
        LoggerMessageWriter::set_dataflash_backend(backend);
        _writesysinfo.set_dataflash_backend(backend);
        _writeentiremission.set_dataflash_backend(backend);
        _writeallrallypoints.set_dataflash_backend(backend);
    }

    void reset() override;
    void process() override;
    bool fmt_done() { return _fmt_done; }

private:

    enum log_start_blockwriter_stage {
        ls_blockwriter_stage_init,
        ls_blockwriter_stage_formats,
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

    bool _fmt_done = false;

    log_start_blockwriter_stage stage = ls_blockwriter_stage_init;

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
