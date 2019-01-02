#pragma once

#include "DataFlash_Backend.h"

#include <AP_Mission/AP_Mission.h>

class DFMessageWriter {
public:

    virtual void reset() = 0;
    virtual void process() = 0;
    virtual bool finished() { return _finished; }

    virtual void set_dataflash_backend(class DataFlash_Backend *backend) {
        _dataflash_backend = backend;
    }

protected:
    bool _finished = false;
    DataFlash_Backend *_dataflash_backend = nullptr;
};


class DFMessageWriter_WriteSysInfo : public DFMessageWriter {
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

class DFMessageWriter_WriteEntireMission : public DFMessageWriter {
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

class DFMessageWriter_DFLogStart : public DFMessageWriter {
public:
    DFMessageWriter_DFLogStart() :
        _writesysinfo(),
        _writeentiremission()
        {
        }

    virtual void set_dataflash_backend(class DataFlash_Backend *backend) override {
        DFMessageWriter::set_dataflash_backend(backend);
        _writesysinfo.set_dataflash_backend(backend);
        _writeentiremission.set_dataflash_backend(backend);
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


    DFMessageWriter_WriteSysInfo _writesysinfo;
    DFMessageWriter_WriteEntireMission _writeentiremission;
};
