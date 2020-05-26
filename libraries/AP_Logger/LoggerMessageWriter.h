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
    enum class Stage : uint8_t {
        FIRMWARE_STRING = 0,
        GIT_VERSIONS,
        SYSTEM_ID,
        PARAM_SPACE_USED,
        RC_PROTOCOL
    };
    Stage stage;
};

class LoggerMessageWriter_WriteEntireMission : public LoggerMessageWriter {
public:

    void reset() override;
    void process() override;

private:
    enum Stage {
        WRITE_NEW_MISSION_MESSAGE = 0,
        WRITE_MISSION_ITEMS,
        DONE
    };

    uint16_t _mission_number_to_send;
    Stage stage;
};

class LoggerMessageWriter_WriteAllRallyPoints : public LoggerMessageWriter {
public:

    void reset() override;
    void process() override;

private:
    enum Stage {
        WRITE_NEW_RALLY_MESSAGE = 0,
        WRITE_ALL_RALLY_POINTS,
        DONE
    };

    uint16_t _rally_number_to_send;
    Stage stage = Stage::WRITE_NEW_RALLY_MESSAGE;
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

    // reset some writers so we push stuff out to logs again.  Will
    // only work if we are in state DONE!
    bool writeentiremission();
    bool writeallrallypoints();

private:

    enum Stage {
        FORMATS = 0,
        UNITS,
        MULTIPLIERS,
        FORMAT_UNITS,
        PARMS,
        VEHICLE_MESSAGES,
        RUNNING_SUBWRITERS, // must be last thing to run as we can redo bits of these
        DONE,
    };

    bool _fmt_done;

    Stage stage;

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
