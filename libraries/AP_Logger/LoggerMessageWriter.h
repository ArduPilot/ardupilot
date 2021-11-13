#pragma once

#include "AP_Logger_Backend.h"
#include <AP_Rally/AP_Rally.h>

class LoggerMessageWriter {
public:

    virtual void reset() = 0;
    virtual void process() = 0;
    virtual bool finished() { return _finished; }

    virtual void set_logger_backend(class AP_Logger_Backend *backend) {
        _logger_backend = backend;
    }

    bool out_of_time_for_writing_messages() const;

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
        VER,  // i.e. the "VER" message
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

#if HAL_LOGGER_FENCE_ENABLED
class LoggerMessageWriter_Write_Polyfence : public LoggerMessageWriter {
public:

    void reset() override;
    void process() override;

private:
    enum Stage {
        WRITE_NEW_FENCE_MESSAGE = 0,
        WRITE_FENCE_ITEMS,
        DONE
    };

    uint16_t _fence_number_to_send;
    Stage stage;
};
#endif // HAL_LOGGER_FENCE_ENABLED

class LoggerMessageWriter_DFLogStart : public LoggerMessageWriter {
public:
    LoggerMessageWriter_DFLogStart() :
        _writesysinfo()
#if AP_MISSION_ENABLED
        , _writeentiremission()
#endif
#if HAL_RALLY_ENABLED
        , _writeallrallypoints()
#endif
#if HAL_LOGGER_FENCE_ENABLED
        , _writeallpolyfence()
#endif
        {
        }

    virtual void set_logger_backend(class AP_Logger_Backend *backend) override {
        LoggerMessageWriter::set_logger_backend(backend);
        _writesysinfo.set_logger_backend(backend);
#if AP_MISSION_ENABLED
        _writeentiremission.set_logger_backend(backend);
#endif
#if HAL_RALLY_ENABLED
        _writeallrallypoints.set_logger_backend(backend);
#endif
#if HAL_LOGGER_FENCE_ENABLED
        _writeallpolyfence.set_logger_backend(backend);
#endif
    }

    bool out_of_time_for_writing_messages() const;

    void reset() override;
    void process() override;
    bool fmt_done() const { return _fmt_done; }
    bool params_done() const { return _params_done; }

    // reset some writers so we push stuff out to logs again.  Will
    // only work if we are in state DONE!
#if AP_MISSION_ENABLED
    bool writeentiremission();
#endif
#if HAL_RALLY_ENABLED
    bool writeallrallypoints();
#endif
#if HAL_LOGGER_FENCE_ENABLED
    bool writeallfence();
#endif

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
    bool _params_done;

    Stage stage;

    uint16_t next_format_to_send;

    uint8_t _next_unit_to_send;
    uint8_t _next_format_unit_to_send;
    uint8_t _next_multiplier_to_send;

    AP_Param::ParamToken token;
    AP_Param *ap;
    float param_default;
    enum ap_var_type type;


    LoggerMessageWriter_WriteSysInfo _writesysinfo;
#if AP_MISSION_ENABLED
    LoggerMessageWriter_WriteEntireMission _writeentiremission;
#endif
#if HAL_RALLY_ENABLED
    LoggerMessageWriter_WriteAllRallyPoints _writeallrallypoints;
#endif
#if HAL_LOGGER_FENCE_ENABLED
    LoggerMessageWriter_Write_Polyfence _writeallpolyfence;
#endif
};
