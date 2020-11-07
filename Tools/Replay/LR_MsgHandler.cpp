#include "LR_MsgHandler.h"
#include "LogReader.h"
#include "Replay.h"

#include <AP_HAL_Linux/Scheduler.h>
#include <AP_DAL/AP_DAL.h>

#include <cinttypes>

extern const AP_HAL::HAL& hal;

#define MSG_CAST(sname,msg) *((log_ ##sname *)(msg+3))

LR_MsgHandler::LR_MsgHandler(struct log_Format &_f) :
    MsgHandler(_f) {
}

void LR_MsgHandler_RFRH::process_message(uint8_t *msg)
{
    AP::dal().handle_message(MSG_CAST(RFRH,msg));
}

void LR_MsgHandler_RFRF::process_message(uint8_t *msg)
{
    AP::dal().handle_message(MSG_CAST(RFRF,msg), ekf2, ekf3);
}

void LR_MsgHandler_RFRN::process_message(uint8_t *msg)
{
    AP::dal().handle_message(MSG_CAST(RFRN,msg));
}

void LR_MsgHandler_REV2::process_message(uint8_t *msg)
{
    const log_REV2 &rev2 = MSG_CAST(REV2,msg);

    switch ((AP_DAL::Event2)rev2.event) {

    case AP_DAL::Event2::resetGyroBias:
        ekf2.resetGyroBias();
        break;
    case AP_DAL::Event2::resetHeightDatum:
        ekf2.resetHeightDatum();
        break;
    case AP_DAL::Event2::setInhibitGPS:
        ekf2.setInhibitGPS();
        break;
    case AP_DAL::Event2::setTakeoffExpected:
        ekf2.setTakeoffExpected(true);
        break;
    case AP_DAL::Event2::unsetTakeoffExpected:
        ekf2.setTakeoffExpected(false);
        break;
    case AP_DAL::Event2::setTouchdownExpected:
        ekf2.setTouchdownExpected(true);
        break;
    case AP_DAL::Event2::unsetTouchdownExpected:
        ekf2.setTouchdownExpected(false);
        break;
    case AP_DAL::Event2::setInhibitGpsVertVelUse:
        ekf2.setInhibitGpsVertVelUse(true);
        break;
    case AP_DAL::Event2::unsetInhibitGpsVertVelUse:
        ekf2.setInhibitGpsVertVelUse(false);
        break;
    case AP_DAL::Event2::setTerrainHgtStable:
        ekf2.setTerrainHgtStable(true);
        break;
    case AP_DAL::Event2::unsetTerrainHgtStable:
        ekf2.setTerrainHgtStable(false);
        break;
    case AP_DAL::Event2::requestYawReset:
        ekf2.requestYawReset();
        break;
    case AP_DAL::Event2::checkLaneSwitch:
        ekf2.checkLaneSwitch();
        break;
    }
}

void LR_MsgHandler_RSO2::process_message(uint8_t *msg)
{
    const log_RSO2 &rso2 = MSG_CAST(RSO2,msg);
    Location loc;
    loc.lat = rso2.lat;
    loc.lng = rso2.lng;
    loc.alt = rso2.alt;
    ekf2.setOriginLLH(loc);
}

void LR_MsgHandler_RWA2::process_message(uint8_t *msg)
{
    const log_RWA2 &rwa2 = MSG_CAST(RWA2,msg);
    ekf2.writeDefaultAirSpeed(rwa2.airspeed);
}


void LR_MsgHandler_REV3::process_message(uint8_t *msg)
{
    const log_REV3 &rev3 = MSG_CAST(REV3,msg);

    switch ((AP_DAL::Event3)rev3.event) {

    case AP_DAL::Event3::resetGyroBias:
        ekf3.resetGyroBias();
        break;
    case AP_DAL::Event3::resetHeightDatum:
        ekf3.resetHeightDatum();
        break;
    case AP_DAL::Event3::setInhibitGPS:
        ekf3.setInhibitGPS();
        break;
    case AP_DAL::Event3::setTakeoffExpected:
        ekf3.setTakeoffExpected(true);
        break;
    case AP_DAL::Event3::unsetTakeoffExpected:
        ekf3.setTakeoffExpected(false);
        break;
    case AP_DAL::Event3::setTouchdownExpected:
        ekf3.setTouchdownExpected(true);
        break;
    case AP_DAL::Event3::unsetTouchdownExpected:
        ekf3.setTouchdownExpected(false);
        break;
    case AP_DAL::Event3::setInhibitGpsVertVelUse:
        ekf3.setInhibitGpsVertVelUse(true);
        break;
    case AP_DAL::Event3::unsetInhibitGpsVertVelUse:
        ekf3.setInhibitGpsVertVelUse(false);
        break;
    case AP_DAL::Event3::setTerrainHgtStable:
        ekf3.setTerrainHgtStable(true);
        break;
    case AP_DAL::Event3::unsetTerrainHgtStable:
        ekf3.setTerrainHgtStable(false);
        break;
    case AP_DAL::Event3::requestYawReset:
        ekf3.requestYawReset();
        break;
    case AP_DAL::Event3::checkLaneSwitch:
        ekf3.checkLaneSwitch();
        break;
    }
}

void LR_MsgHandler_RSO3::process_message(uint8_t *msg)
{
    const log_RSO3 &rso3 = MSG_CAST(RSO3,msg);
    Location loc;
    loc.lat = rso3.lat;
    loc.lng = rso3.lng;
    loc.alt = rso3.alt;
    ekf3.setOriginLLH(loc);
}

void LR_MsgHandler_RWA3::process_message(uint8_t *msg)
{
    const log_RWA3 &rwa3 = MSG_CAST(RWA3,msg);
    ekf3.writeDefaultAirSpeed(rwa3.airspeed);
}

void LR_MsgHandler_REY3::process_message(uint8_t *msg)
{
    const log_REY3 &rey3 = MSG_CAST(REY3,msg);
    ekf3.writeEulerYawAngle(rey3.yawangle, rey3.yawangleerr, rey3.timestamp_ms, rey3.type);
}

void LR_MsgHandler_RISH::process_message(uint8_t *msg)
{
    AP::dal().handle_message(MSG_CAST(RISH,msg));
}
void LR_MsgHandler_RISI::process_message(uint8_t *msg)
{
    AP::dal().handle_message(MSG_CAST(RISI,msg));
}

void LR_MsgHandler_RASH::process_message(uint8_t *msg)
{
    AP::dal().handle_message(MSG_CAST(RASH,msg));
}
void LR_MsgHandler_RASI::process_message(uint8_t *msg)
{
    AP::dal().handle_message(MSG_CAST(RASI,msg));
}

void LR_MsgHandler_RBRH::process_message(uint8_t *msg)
{
    AP::dal().handle_message(MSG_CAST(RBRH,msg));
}
void LR_MsgHandler_RBRI::process_message(uint8_t *msg)
{
    AP::dal().handle_message(MSG_CAST(RBRI,msg));
}

void LR_MsgHandler_RRNH::process_message(uint8_t *msg)
{
    AP::dal().handle_message(MSG_CAST(RRNH,msg));
}
void LR_MsgHandler_RRNI::process_message(uint8_t *msg)
{
    AP::dal().handle_message(MSG_CAST(RRNI,msg));
}

void LR_MsgHandler_RGPH::process_message(uint8_t *msg)
{
    AP::dal().handle_message(MSG_CAST(RGPH,msg));
}
void LR_MsgHandler_RGPI::process_message(uint8_t *msg)
{
    AP::dal().handle_message(MSG_CAST(RGPI,msg));
}
void LR_MsgHandler_RGPJ::process_message(uint8_t *msg)
{
    AP::dal().handle_message(MSG_CAST(RGPJ,msg));
}

void LR_MsgHandler_RMGH::process_message(uint8_t *msg)
{
    AP::dal().handle_message(MSG_CAST(RMGH,msg));
}
void LR_MsgHandler_RMGI::process_message(uint8_t *msg)
{
    AP::dal().handle_message(MSG_CAST(RMGI,msg));
}

void LR_MsgHandler_RBCH::process_message(uint8_t *msg)
{
    AP::dal().handle_message(MSG_CAST(RBCH,msg));
}
void LR_MsgHandler_RBCI::process_message(uint8_t *msg)
{
    AP::dal().handle_message(MSG_CAST(RBCI,msg));
}

void LR_MsgHandler_RVOH::process_message(uint8_t *msg)
{
    AP::dal().handle_message(MSG_CAST(RVOH,msg));
}

void LR_MsgHandler_ROFH::process_message(uint8_t *msg)
{
    AP::dal().handle_message(MSG_CAST(ROFH,msg), ekf2, ekf3);
}

void LR_MsgHandler_RWOH::process_message(uint8_t *msg)
{
    AP::dal().handle_message(MSG_CAST(RWOH,msg), ekf2, ekf3);
}

void LR_MsgHandler_RBOH::process_message(uint8_t *msg)
{
    AP::dal().handle_message(MSG_CAST(RBOH,msg), ekf2, ekf3);
}

void LR_MsgHandler_REPH::process_message(uint8_t *msg)
{
    AP::dal().handle_message(MSG_CAST(REPH,msg), ekf2, ekf3);
}

void LR_MsgHandler_REVH::process_message(uint8_t *msg)
{
    AP::dal().handle_message(MSG_CAST(REVH,msg), ekf2, ekf3);
}

#include <AP_AHRS/AP_AHRS.h>
#include "VehicleType.h"

bool LR_MsgHandler_PARM::set_parameter(const char *name, const float value)
{
    const char *ignore_parms[] = {
        // "GPS_TYPE",
        // "AHRS_EKF_TYPE",
        // "EK2_ENABLE",
        // "EK3_ENABLE",
        // "COMPASS_ORIENT",
        // "COMPASS_ORIENT2",
        // "COMPASS_ORIENT3",
        "LOG_FILE_BUFSIZE",
        "LOG_DISARMED"
    };
    for (uint8_t i=0; i < ARRAY_SIZE(ignore_parms); i++) {
        if (strncmp(name, ignore_parms[i], AP_MAX_NAME_SIZE) == 0) {
            ::printf("Ignoring set of %s to %f\n", name, value);
            return true;
        }
    }

    return _set_parameter_callback(name, value);
}

void LR_MsgHandler_PARM::process_message(uint8_t *msg)
{
    const uint8_t parameter_name_len = AP_MAX_NAME_SIZE + 1; // null-term
    char parameter_name[parameter_name_len];
    uint64_t time_us;

    if (field_value(msg, "TimeUS", time_us)) {
    } else {
        // older logs can have a lot of FMT and PARM messages up the
        // front which don't have timestamps.  Since in Replay we run
        // AP_Logger's IO only when stop_clock is called, we can
        // overflow AP_Logger's ringbuffer.  This should force us to
        // do IO:
        hal.scheduler->stop_clock(((Linux::Scheduler*)hal.scheduler)->stopped_clock_usec());
    }

    require_field(msg, "Name", parameter_name, parameter_name_len);

    float value = require_field_float(msg, "Value");
    // if (globals.no_params || replay.check_user_param(parameter_name)) {
    //     printf("Not changing %s to %f\n", parameter_name, value);
    // } else {
    set_parameter(parameter_name, value);
    // }
}
