#include "LR_MsgHandler.h"
#include "LogReader.h"
#include "Replay.h"

#include <AP_DAL/AP_DAL.h>

#include <cinttypes>
#include <cstddef>

extern const AP_HAL::HAL& hal;

#define MSG_CREATE(sname,msgbytes) log_ ##sname msg; memcpy((void*)&msg, (msgbytes)+3, offsetof(log_ ##sname, _end));

LR_MsgHandler::LR_MsgHandler(struct log_Format &_f) :
    MsgHandler(_f) {
}

void LR_MsgHandler_RFRH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RFRH, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RFRF::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RFRF, msgbytes);
#define MAP_FLAG(flag1, flag2) if (msg.frame_types & uint8_t(flag1)) msg.frame_types |= uint8_t(flag2)
    /*
      when we force an EKF we map the trigger flags over
     */
    if (replay_force_ekf2) {
        MAP_FLAG(AP_DAL::FrameType::InitialiseFilterEKF3, AP_DAL::FrameType::InitialiseFilterEKF2);
        MAP_FLAG(AP_DAL::FrameType::UpdateFilterEKF3, AP_DAL::FrameType::UpdateFilterEKF2);
        MAP_FLAG(AP_DAL::FrameType::LogWriteEKF3, AP_DAL::FrameType::LogWriteEKF2);
    }
    if (replay_force_ekf3) {
        MAP_FLAG(AP_DAL::FrameType::InitialiseFilterEKF2, AP_DAL::FrameType::InitialiseFilterEKF3);
        MAP_FLAG(AP_DAL::FrameType::UpdateFilterEKF2, AP_DAL::FrameType::UpdateFilterEKF3);
        MAP_FLAG(AP_DAL::FrameType::LogWriteEKF2, AP_DAL::FrameType::LogWriteEKF3);
    }
#undef MAP_FLAG
    AP::dal().handle_message(msg, ekf2, ekf3);
}

void LR_MsgHandler_RFRN::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RFRN, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_REV2::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(REV2, msgbytes);

    switch ((AP_DAL::Event)msg.event) {

    case AP_DAL::Event::resetGyroBias:
        ekf2.resetGyroBias();
        break;
    case AP_DAL::Event::resetHeightDatum:
        ekf2.resetHeightDatum();
        break;
    case AP_DAL::Event::setTerrainHgtStable:
        ekf2.setTerrainHgtStable(true);
        break;
    case AP_DAL::Event::unsetTerrainHgtStable:
        ekf2.setTerrainHgtStable(false);
        break;
    case AP_DAL::Event::requestYawReset:
        ekf2.requestYawReset();
        break;
    case AP_DAL::Event::checkLaneSwitch:
        ekf2.checkLaneSwitch();
        break;
    case AP_DAL::Event::setSourceSet0 ... AP_DAL::Event::setSourceSet2:
        break;
    }
    if (replay_force_ekf3) {
        LR_MsgHandler_REV3 h{f, ekf2, ekf3};
        h.process_message(msgbytes);
    }
}

void LR_MsgHandler_RSO2::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RSO2, msgbytes);
    Location loc;
    loc.lat = msg.lat;
    loc.lng = msg.lng;
    loc.alt = msg.alt;
    ekf2.setOriginLLH(loc);

    if (replay_force_ekf3) {
        LR_MsgHandler_RSO2 h{f, ekf2, ekf3};
        h.process_message(msgbytes);
    }
}

void LR_MsgHandler_RWA2::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RWA2, msgbytes);
    ekf2.writeDefaultAirSpeed(msg.airspeed);
    if (replay_force_ekf3) {
        LR_MsgHandler_RWA2 h{f, ekf2, ekf3};
        h.process_message(msgbytes);
    }
}


void LR_MsgHandler_REV3::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(REV3, msgbytes);

    switch ((AP_DAL::Event)msg.event) {

    case AP_DAL::Event::resetGyroBias:
        ekf3.resetGyroBias();
        break;
    case AP_DAL::Event::resetHeightDatum:
        ekf3.resetHeightDatum();
        break;
    case AP_DAL::Event::setTerrainHgtStable:
        ekf3.setTerrainHgtStable(true);
        break;
    case AP_DAL::Event::unsetTerrainHgtStable:
        ekf3.setTerrainHgtStable(false);
        break;
    case AP_DAL::Event::requestYawReset:
        ekf3.requestYawReset();
        break;
    case AP_DAL::Event::checkLaneSwitch:
        ekf3.checkLaneSwitch();
        break;
    case AP_DAL::Event::setSourceSet0 ... AP_DAL::Event::setSourceSet2:
        ekf3.setPosVelYawSourceSet(uint8_t(msg.event)-uint8_t(AP_DAL::Event::setSourceSet0));
        break;
    }

    if (replay_force_ekf2) {
        LR_MsgHandler_REV2 h{f, ekf2, ekf3};
        h.process_message(msgbytes);
    }
}

void LR_MsgHandler_RSO3::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RSO3, msgbytes);
    Location loc;
    loc.lat = msg.lat;
    loc.lng = msg.lng;
    loc.alt = msg.alt;
    ekf3.setOriginLLH(loc);
    if (replay_force_ekf2) {
        LR_MsgHandler_RSO2 h{f, ekf2, ekf3};
        h.process_message(msgbytes);
    }
}

void LR_MsgHandler_RWA3::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RWA3, msgbytes);
    ekf3.writeDefaultAirSpeed(msg.airspeed, msg.uncertainty);
    if (replay_force_ekf2) {
        LR_MsgHandler_RWA2 h{f, ekf2, ekf3};
        h.process_message(msgbytes);
    }
}

void LR_MsgHandler_REY3::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(REY3, msgbytes);
    ekf3.writeEulerYawAngle(msg.yawangle, msg.yawangleerr, msg.timestamp_ms, msg.type);
}

void LR_MsgHandler_RISH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RISH, msgbytes);
    AP::dal().handle_message(msg);
}
void LR_MsgHandler_RISI::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RISI, msgbytes);
    AP::dal().handle_message(msg);
}
void LR_MsgHandler_RISJ::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RISJ, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RASH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RASH, msgbytes);
    AP::dal().handle_message(msg);
}
void LR_MsgHandler_RASI::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RASI, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RBRH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RBRH, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RBRI::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RBRI, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RRNH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RRNH, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RRNI::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RRNI, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RGPH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RGPH, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RGPI::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RGPI, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RGPJ::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RGPJ, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RMGH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RMGH, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RMGI::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RMGI, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RBCH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RBCH, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RBCI::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RBCI, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RVOH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RVOH, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_ROFH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(ROFH, msgbytes);
    AP::dal().handle_message(msg, ekf2, ekf3);
}

void LR_MsgHandler_RWOH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RWOH, msgbytes);
    AP::dal().handle_message(msg, ekf2, ekf3);
}

void LR_MsgHandler_RBOH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RBOH, msgbytes);
    AP::dal().handle_message(msg, ekf2, ekf3);
}

#if EK3_FEATURE_REPLAY_SNAPSHOT
/*
  EKF snapshot assembly for LOG_REPLAY=2 logs. Each RSNH restarts
  assembly for its core, so a partially written snapshot that the
  firmware retried is discarded cleanly.
 */
static struct {
    uint8_t *buf;
    uint32_t total_len;
    uint16_t num_chunks;
    uint16_t chunks_seen;
    uint8_t seen[(RSN_SNAPSHOT_MAX_CHUNKS+7)/8];
    uint8_t version;
    uint8_t ftype_size;
} snapshot_assembly[MAX_EKF_CORES];

void LR_MsgHandler_RSNH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RSNH, msgbytes);
    if (msg.core >= ARRAY_SIZE(snapshot_assembly)) {
        return;
    }
    auto &assembly = snapshot_assembly[msg.core];
    free(assembly.buf);
    assembly.buf = nullptr;
    if (msg.total_len == 0 || msg.total_len > RSN_SNAPSHOT_MAX_LEN_BYTES) {
        ::printf("RSNH core %u: bad total_len %u, ignored\n",
                 unsigned(msg.core), unsigned(msg.total_len));
        return;
    }
    assembly.buf = (uint8_t *)calloc(1, msg.total_len);
    if (assembly.buf == nullptr) {
        ::printf("RSNH core %u: allocation failed\n", unsigned(msg.core));
        return;
    }
    assembly.total_len = msg.total_len;
    assembly.num_chunks = (msg.total_len + RSND_CHUNK_LEN_BYTES - 1) / RSND_CHUNK_LEN_BYTES;
    assembly.chunks_seen = 0;
    memset(assembly.seen, 0, sizeof(assembly.seen));
    assembly.version = msg.version;
    assembly.ftype_size = msg.ftype_size;
}

void LR_MsgHandler_RSND::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RSND, msgbytes);
    if (msg.core >= ARRAY_SIZE(snapshot_assembly)) {
        return;
    }
    auto &assembly = snapshot_assembly[msg.core];
    const uint32_t ofs = msg.seq * RSND_CHUNK_LEN_BYTES;
    if (assembly.buf == nullptr || msg.seq >= assembly.num_chunks) {
        return;
    }
    // every chunk is full sized except the last
    const uint32_t expected = MIN(assembly.total_len - ofs, uint32_t(RSND_CHUNK_LEN_BYTES));
    if (msg.len != expected) {
        return;
    }
    // a chunk can appear twice if one log backend dropped it and the
    // firmware retried; only the first copy counts towards completion
    if (assembly.seen[msg.seq/8] & (1U << (msg.seq%8))) {
        return;
    }
    assembly.seen[msg.seq/8] |= 1U << (msg.seq%8);
    assembly.chunks_seen++;
    memcpy(&assembly.buf[ofs], msg.data, msg.len);
    if (assembly.chunks_seen == assembly.num_chunks) {
        if (!ekf3.loadCoreSnapshot(msg.core, assembly.version, assembly.ftype_size,
                                   assembly.buf, assembly.total_len)) {
            ::printf("EKF3 snapshot core %u load failed\n", unsigned(msg.core));
        }
        free(assembly.buf);
        assembly.buf = nullptr;
    }
}
#endif  // EK3_FEATURE_REPLAY_SNAPSHOT

void LR_MsgHandler_REPH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(REPH, msgbytes);
    AP::dal().handle_message(msg, ekf2, ekf3);
}

void LR_MsgHandler_RSLL::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RSLL, msgbytes);
    AP::dal().handle_message(msg, ekf2, ekf3);
}

void LR_MsgHandler_REVH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(REVH, msgbytes);
    AP::dal().handle_message(msg, ekf2, ekf3);
}

#include <AP_AHRS/AP_AHRS.h>
#include "VehicleType.h"

bool LR_MsgHandler_PARM::set_parameter(const char *name, const float value)
{
    const char *ignore_parms[] = {
        "LOG_FILE_BUFSIZE",
        "LOG_DISARMED"
    };
    for (uint8_t i=0; i < ARRAY_SIZE(ignore_parms); i++) {
        if (strncmp(name, ignore_parms[i], AP_MAX_NAME_SIZE) == 0) {
            ::printf("Ignoring set of %s to %f\n", name, value);
            return true;
        }
    }

    return LogReader::set_parameter(name, value);
}

void LR_MsgHandler_PARM::process_message(uint8_t *msg)
{
    const uint8_t parameter_name_len = AP_MAX_NAME_SIZE + 1; // null-term
    char parameter_name[parameter_name_len];

    require_field(msg, "Name", parameter_name, parameter_name_len);

    float value = require_field_float(msg, "Value");
    set_parameter(parameter_name, value);
}

void LR_MsgHandler_RTER::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RTER, msgbytes);
    AP::dal().handle_message(msg, ekf2, ekf3);
}
