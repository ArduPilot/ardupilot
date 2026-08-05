#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF3.h"
#include "AP_NavEKF3_core.h"
#include <AP_DAL/AP_DAL.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

// the AP_DAL_Standalone example links without AP_Logger; this exclusion
// cannot live in AP_NavEKF3_feature.h, where APM_BUILD_TYPE must not be
// evaluated (see there)
#if APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone)
#undef EK3_FEATURE_REPLAY_SNAPSHOT
#define EK3_FEATURE_REPLAY_SNAPSHOT 0
#endif

#if EK3_FEATURE_REPLAY_SNAPSHOT
/*
  LOG_REPLAY=2 snapshot support. The snapshot carries the slow-to-learn
  filter state so Replay can warm-start at arming; the boolean order in
  the flags word is part of the schema, so bump SNAPSHOT_VERSION when
  changing any of this.
 */

// each buffer section is a 1 byte id plus a 2 byte length, then payload
static constexpr uint8_t SECTION_HDR_LEN = 3;

// Snapshot::flags bit layout; bits 0-17 hold the bools arrays in
// pack/unpackSnapshotFlags, in order
static constexpr uint8_t SNAPSHOT_FLAGS_AIDING_SHIFT = 18;   // 2 bits
static constexpr uint32_t SNAPSHOT_FLAGS_AIDING_MASK = 3U;
static constexpr uint32_t SNAPSHOT_FLAG_POS_TIMEOUT = 1U<<20;
static constexpr uint32_t SNAPSHOT_FLAG_VEL_TIMEOUT = 1U<<21;
static constexpr uint32_t SNAPSHOT_FLAG_HGT_TIMEOUT = 1U<<22;
static constexpr uint32_t SNAPSHOT_FLAG_TAS_TIMEOUT = 1U<<23;
static constexpr uint32_t SNAPSHOT_FLAG_DRAG_TIMEOUT = 1U<<24;
static constexpr uint32_t SNAPSHOT_FLAG_COMMON_ORIGIN = 1U<<27;
static constexpr uint32_t SNAPSHOT_FLAG_ARMED = 1U<<28;
static constexpr uint32_t SNAPSHOT_FLAG_PREV_ARMED = 1U<<29;

// number of entries in the pack/unpackSnapshotFlags bools arrays
static constexpr uint8_t SNAPSHOT_FLAGS_NUM_BOOLS = 18;
static_assert(SNAPSHOT_FLAGS_NUM_BOOLS <= SNAPSHOT_FLAGS_AIDING_SHIFT,
              "snapshot bool flags overlap the aiding mode bits");

/*
  buffer sections are tagged with an id so a loader built with a
  different EKF feature set can skip sections it does not have and
  tolerate ones the writer did not include. The fixed header is not
  feature-tagged: SNAPSHOT_VERSION and ftype_size must still match.
 */
enum class SnapshotSection : uint8_t {
    IMU = 1,
    OUTPUT = 2,
    GPS = 3,
    MAG = 4,
    BARO = 5,
    TAS = 6,
    RANGE = 7,
    OPTFLOW = 8,
    BODY_ODOM = 9,
    GPS_YAW = 10,
    DRAG = 11,
};

/*
  the section list lives here and nowhere else: snapshot_len and
  serialiseSnapshot both iterate it, so the allocated length and the
  serialised layout cannot disagree
 */
template <typename F>
void NavEKF3_core::forEachSnapshotSection(F &fn) const
{
    fn(SnapshotSection::IMU, storedIMU);
    fn(SnapshotSection::OUTPUT, storedOutput);
    fn(SnapshotSection::GPS, storedGPS);
    fn(SnapshotSection::MAG, storedMag);
    fn(SnapshotSection::BARO, storedBaro);
    fn(SnapshotSection::TAS, storedTAS);
#if EK3_FEATURE_RANGEFINDER_MEASUREMENTS
    fn(SnapshotSection::RANGE, storedRange);
#endif
    fn(SnapshotSection::GPS_YAW, storedYawAng);
#if EK3_FEATURE_DRAG_FUSION
    fn(SnapshotSection::DRAG, storedDrag);
#endif
#if EK3_FEATURE_OPTFLOW_FUSION
    fn(SnapshotSection::OPTFLOW, storedOF);
#endif
#if EK3_FEATURE_BODY_ODOM
    fn(SnapshotSection::BODY_ODOM, storedBodyOdm);
#endif
}

// sums SECTION_HDR_LEN plus payload for each section
struct SnapshotLenAccumulator {
    uint32_t len;
    template <typename T>
    void operator()(SnapshotSection, const T &b) {
        len += SECTION_HDR_LEN + b.serialise_len();
    }
};

// writes each section's header and payload, advancing p
struct SnapshotSectionWriter {
    uint8_t *p;
    template <typename T>
    void operator()(SnapshotSection id, const T &b) {
        const uint16_t l = b.serialise_len();
        p[0] = uint8_t(id);
        p[1] = l & 0xFF;
        p[2] = l >> 8;
        b.serialise(&p[SECTION_HDR_LEN], l);
        p += SECTION_HDR_LEN + l;
    }
};

uint16_t NavEKF3_core::snapshot_len(void) const
{
    // the serialised layout is the on-disk schema; if this fires the
    // wire format has changed - update the size AND bump SNAPSHOT_VERSION
    static_assert(sizeof(Snapshot) == (sizeof(ftype) == 8 ? 5472 : 2768),
                  "Snapshot layout changed: bump SNAPSHOT_VERSION and update this size");
    SnapshotLenAccumulator acc { sizeof(Snapshot) };
    forEachSnapshotSection(acc);
    // report an impossible length rather than overflowing the chunk
    // numbering
    if (acc.len > RSN_SNAPSHOT_MAX_LEN_BYTES) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return 0;
    }
    return acc.len;
}

uint16_t NavEKF3_core::snapshot_age(uint32_t last_time_ms) const
{
    if (last_time_ms == 0 || last_time_ms > imuSampleTime_ms) {
        return UINT16_MAX;
    }
    return MIN(imuSampleTime_ms - last_time_ms, UINT16_MAX - 1U);
}

/*
  ages are rebased onto the recorded snapshot time, not the local
  clock: the snapshot can be applied before the loader has processed
  its first frame header, when the local time is still zero
 */
void NavEKF3_core::restore_snapshot_age(uint32_t &last_time_ms, uint16_t age, uint32_t base_time_ms) const
{
    if (age == UINT16_MAX || age >= base_time_ms) {
        last_time_ms = 0;
    } else {
        last_time_ms = base_time_ms - age;
    }
}

uint32_t NavEKF3_core::packSnapshotFlags(void) const
{
    const bool bools[] = {
        tiltAlignComplete,
        yawAlignComplete,
        magStateInitComplete,
        delAngBiasLearned,
        magFieldLearned,
        inhibitWindStates,
        windStatesAligned,
        inhibitMagStates,
        inhibitDelAngBiasStates,
        inhibitDelVelBiasStates,
        validOrigin,
        onGround,
        prevOnGround,
        inFlight,
        prevInFlight,
        onGroundNotMoving,
        finalInflightYawInit,
        finalInflightMagInit,
    };
    static_assert(ARRAY_SIZE(bools) == SNAPSHOT_FLAGS_NUM_BOOLS,
                  "flags schema changed: bump SNAPSHOT_VERSION and update both bools arrays");
    uint32_t flags = 0;
    for (uint8_t i=0; i<ARRAY_SIZE(bools); i++) {
        if (bools[i]) {
            flags |= (1U<<i);
        }
    }
    // aiding mode encoded so 0 means AID_NONE regardless of the enum
    // values
    switch (PV_AidingMode) {
    case AID_NONE:
        break;
    case AID_ABSOLUTE:
        flags |= 1U<<SNAPSHOT_FLAGS_AIDING_SHIFT;
        break;
    case AID_RELATIVE:
        flags |= 2U<<SNAPSHOT_FLAGS_AIDING_SHIFT;
        break;
    }
    if (posTimeout) {
        flags |= SNAPSHOT_FLAG_POS_TIMEOUT;
    }
    if (velTimeout) {
        flags |= SNAPSHOT_FLAG_VEL_TIMEOUT;
    }
    if (hgtTimeout) {
        flags |= SNAPSHOT_FLAG_HGT_TIMEOUT;
    }
    if (tasTimeout) {
        flags |= SNAPSHOT_FLAG_TAS_TIMEOUT;
    }
    if (dragTimeout) {
        flags |= SNAPSHOT_FLAG_DRAG_TIMEOUT;
    }
    if (frontend->common_origin_valid) {
        flags |= SNAPSHOT_FLAG_COMMON_ORIGIN;
    }
    if (motorsArmed) {
        flags |= SNAPSHOT_FLAG_ARMED;
    }
    if (prevMotorsArmed) {
        flags |= SNAPSHOT_FLAG_PREV_ARMED;
    }
    return flags;
}

void NavEKF3_core::unpackSnapshotFlags(uint32_t flags)
{
    bool *bools[] = {
        &tiltAlignComplete,
        &yawAlignComplete,
        &magStateInitComplete,
        &delAngBiasLearned,
        &magFieldLearned,
        &inhibitWindStates,
        &windStatesAligned,
        &inhibitMagStates,
        &inhibitDelAngBiasStates,
        &inhibitDelVelBiasStates,
        &validOrigin,
        &onGround,
        &prevOnGround,
        &inFlight,
        &prevInFlight,
        &onGroundNotMoving,
        &finalInflightYawInit,
        &finalInflightMagInit,
    };
    static_assert(ARRAY_SIZE(bools) == SNAPSHOT_FLAGS_NUM_BOOLS,
                  "flags schema changed: bump SNAPSHOT_VERSION and update both bools arrays");
    for (uint8_t i=0; i<ARRAY_SIZE(bools); i++) {
        *bools[i] = (flags & (1U<<i)) != 0;
    }
    switch ((flags >> SNAPSHOT_FLAGS_AIDING_SHIFT) & SNAPSHOT_FLAGS_AIDING_MASK) {
    case 1:
        PV_AidingMode = AID_ABSOLUTE;
        break;
    case 2:
        PV_AidingMode = AID_RELATIVE;
        break;
    default:
        PV_AidingMode = AID_NONE;
        break;
    }
    posTimeout = (flags & SNAPSHOT_FLAG_POS_TIMEOUT) != 0;
    velTimeout = (flags & SNAPSHOT_FLAG_VEL_TIMEOUT) != 0;
    hgtTimeout = (flags & SNAPSHOT_FLAG_HGT_TIMEOUT) != 0;
    tasTimeout = (flags & SNAPSHOT_FLAG_TAS_TIMEOUT) != 0;
    dragTimeout = (flags & SNAPSHOT_FLAG_DRAG_TIMEOUT) != 0;
    motorsArmed = (flags & SNAPSHOT_FLAG_ARMED) != 0;
    prevMotorsArmed = (flags & SNAPSHOT_FLAG_PREV_ARMED) != 0;
}

// map a logged height source byte back to the enum, defaulting to BARO
// for values this build does not know
static AP_NavEKF_Source::SourceZ snapshot_hgt_source(uint8_t v)
{
    switch (AP_NavEKF_Source::SourceZ(v)) {
    case AP_NavEKF_Source::SourceZ::NONE:
    case AP_NavEKF_Source::SourceZ::RANGEFINDER:
    case AP_NavEKF_Source::SourceZ::GPS:
    case AP_NavEKF_Source::SourceZ::BEACON:
    case AP_NavEKF_Source::SourceZ::EXTNAV:
        return AP_NavEKF_Source::SourceZ(v);
    case AP_NavEKF_Source::SourceZ::BARO:
    default:
        return AP_NavEKF_Source::SourceZ::BARO;
    }
}

// parse one section header at p, bounds-checked against end; false if
// the header or its payload would overrun the blob
static bool snapshot_get_section(const uint8_t *p, const uint8_t *end,
                                 SnapshotSection &id, uint16_t &l)
{
    if (end - p < SECTION_HDR_LEN) {
        return false;
    }
    id = SnapshotSection(p[0]);
    l = p[1] | (p[2] << 8);
    return end - p >= SECTION_HDR_LEN + l;
}

void NavEKF3_core::serialiseSnapshot(uint8_t *buf) const
{
    Snapshot &snap = *(Snapshot *)(void *)buf;
    memcpy(&snap.states, &stateStruct, sizeof(snap.states));
    memcpy(&snap.P, &P, sizeof(snap.P));
    snap.out_new = outputDataNew;
    snap.out_delayed = outputDataDelayed;
    snap.del_ang_correction = delAngCorrection;
    snap.vel_err_integral = velErrintegral;
    snap.pos_err_integral = posErrintegral;
    snap.vert_comp_pos = vertCompFiltState.pos;
    snap.vert_comp_vel = vertCompFiltState.vel;
    snap.vert_comp_acc = vertCompFiltState.acc;
    snap.prev_tnb = prevTnb;
    snap.dt_imu_avg = dtIMUavg;
    snap.dt_ekf_avg = dtEkfAvg;
    snap.hgt_rate = hgtRate;
    snap.terrain_state = terrainState;
    snap.baro_hgt_offset = baroHgtOffset;
    snap.earth_mag_var = earthMagFieldVar;
    snap.body_mag_var = bodyMagFieldVar;
    for (uint8_t i=0; i<ARRAY_SIZE(snap.inactive_gyro_bias); i++) {
        if (i < INS_MAX_INSTANCES) {
            snap.inactive_gyro_bias[i] = inactiveBias[i].gyro_bias;
            snap.inactive_accel_bias[i] = inactiveBias[i].accel_bias;
        } else {
            snap.inactive_gyro_bias[i].zero();
            snap.inactive_accel_bias[i].zero();
        }
    }
    snap.last_known_pos_ne = lastKnownPositionNE;
    snap.pos_down_at_takeoff = posDownAtTakeoff;
    snap.mea_hgt_at_takeoff = meaHgtAtTakeOff;
#if EK3_FEATURE_OPTFLOW_FUSION
    snap.terrain_var = Popt;
#else
    snap.terrain_var = 0;
#endif
    snap.bad_imu_vel_err_integral = badImuVelErrIntegral;
    snap.gps_ref_hgt = ekfGpsRefHgt;
    snap.age_vel_ms = snapshot_age(lastVelPassTime_ms);
    snap.age_gpspos_ms = snapshot_age(lastGpsPosPassTime_ms);
    snap.age_hgt_ms = snapshot_age(lastHgtPassTime_ms);
    snap.age_tas_ms = snapshot_age(lastTasPassTime_ms);
    snap.age_flow_ms = snapshot_age(prevFlowFuseTime_ms);
    snap.age_bodyvel_ms = snapshot_age(prevBodyVelFuseTime_ms);
    snap.age_drag_ms = snapshot_age(lastDragPassTime_ms);
    snap.age_unused_ms = UINT16_MAX;
    snap.snapshot_time_ms = imuSampleTime_ms;
    snap.active_hgt_source = uint8_t(activeHgtSource);
    snap.prev_hgt_source = uint8_t(prevHgtSource);
    snap.source_set = frontend->sources.getActiveSourceSet(core_index);
    snap.reserved0 = 0;
    snap.origin_lat = EKF_origin.lat;
    snap.origin_lng = EKF_origin.lng;
    snap.origin_alt_cm = EKF_origin.alt;
    snap.public_origin_lat = public_origin.lat;
    snap.public_origin_lng = public_origin.lng;
    snap.public_origin_alt_cm = public_origin.alt;
    snap.age_arming_ms = snapshot_age(timeAtArming_ms);
    snap.flags = packSnapshotFlags();

    SnapshotSectionWriter writer { buf + sizeof(Snapshot) };
    forEachSnapshotSection(writer);
}

bool NavEKF3_core::deserialiseSnapshot(const uint8_t *buf, uint16_t len)
{
    // sections carry their own sizes, so only the fixed header is checked
    if (len < sizeof(Snapshot)) {
#if APM_BUILD_TYPE(APM_BUILD_Replay)
        ::printf("EKF3 snapshot: len %u below fixed header %u\n", unsigned(len), unsigned(sizeof(Snapshot)));
#endif
        return false;
    }
    const Snapshot &snap = *(const Snapshot *)(const void *)buf;
    // reject numerically invalid state before touching the core
    {
        const ftype *sv = (const ftype *)(const void *)&snap.states;
        for (uint8_t i=0; i<sizeof(snap.states)/sizeof(ftype); i++) {
            if (!isfinite(float(sv[i]))) {
                return false;
            }
        }
        // unit quaternion, with generous slack for logging precision
        const ftype qlen = snap.states.quat.length();
        if (qlen < 0.9f || qlen > 1.1f) {
            return false;
        }
        for (uint8_t i=0; i<ARRAY_SIZE(snap.P); i++) {
            if (!isfinite(float(snap.P[i][i])) || snap.P[i][i] < 0) {
                return false;
            }
        }
        if (!check_latlng(snap.origin_lat, snap.origin_lng)) {
            return false;
        }
    }
    // a cold start runs InitialiseVariables before setting states; do the
    // same so timers and fusion state machines start from now rather than
    // zero, which would fire spurious timeouts and sensor switches
    InitialiseVariables();
    memcpy(&stateStruct, &snap.states, sizeof(stateStruct));
    memcpy(&P, &snap.P, sizeof(P));
    outputDataNew = snap.out_new;
    outputDataDelayed = snap.out_delayed;
    delAngCorrection = snap.del_ang_correction;
    velErrintegral = snap.vel_err_integral;
    posErrintegral = snap.pos_err_integral;
    vertCompFiltState.pos = snap.vert_comp_pos;
    vertCompFiltState.vel = snap.vert_comp_vel;
    vertCompFiltState.acc = snap.vert_comp_acc;
    prevTnb = snap.prev_tnb;
    dtIMUavg = snap.dt_imu_avg;
    dtEkfAvg = snap.dt_ekf_avg;
    hgtRate = snap.hgt_rate;
    terrainState = snap.terrain_state;
    baroHgtOffset = snap.baro_hgt_offset;
    earthMagFieldVar = snap.earth_mag_var;
    bodyMagFieldVar = snap.body_mag_var;
    for (uint8_t i=0; i<ARRAY_SIZE(snap.inactive_gyro_bias) && i<INS_MAX_INSTANCES; i++) {
        inactiveBias[i].gyro_bias = snap.inactive_gyro_bias[i];
        inactiveBias[i].accel_bias = snap.inactive_accel_bias[i];
    }
    lastKnownPositionNE = snap.last_known_pos_ne;
    posDownAtTakeoff = snap.pos_down_at_takeoff;
    meaHgtAtTakeOff = snap.mea_hgt_at_takeoff;
#if EK3_FEATURE_OPTFLOW_FUSION
    Popt = snap.terrain_var;
#endif
    badImuVelErrIntegral = snap.bad_imu_vel_err_integral;
    activeHgtSource = snapshot_hgt_source(snap.active_hgt_source);
    prevHgtSource = snapshot_hgt_source(snap.prev_hgt_source);
    ekfGpsRefHgt = snap.gps_ref_hgt;
    EKF_origin.lat = snap.origin_lat;
    EKF_origin.lng = snap.origin_lng;
    EKF_origin.alt = snap.origin_alt_cm;
    unpackSnapshotFlags(snap.flags);
    // the state index limit follows the inhibit flags via transitions the
    // restored filter never went through
    updateStateIndexLim();
    PV_AidingModePrev = PV_AidingMode;
    const uint32_t snap_t = snap.snapshot_time_ms;
    restore_snapshot_age(lastVelPassTime_ms, snap.age_vel_ms, snap_t);
    restore_snapshot_age(lastGpsPosPassTime_ms, snap.age_gpspos_ms, snap_t);
    restore_snapshot_age(lastHgtPassTime_ms, snap.age_hgt_ms, snap_t);
    restore_snapshot_age(lastTasPassTime_ms, snap.age_tas_ms, snap_t);
    restore_snapshot_age(prevFlowFuseTime_ms, snap.age_flow_ms, snap_t);
    restore_snapshot_age(prevBodyVelFuseTime_ms, snap.age_bodyvel_ms, snap_t);
    restore_snapshot_age(lastDragPassTime_ms, snap.age_drag_ms, snap_t);
    restore_snapshot_age(timeAtArming_ms, uint16_t(snap.age_arming_ms), snap_t);
    statesInitialised = true;

    const uint8_t *p = buf + sizeof(Snapshot);
    const uint8_t *end = buf + len;
    // validate the section framing, and that the IMU and output predictor
    // buffers agree in depth, before mutating any buffer (the fixed state
    // is already restored; callers re-bootstrap on failure)
    {
        const uint8_t *q = p;
        uint8_t imu_sz = 0, out_sz = 0;
        while (q < end) {
            SnapshotSection id;
            uint16_t l;
            if (!snapshot_get_section(q, end, id, l)) {
                statesInitialised = false;
                return false;
            }
            // the first payload byte of a serialised buffer is its depth
            if (id == SnapshotSection::IMU && l >= 1) {
                imu_sz = q[SECTION_HDR_LEN];
            }
            if (id == SnapshotSection::OUTPUT && l >= 1) {
                out_sz = q[SECTION_HDR_LEN];
            }
            q += SECTION_HDR_LEN + l;
        }
        if (imu_sz == 0 || imu_sz != out_sz) {
            statesInitialised = false;
            return false;
        }
    }
    bool imu_restored = false;
    bool output_restored = false;
    bool gps_restored = false;
    bool walk_ok = true;
    while (p < end && walk_ok) {
        SnapshotSection id;
        uint16_t l;
        if (!snapshot_get_section(p, end, id, l)) {
            walk_ok = false;
            break;
        }
        const uint8_t *payload = &p[SECTION_HDR_LEN];
        switch (id) {
        case SnapshotSection::IMU:
            imu_restored = storedIMU.deserialise(payload, l);
            walk_ok = imu_restored;
            break;
        case SnapshotSection::OUTPUT:
            output_restored = storedOutput.deserialise(payload, l);
            walk_ok = output_restored;
            break;
        case SnapshotSection::GPS:
            gps_restored = storedGPS.deserialise(payload, l);
            walk_ok = gps_restored;
            break;
        case SnapshotSection::MAG:
            walk_ok = storedMag.deserialise(payload, l);
            break;
        case SnapshotSection::BARO:
            walk_ok = storedBaro.deserialise(payload, l);
            break;
        case SnapshotSection::TAS:
            walk_ok = storedTAS.deserialise(payload, l);
            break;
#if EK3_FEATURE_RANGEFINDER_MEASUREMENTS
        case SnapshotSection::RANGE:
            walk_ok = storedRange.deserialise(payload, l);
            break;
#endif
        case SnapshotSection::GPS_YAW:
            walk_ok = storedYawAng.deserialise(payload, l);
            break;
#if EK3_FEATURE_DRAG_FUSION
        case SnapshotSection::DRAG:
            walk_ok = storedDrag.deserialise(payload, l);
            break;
#endif
#if EK3_FEATURE_OPTFLOW_FUSION
        case SnapshotSection::OPTFLOW:
            walk_ok = storedOF.deserialise(payload, l);
            break;
#endif
#if EK3_FEATURE_BODY_ODOM
        case SnapshotSection::BODY_ODOM:
            walk_ok = storedBodyOdm.deserialise(payload, l);
            break;
#endif
        default:
            // written by a build with a feature this one lacks
            break;
        }
        p += SECTION_HDR_LEN + l;
    }
    // the delayed horizon and output predictor buffers are the point of
    // the snapshot; anything else missing just stays reset
    if (!walk_ok || !imu_restored || !output_restored) {
#if APM_BUILD_TYPE(APM_BUILD_Replay)
        ::printf("EKF3 snapshot: bad section walk (ok=%u imu=%u out=%u ofs=%u len=%u)\n",
                 unsigned(walk_ok), unsigned(imu_restored), unsigned(output_restored),
                 unsigned(p-buf), unsigned(len));
#endif
        imu_buffer_length = storedIMU.get_size();
        obs_buffer_length = storedGPS.get_size();
        statesInitialised = false;
        return false;
    }
    // the delayed-horizon processing must run over the restored buffers
    // at their original sizes
    imu_buffer_length = storedIMU.get_size();
    if (gps_restored) {
        obs_buffer_length = storedGPS.get_size();
    }
    // the frontend-shared state is only applied once the whole snapshot
    // has validated, so a rejected blob cannot leave it half changed
    frontend->sources.setPosVelYawSourceSet(AP_NavEKF_Source::SourceSetSelection(snap.source_set));
    // refresh the change-detection latches or the first update sees a
    // fake source change and resets position and yaw
    posxy_source_last = frontend->sources.getPosXYSource(core_index);
    yaw_source_last = frontend->sources.getYawSource(core_index);
    // a restored origin never goes through setOrigin, so adopt it as the
    // public origin or all position outputs would be offset from (0,0),
    // and recompute the earth rotation compensation it normally provides
    if (validOrigin) {
        // restore the public origin as it was, not as a copy of the EKF
        // origin: getPosNE reports the delta between the two
        if (snap.flags & SNAPSHOT_FLAG_COMMON_ORIGIN) {
            frontend->common_EKF_origin.lat = snap.public_origin_lat;
            frontend->common_EKF_origin.lng = snap.public_origin_lng;
            frontend->common_EKF_origin.alt = snap.public_origin_alt_cm;
        } else {
            frontend->common_EKF_origin = EKF_origin;
        }
        frontend->common_origin_valid = true;
        calcEarthRateNED(earthRateNED, EKF_origin.lat);
    }
    // outputDataNew, storedOutput and the tracking corrections are all
    // restored, so no StoreOutputReset() here

    return true;
}
#endif  // EK3_FEATURE_REPLAY_SNAPSHOT
