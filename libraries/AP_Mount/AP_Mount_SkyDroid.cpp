#include "AP_Mount_config.h"

#if HAL_MOUNT_SKYDROID_ENABLED

#include "AP_Mount_SkyDroid.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define AP_MOUNT_SKYDROID_UPDATE_INTERVAL_MS 100                 // resend angle or rate targets, and push our attitude, at this interval
#define AP_MOUNT_SKYDROID_HEALTH_TIMEOUT_MS  1000                // timeout for health (based on attitude reports from gimbal)
#define AP_MOUNT_SKYDROID_ATTITUDE_RATE_HZ   50                  // rate we ask the gimbal to stream its attitude to us (matches the 50hz rate AP_Mount::update() is actually called at; doc allows up to 100hz)

// 3 character identifiers
#define AP_MOUNT_SKYDROID_ID3CHAR_GIMBAL_MODE       "PTZ"        // discrete gimbal control, data bytes: 00:stop, 01:up, 02:down, 03:left, 04:right, 05:center, 06:follow, 07:lock head.  Only the follow/lock codes (06/07 - see set_gimbal_lock()) and center (05 - see send_center_command()) are used
#define AP_MOUNT_SKYDROID_ID3CHAR_SPEED_YAW          "GSY"       // individual-axis yaw rate control, data bytes: signed 8bit hex.  The only command confirmed to move yaw on real hardware (GAM/GSM/GAY all silently ignored); its sign is also inverted vs the doc - see send_target_rates()
#define AP_MOUNT_SKYDROID_ID3CHAR_SPEED_PITCH        "GSP"       // individual-axis pitch rate control, data bytes: signed 8bit hex.  Confirmed functional on real hardware, sign matches the doc
// matches the protocol doc's and SkyDroid's own RCSDK's documented 0.5deg/s per LSB
// (max speed +/-63.5 deg/s == 127 * 0.5).  An earlier real-hardware measurement of
// this driver had it at 1/16th of this (0.03125) - that measurement was wrong, not
// the documentation: a real-C11 dataflash log of a sustained full-deflection GSY
// rate-mode command (MNT1_RC_RATE=90, so comfortably saturating to the max LSB
// value of 127) measured yaw moving at a clean, consistent ~64deg/s across three
// independent full-speed sweeps (e.g. a 179deg sweep in exactly 2.80s = 63.97deg/s)
// - matching 0.5deg/s/LSB almost exactly, not 0.03125
#define AP_MOUNT_SKYDROID_AXIS_DPS_PER_LSB 0.5f
#define AP_MOUNT_SKYDROID_AXIS_MAX_DPS    (127 * AP_MOUNT_SKYDROID_AXIS_DPS_PER_LSB)
// every other 3-character command ID is used in exactly one place, so is a plain
// string literal at its own call site instead of a macro defined here

#define AP_MOUNT_SKYDROID_DEBUG 0
#define debug(fmt, args ...) do { if (AP_MOUNT_SKYDROID_DEBUG) { GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SkyDroid: " fmt, ## args); } } while (0)

const char* AP_Mount_SkyDroid::send_message_prefix = "Mount: SkyDroid";

// update mount position - should be called periodically
void AP_Mount_SkyDroid::update()
{
    AP_Mount_Backend::update();

    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // reading incoming packets from gimbal
    read_incoming_packets();

    // update based on mount mode, and send target angles or rates depending on the
    // target type.  Deliberately NOT gated by the 10hz throttle below - AP_Mount::update()
    // is actually called at 50hz (see ArduPlane/ArduCopter's SCHED_TASK entry), and the
    // closed-loop angle control in send_target_angles() benefits from running its
    // P-controller at the full rate rather than being throttled down further
    update_mnt_target();
    send_target_to_gimbal();

    // everything below updates at 10hz
    uint32_t now_ms = AP_HAL::millis();
    if ((now_ms - _last_req_current_info_ms) < AP_MOUNT_SKYDROID_UPDATE_INTERVAL_MS) {
        return;
    }
    _last_req_current_info_ms = now_ms;

    // push our own attitude to the gimbal
    send_attitude_to_gimbal();

    // calls below here called at 1hz
    _last_req_step++;
    if (_last_req_step >= (uint8_t)ReqStep::NUM_STEPS) {
        _last_req_step = 0;
    }
    switch ((ReqStep)_last_req_step) {
    case ReqStep::VERSION:
        // get gimbal firmware version.  Worth retrying until answered rather than
        // asking once: the version determines which command sets the gimbal actually
        // implements (SkyDroid's RCSDK gates its combined yaw+pitch call on firmware
        // >= 0.5, and we've found the combined/absolute-angle commands dead on the
        // firmware we have), so it's the single most useful thing to know when
        // diagnosing a gimbal that connects but won't move
        if (!_got_gimbal_version) {
            request_gimbal_version();
        }
        break;
    case ReqStep::TIME_SYNC:
        // (re)send current UTC time so photos/videos are timestamped correctly - see
        // send_time_sync() for why this is needed and why it's resent periodically
        send_time_sync();
        break;
    case ReqStep::ATTITUDE_ENABLE:
        // (re)request gimbal attitude streaming.  harmless to resend if already enabled,
        // and guards against the enable packet being lost over UDP
        request_gimbal_attitude();
        break;
    case ReqStep::MODEL:
        // get the model name.  Purely informational (reported to the GCS via
        // CAMERA_INFORMATION) - no control decision depends on it, since SkyDroid have
        // confirmed the gimbal-control commands are identical across models.  That's
        // why a slow answer here is harmless and 1hz is plenty: "MOD" has been seen to
        // take minutes to reply on real hardware, and control works throughout
        if (!_got_model_name) {
            request_gimbal_model();
        }
        break;
    case ReqStep::SDCARD:
        // request memory card information
        request_gimbal_sdcard_info();
        break;
    case ReqStep::ATTITUDE_ACCEPT:
        // (re)enable gimbal to accept our attitude pushes
        send_attitude_enable();
        break;
    default:
        // spare steps (5, 7, 8, 9) - nothing to do
        break;
    }
}

// return true if healthy
bool AP_Mount_SkyDroid::healthy() const
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    // unhealthy until we've heard at least one "GAC" attitude report, and if
    // attitude information has not been received recently since then
    const uint32_t last_current_angle_ms = _last_current_angle_ms;
    if (last_current_angle_ms == 0) {
        return false;
    }
    return (AP_HAL::millis() - last_current_angle_ms < AP_MOUNT_SKYDROID_HEALTH_TIMEOUT_MS);
}

// take a picture.  returns true on success
bool AP_Mount_SkyDroid::take_picture()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    // exit immediately if the memory card is abnormal
    if (!_sdcard_healthy) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s SD card error", send_message_prefix);
        return false;
    }

    // "CAP": take picture, data bytes: 01.  sample command: #TPUD2wCAP01
    return send_fixedlen_packet(AddressByte::SYSTEM_AND_IMAGE, "CAP", true, 1);
}

// start or stop video recording.  returns true on success
// set start_recording = true to start record, false to stop recording
bool AP_Mount_SkyDroid::record_video(bool start_recording)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    // exit immediately if the memory card is abnormal
    if (!_sdcard_healthy) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s SD card error", send_message_prefix);
        return false;
    }

    // "REC": record video, data bytes: 00:stop, 01:start.  sample command: #TPUD2wREC01
    if (send_fixedlen_packet(AddressByte::SYSTEM_AND_IMAGE, "REC", true, start_recording ? 1 : 0)) {
        // SkyDroid does not push unsolicited recording-state changes to us so track our own request locally
        _recording = start_recording;
        return true;
    }
    return false;
}

// set zoom specified as a rate.  SkyDroid's digital zoom is stepped, not continuous:
// there is no "stop" data value, only single-shot zoom-in/zoom-out pulses
bool AP_Mount_SkyDroid::set_zoom(ZoomType zoom_type, float zoom_value)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    // only rate based zoom is supported
    if (zoom_type != ZoomType::RATE) {
        return false;
    }

    // zero rate has no corresponding command so treat as a successful no-op
    if (is_zero(zoom_value)) {
        return true;
    }

    // "DZM": digital zoom, data bytes: 0A:zoom+ (single step), 0B:zoom- (single step).
    // sample command: #TPUM2wDZM0A65
    const uint8_t zoom_cmd = (zoom_value < 0) ? 0x0B : 0x0A;  // 0x0B: zoom-, 0x0A: zoom+
    return send_fixedlen_packet(AddressByte::LENS, "DZM", true, zoom_cmd);
}

// send camera settings message to GCS
void AP_Mount_SkyDroid::send_camera_settings(mavlink_channel_t chan) const
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // send CAMERA_SETTINGS message
    mavlink_msg_camera_settings_send(
        chan,
        AP_HAL::millis(),   // time_boot_ms
        _recording ? CAMERA_MODE_VIDEO : CAMERA_MODE_IMAGE, // camera mode (0:image, 1:video, 2:image survey)
        NaNf,               // zoomLevel float, percentage from 0 to 100, NaN if unknown
        NaNf);              // focusLevel float, percentage from 0 to 100, NaN if unknown
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_SkyDroid::get_attitude_quaternion(Quaternion& att_quat)
{
    // fail while we've never actually received a "GAC" attitude report - otherwise
    // callers (e.g. GIMBAL_DEVICE_ATTITUDE_STATUS reporting) would be given a
    // fabricated (0,0,0) attitude as if it were real data.  _last_current_angle_ms
    // is zero-initialised and only ever set by gimbal_angle_analyse() on receipt of
    // a real "GAC" packet, so this is the same "have we ever heard from the gimbal"
    // signal healthy() uses for its own timeout check
    if (_last_current_angle_ms == 0) {
        return false;
    }
    // x=roll (always zero on models with no roll axis, e.g. C11), y=pitch, z=yaw
    att_quat.from_euler(_current_angle_rad.x, _current_angle_rad.y, _current_angle_rad.z);
    return true;
}

// dispatch on the 3-character command ID to the function that consumes that
// message - called by AP_Mount_Backend_TPFrame::read_incoming_packets() once a
// packet's CRC has been verified
void AP_Mount_SkyDroid::handle_message(const char* msg_id)
{
    if (strncmp(msg_id, "GAC", 3) == 0) {
        gimbal_angle_analyse();
    } else if (strncmp(msg_id, "REC", 3) == 0) {
        gimbal_record_analyse();
    } else if (strncmp(msg_id, "SDC", 3) == 0) {
        gimbal_sdcard_analyse();
    } else if (strncmp(msg_id, "VER", 3) == 0) {
        gimbal_version_analyse();
    } else if (strncmp(msg_id, "MOD", 3) == 0) {
        gimbal_model_analyse();
    }
}

// request gimbal to (re)start sending us attitude at 10hz
void AP_Mount_SkyDroid::request_gimbal_attitude()
{
    // "GAA": enable/disable gimbal->us attitude streaming, data bytes: 00:off,
    // 01-64:rate in Hz.  sample command: #TPUG2wGAA0A
    send_fixedlen_packet(AddressByte::GIMBAL, "GAA", true, AP_MOUNT_SKYDROID_ATTITUDE_RATE_HZ);
}

// request gimbal memory card information
void AP_Mount_SkyDroid::request_gimbal_sdcard_info()
{
    // "SDC": get SD card state, data bytes: 00 to query.  sample command: #TPUD2rSDC00
    send_fixedlen_packet(AddressByte::SYSTEM_AND_IMAGE, "SDC", false, 0);
}

// request gimbal version
void AP_Mount_SkyDroid::request_gimbal_version()
{
    // "VER": get firmware version, data bytes always 00.  sample command: #TPUD2rVER00
    send_fixedlen_packet(AddressByte::SYSTEM_AND_IMAGE, "VER", false, 0);
}

// request gimbal model name (e.g. "C11", "C13")
void AP_Mount_SkyDroid::request_gimbal_model()
{
    // "MOD": get model name (e.g. "C11"), data bytes always 00.  sample command: #TPUD2rMOD00
    send_fixedlen_packet(AddressByte::SYSTEM_AND_IMAGE, "MOD", false, 0);
}

// send current UTC date/time to the gimbal so photos/videos are timestamped correctly.
// Confirmed on real hardware that the camera has no RTC of its own and defaults to
// 1970-01-01 without this.  Uses UTC since ArduPilot has no local timezone concept -
// this means driver-triggered captures' timestamps will differ from ones taken via
// SkyDroid's own app (which uses the connected device's local time) by the local UTC
// offset.  Returns false (without sending) if the vehicle doesn't yet have a valid
// time source (e.g. GPS not locked)
bool AP_Mount_SkyDroid::send_time_sync()
{
    uint16_t year, ms;
    uint8_t month, day, hour, min, sec;
    if (!AP::rtc().get_date_and_time_utc(year, month, day, hour, min, sec, ms)) {
        return false;
    }

    // "TIM": set current time, data bytes: hhmmss.ccDDMMYY (15 ASCII chars,
    // cc=hundredths of a second).  Confirmed on real hardware that the camera has no
    // RTC of its own and defaults to 1970-01-01 without this.
    // sample command: #tpUDFwTIM142832.00031218 (2018-12-03 14:28:32.00) - data is 15
    // ASCII chars: hhmmss.ccDDMMYY, cc=hundredths of a second, YY=2-digit year
    uint8_t databuff[16];
    hal.util->snprintf((char*)databuff, ARRAY_SIZE(databuff), "%02u%02u%02u.%02u%02u%02u%02u",
                        hour, min, sec, (unsigned)((ms / 10) % 100),
                        day, month, (unsigned)(year % 100));
    return send_variablelen_packet(HeaderType::VARIABLE_LEN, AddressByte::SYSTEM_AND_IMAGE, "TIM", true, databuff, ARRAY_SIZE(databuff)-1);
}

// (re)enable the gimbal to accept our attitude pushes
bool AP_Mount_SkyDroid::send_attitude_enable()
{
    // "FAE": enable/disable us->gimbal attitude streaming, data bytes: 00:off, 01:on.
    // sample command: #TPUG2wFAE01
    return send_fixedlen_packet(AddressByte::GIMBAL, "FAE", true, 1);
}

// send our current attitude to the gimbal
bool AP_Mount_SkyDroid::send_attitude_to_gimbal()
{
    const int16_t yaw_cd = wrap_180_cd((int32_t)(AP::ahrs().get_yaw_deg() * 100));
    const int16_t pitch_cd = (int16_t)(AP::ahrs().get_pitch_deg() * 100);
    const int16_t roll_cd = (int16_t)(AP::ahrs().get_roll_deg() * 100);

    // 1: fixed-wing, 0: copter/hover - this is SkyDroid's own documented field for
    // this command, and APM_BUILD_TYPE(APM_BUILD_ArduPlane) is the obvious compile-time
    // proxy for it, but we don't actually know what it changes in the gimbal's own
    // firmware, or whether that proxy is the right one (e.g. a Plane holding/loitering
    // isn't continuously moving forward either).  Raised with SkyDroid to find out what
    // this bit actually affects before assuming a "more correct" runtime check would
    // really be better rather than just differently wrong
    const bool fixed_wing = APM_BUILD_TYPE(APM_BUILD_ArduPlane);

    // "FAI": our attitude sent to gimbal, data bytes: yaw+pitch+roll (4hex each,
    // 0.01deg) + mode (1:fixed-wing, 0:hover).  sample command: #tpUG0EwFAI
    uint8_t databuff[15];
    hal.util->snprintf((char*)databuff, ARRAY_SIZE(databuff), "%04X%04X%04X%02X",
                        (uint16_t)yaw_cd, (uint16_t)pitch_cd, (uint16_t)roll_cd, fixed_wing ? 1 : 0);
    return send_variablelen_packet(HeaderType::VARIABLE_LEN, AddressByte::GIMBAL, "FAI", true, databuff, ARRAY_SIZE(databuff)-1);
}

// send angle target in radians to gimbal, by closing the loop ourselves using the
// gimbal's own "GAC" attitude feedback and driving GSY/GSP as the rate actuator.
// There is no absolute-angle command that works on this hardware - GAM/GAY/GAP are
// all silently ignored (see this file's header comment)
void AP_Mount_SkyDroid::send_target_angles(const MountAngleTarget& angle_rad)
{
    // set gimbal's lock state (follow the body-frame target)
    set_gimbal_lock(false);

    // clamp to the configured MNT1_YAW/PITCH_MIN/MAX range (also in degrees) -
    // AP_Mount's frontend does not clamp the target itself before calling us.
    // Roll is deliberately absent: the gimbal self-stabilizes roll and offers no
    // way to command it (see has_roll_control()).  Everything below stays in
    // degrees from here on - the only unit conversions in this function are the
    // unavoidable ones at its boundaries: angle_rad (radians, AP_Mount's own
    // target-type convention) coming in, and _current_angle_rad (radians, kept
    // that way for get_attitude_quaternion()'s benefit) read via degrees() below
    const float yaw_target_deg = constrain_float(degrees(angle_rad.get_bf_yaw()),
                                                  _params.yaw_angle_min, _params.yaw_angle_max);
    const float pitch_target_deg = constrain_float(degrees(angle_rad.pitch),
                                                    _params.pitch_angle_min, _params.pitch_angle_max);

    // if GAC attitude reports have stopped arriving, _current_angle_rad is stale -
    // the error computed against it below would be wrong, and (since it's held
    // fixed once the feed drops) could keep commanding a nonzero rate indefinitely
    // rather than converging.  Command an explicit stop instead of just skipping
    // the send, since a resumed-later GAC feed is the only thing that would
    // otherwise correct a stale outstanding rate command
    if (!healthy()) {
        send_axis_rate(AP_MOUNT_SKYDROID_ID3CHAR_SPEED_YAW, 0);
        send_axis_rate(AP_MOUNT_SKYDROID_ID3CHAR_SPEED_PITCH, 0);
        return;
    }

    // simple P-controller driving GSY/GSP as the rate actuator, using the GAC
    // attitude feedback already parsed by gimbal_angle_analyse().
    //
    // Confirmed on real C11 hardware that kP=2.0 sustains a continuous limit-cycle
    // oscillation on both axes: at that gain, full-scale rate is reached by just
    // ~2deg of error, so the actuator was being driven at max speed for almost any
    // real excursion - combined with the real feedback's lag (GAC round-trip plus
    // the gimbal's own mechanical response), a fast-reacting/early-saturating
    // P-controller overshoots and re-corrects indefinitely instead of settling.
    //
    // kP=1.0 was chosen (before AP_MOUNT_SKYDROID_AXIS_DPS_PER_LSB's real value was
    // known - see that constant's comment) to saturate at ~4deg of error
    // (AP_MOUNT_SKYDROID_AXIS_MAX_DPS/kP), staying clear of the ~2deg real-hardware
    // oscillation zone confirmed below while converging fast enough to meet
    // mount_test_body's tightest check (test_mount_rc_targetting()'s hardcoded
    // 0.1deg tolerance).  Now that AXIS_MAX_DPS is 16x larger (0.5, not 0.03125
    // deg/s per LSB), this saturates at ~63.5deg instead - even further from the
    // 2deg danger zone, so no less safe, but the "~4deg" figure below is no longer
    // accurate to the number, just the reasoning.  NOT YET RE-VALIDATED ON REAL
    // HARDWARE - the original oscillation was only ever found via real-hardware
    // testing, not SITL, so confirm this still settles cleanly (no hunting/dither)
    // on the real C11 before relying on this
    constexpr float kP = 1.0;  // (deg/s of rate command) per (deg of angle error)
    //
    // The deadzone below (stop correcting entirely once close, rather than tapering
    // to an ever-smaller command) guarantees a clean stop rather than dither once
    // within range - its WIDTH is a separate knob from kP, and was originally set to
    // 2.0deg for margin without much thought, wider than mount_test_body's autotest
    // tolerances and hence a real bug, not a hardware ceiling.  0.05deg keeps margin
    // under the 0.1deg check while staying a "stop dead" cutoff (not tapering).
    //
    // IMPORTANT: this deadzone is no longer the binding constraint it was designed
    // to be.  GSY/GSP's wire value is a quantized 8-bit signed LSB - see
    // AP_MOUNT_SKYDROID_AXIS_DPS_PER_LSB, now known to be 0.5deg/s per LSB (not the
    // 0.03125 this was tuned against) - so kP*error now rounds to 0 LSB at
    // send_axis_rate() for any error below ~0.25deg (half an LSB step at this kP),
    // which is COARSER than the 0.05deg deadzone below and than the 0.1deg test
    // tolerance this was tuned to meet.  In other words: the actuator's real,
    // confirmed resolution floor may no longer be fine enough to pass
    // test_mount_rc_targetting() at all via this fixed-gain approach, regardless of
    // deadzone or kP tuning - re-run the autotest and see before assuming this still
    // passes
    constexpr float deadzone_deg = 0.05;  // stop correcting once within this many degrees
    const float yaw_error_deg = wrap_180(yaw_target_deg - degrees(_current_angle_rad.z));
    const float pitch_error_deg = pitch_target_deg - degrees(_current_angle_rad.y);
    const float yaw_rate_dps = angle_error_to_rate(yaw_error_deg, kP, AP_MOUNT_SKYDROID_AXIS_MAX_DPS, deadzone_deg);
    const float pitch_rate_dps = angle_error_to_rate(pitch_error_deg, kP, AP_MOUNT_SKYDROID_AXIS_MAX_DPS, deadzone_deg);

    // GSY's sign is inverted vs AP_Mount's convention (see send_axis_rate below)
    send_axis_rate(AP_MOUNT_SKYDROID_ID3CHAR_SPEED_YAW, -yaw_rate_dps);
    send_axis_rate(AP_MOUNT_SKYDROID_ID3CHAR_SPEED_PITCH, pitch_rate_dps);
}

// send rate target in rad/s to gimbal, directly via GSY/GSP - the only commands
// confirmed to move this hardware (GSM is silently ignored)
void AP_Mount_SkyDroid::send_target_rates(const MountRateTarget& rate_rads)
{
    // set gimbal's lock state if it has changed
    set_gimbal_lock(rate_rads.yaw_is_ef);

    // GSY's sign is inverted vs AP_Mount's convention (confirmed on real hardware: a
    // positive GSY value moves yaw LEFT, not right) - negate here so callers of this
    // function keep using AP_Mount's normal yaw-right-positive convention.  GSP's
    // sign matches AP_Mount's convention (pitch-up-positive) so is passed straight
    // through.  rate_rads.roll is deliberately ignored - the gimbal self-stabilizes
    // roll and offers no way to command it (see has_roll_control())
    send_axis_rate(AP_MOUNT_SKYDROID_ID3CHAR_SPEED_YAW, -degrees(rate_rads.yaw));
    send_axis_rate(AP_MOUNT_SKYDROID_ID3CHAR_SPEED_PITCH, degrees(rate_rads.pitch));
}

// send a single-axis rate command (GSY or GSP) for rate_dps, converted to the wire's
// signed 8bit LSB units using the real-world calibrated scale.  Caller is responsible
// for any axis-specific sign compensation (see send_target_rates() above)
void AP_Mount_SkyDroid::send_axis_rate(const Identifier id, float rate_dps)
{
    const int8_t rate_lsb = constrain_int16(roundf(rate_dps / AP_MOUNT_SKYDROID_AXIS_DPS_PER_LSB), -127, 127);
    send_fixedlen_packet(AddressByte::GIMBAL, id, true, (uint8_t)rate_lsb);
}

// attitude information analysis of gimbal (arrives as "GAC" in response to our "GAA" enable request)
void AP_Mount_SkyDroid::gimbal_angle_analyse()
{
    // consume current angles.  data is yaw, pitch, roll in that order, each 4 hex chars, 0.01deg units
    uint32_t yaw_raw, pitch_raw, roll_raw;
    if (!hex_chars_to_uint32((const char*)&_msg_buff[AP_MOUNT_TPFRAME_MSGOFS_DATA], 4, yaw_raw) ||
        !hex_chars_to_uint32((const char*)&_msg_buff[AP_MOUNT_TPFRAME_MSGOFS_DATA + 4], 4, pitch_raw) ||
        !hex_chars_to_uint32((const char*)&_msg_buff[AP_MOUNT_TPFRAME_MSGOFS_DATA + 8], 4, roll_raw)) {
        return;
    }
    const int16_t yaw_angle_cd = wrap_180_cd((int16_t)yaw_raw);
    const int16_t pitch_angle_cd = (int16_t)pitch_raw;
    // roll comes from the gimbal's own self-stabilization - we report it for telemetry
    // but cannot command it (see has_roll_control())
    const int16_t roll_angle_cd = (int16_t)roll_raw;

    // convert cd to radians
    _current_angle_rad.x = cd_to_rad(roll_angle_cd);
    _current_angle_rad.y = cd_to_rad(pitch_angle_cd);
    _current_angle_rad.z = cd_to_rad(yaw_angle_cd);
    _last_current_angle_ms = AP_HAL::millis();

    // announce gimbal connection to the user on the first attitude report received.
    // this does not depend on the "VER" command (whose model support is undocumented
    // for some SkyDroid models) so it is a more reliable connection signal
    if (!_announced_connected) {
        _announced_connected = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s connected", send_message_prefix);
    }
}

// gimbal video information analysis
void AP_Mount_SkyDroid::gimbal_record_analyse()
{
    // data is 2 ASCII chars ("00" or "01") - only the low digit is ever non-zero, so
    // that's the only one we need to check
    _recording = (_msg_buff[AP_MOUNT_TPFRAME_MSGOFS_DATA + 1] == '1');
}

// information analysis of gimbal storage card
void AP_Mount_SkyDroid::gimbal_sdcard_analyse()
{
    // data is 10 hex chars: 5 for remaining capacity, 5 for total capacity (units MB)
    // all zeros means no card inserted.  A too-short buffer is treated the same as
    // all-zero (no card), matching this function's previous behaviour
    static const uint8_t all_zero_chars[10] = {'0','0','0','0','0','0','0','0','0','0'};
    bool all_zero = true;
    if (_msg_buff_len >= AP_MOUNT_TPFRAME_MSGOFS_DATA + ARRAY_SIZE(all_zero_chars)) {
        all_zero = (memcmp(&_msg_buff[AP_MOUNT_TPFRAME_MSGOFS_DATA], all_zero_chars, ARRAY_SIZE(all_zero_chars)) == 0);
    }
    _sdcard_healthy = !all_zero;
}

// gimbal basic information analysis.  response data is of the form "VX.X.X" (e.g. "V1.0.78")
void AP_Mount_SkyDroid::gimbal_version_analyse()
{
    uint8_t data_buf_len;
    if (!hex_char_to_nibble(_msg_buff[AP_MOUNT_TPFRAME_MSGOFS_DATALEN], data_buf_len) || data_buf_len == 0 ||
        _msg_buff[AP_MOUNT_TPFRAME_MSGOFS_DATA] != 'V') {
        return;
    }

    // version array with index 0=major, 1=minor, 2=patch
    uint8_t version[3] {};
    uint8_t ver_count = 0;
    uint32_t ver_num = 0;
    for (uint8_t i = 1; i < data_buf_len && ver_count < ARRAY_SIZE(version); i++) {
        const uint8_t c = _msg_buff[AP_MOUNT_TPFRAME_MSGOFS_DATA + i];
        if (c == '.') {
            version[ver_count++] = ver_num;
            ver_num = 0;
            continue;
        }
        uint8_t digit;
        if (!hex_char_to_nibble(c, digit)) {
            return;
        }
        ver_num = ver_num * 10 + digit;
    }
    if (ver_count < ARRAY_SIZE(version)) {
        version[ver_count] = ver_num;
    }
    _firmware_ver = (version[2] << 16) | (version[1] << 8) | (version[0]);

    // display gimbal firmware version to user.  Worth reporting prominently: which
    // command sets this gimbal actually implements appears to depend on it (SkyDroid's
    // RCSDK gates its combined yaw+pitch call on firmware >= 0.5, and we've found the
    // combined/absolute-angle commands dead on the firmware we have), so this is the
    // first thing to check when a gimbal connects but won't move
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s firmware v%u.%u.%u",
        send_message_prefix,
        version[0],     // major version
        version[1],     // minor version
        version[2]);    // patch version

    _got_gimbal_version = true;
}

// gimbal model name analysis.  response data is raw ASCII text, e.g. "C13"
void AP_Mount_SkyDroid::gimbal_model_analyse()
{
    uint8_t data_buf_len;
    if (!hex_char_to_nibble(_msg_buff[AP_MOUNT_TPFRAME_MSGOFS_DATALEN], data_buf_len) || data_buf_len == 0) {
        return;
    }
    memset(_model_name, 0, sizeof(_model_name));
    memcpy(_model_name, _msg_buff + AP_MOUNT_TPFRAME_MSGOFS_DATA, MIN((uint8_t)(sizeof(_model_name)-1), data_buf_len));

    // display gimbal model name to user.  Informational only - no control decision
    // depends on it (see this driver's header comment)
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s model %s", send_message_prefix, _model_name);

    _got_model_name = true;
}

// set gimbal's lock vs follow mode
// lock should be true if gimbal should maintain an earth-frame target
// lock is false to follow / maintain a body-frame target
bool AP_Mount_SkyDroid::set_gimbal_lock(bool lock)
{
    if (_last_lock == lock) {
        return true;
    }

    // send message and update lock state.  PTZ data: 0x06 = follow, 0x07 = lock head
    if (send_fixedlen_packet(AddressByte::GIMBAL, AP_MOUNT_SKYDROID_ID3CHAR_GIMBAL_MODE, true, lock ? 0x07 : 0x06)) {
        _last_lock = lock;
        return true;
    }
    return false;
}

// send the gimbal's own one-shot "center" command.  Deliberately not deduped like
// set_gimbal_lock() above - unlike lock/follow (a persistent mode we don't want to
// keep re-sending), center is a one-shot action that should fire every time the
// mode is (re)selected, same as every other RETRACT/NEUTRAL backend's behaviour
bool AP_Mount_SkyDroid::send_center_command()
{
    return send_fixedlen_packet(AddressByte::GIMBAL, AP_MOUNT_SKYDROID_ID3CHAR_GIMBAL_MODE, true, 0x05);
}

// move to a "retracted" position - see this file's header declaration for why this
// uses the gimbal's own "center" command rather than falling through to the
// angle-based conversion (which needs GAC attitude feedback to converge)
void AP_Mount_SkyDroid::send_target_retracted()
{
    send_center_command();
}

// move to a neutral (forward-pointing) position - see this file's header declaration
void AP_Mount_SkyDroid::send_target_neutral()
{
    send_center_command();
}

#endif // HAL_MOUNT_SKYDROID_ENABLED
