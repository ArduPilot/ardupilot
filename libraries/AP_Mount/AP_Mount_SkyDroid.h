/*
  SkyDroid gimbal driver using custom serial protocol (usually run over UDP)

  This is the same "#TP"/"#tp" wire framing used by SkyDroid's OEM supplier
  for the Topotek driver (see AP_Mount_Topotek) - both derive from
  AP_Mount_Backend_TPFrame, which implements the shared framing/CRC/packet-
  send layer.  Neither protocol document ever names or expands what "TP"
  stands for.  The address bytes, command identifiers and units used by
  SkyDroid's own firmware differ from Topotek's, so everything past that
  shared layer is a separate, independent implementation.

  -------------------------------------------------------------------------------------------
  Field                 Index   Bytes       Description
  -------------------------------------------------------------------------------------------
  Frame Header          0       3           #TP (fixed length) or #tp (variable length)
  Address Bit           3       2           source address first, destination address second
  Data_Len              5       1           data length (hex nibble, max 0x0F)
  Control Bit           6       1           r -> query   w -> set/control
  Identification Bit    7       3           3 character command identifier
  Data                  10      Data_Len
  Check Bit                     2           sum of all preceding bytes, output as 2 ASCII hex
                                            characters (high nibble first)

  This one driver covers every model in SkyDroid's "TOP protocol" gimbal camera
  family:
  - control is over UDP only (no direct UART), source address is always 'U'
  - confirmed directly with SkyDroid: the gimbal-control commands are IDENTICAL
    across models (C11, C13, ...).  There is no model-specific control path, and
    this driver deliberately has no model-dependent behaviour.  The C13's extra
    features over the C11 are infrared thermal imaging and laser ranging, neither
    of which this driver currently uses.  Do not reintroduce per-model dispatch
    without new information from SkyDroid - an earlier version of this driver had
    it, and it was wrong
  - confirmed directly with SkyDroid: ROLL IS SELF-STABILIZED BY THE GIMBAL AND HAS
    NO CONTROL COMMAND AT ALL, on any model.  The protocol document does describe
    roll commands ("GAR" angle, "GSR" rate) but the firmware does not implement
    them, so this driver does not send them and reports has_roll_control() == false.
    Roll is still parsed from the gimbal's own attitude reports and passed through
    for telemetry, and our vehicle roll is still pushed to the gimbal ("FAI") to
    feed its stabilizer
  - pitch/yaw ranges are configured via MNT1_PITCH/YAW_MIN/MAX (e.g. the C11 has
    pitch -90 to +10 deg, yaw -90 to +90 deg).  This driver does not hardcode any
    model's limits itself, so it stays correct across the family
  - SkyDroid's documented sign convention is yaw-right-positive, pitch-up-positive, which
    matches AP_Mount's own convention (no sign flip needed, unlike Topotek's protocol)
  - the connected model (e.g. "C11", "C13") is queried at runtime via the "MOD"
    command and reported through CAMERA_INFORMATION.  This is INFORMATIONAL ONLY -
    no control decision depends on it, which matters because "MOD" has been observed
    on real hardware to take anywhere from under a second to 8+ minutes to answer
    (SkyDroid's own SDK documents that camera-side commands are only effective once
    the camera is producing video frames - "需要在出图后设置才有效" - which would
    explain it).  Gimbal-addressed commands (GSY/GSP/PTZ) have usually been observed
    to work well before "MOD" answers - but at least once, on real hardware, NOTHING
    worked (no RC, no GAC, no GCS messages of any kind) for over 5 minutes, so this
    is not a guarantee - root cause of that particular case is still unknown (raised
    with SkyDroid, response pending).  Whatever gates it, this driver deliberately
    does not wait on it: gimbal-addressed commands are always sent unconditionally,
    for whenever the link does come up
  - confirmed on real C11 hardware: the combined and absolute-angle commands (GAM,
    GSM, GAY, GAP) are all silently ignored - only the individual-axis speed
    commands (GSY, GSP) actually move the gimbal, so both rate and (closed-loop)
    angle control are driven through those.  This is believed to be a FIRMWARE
    limitation rather than a model one: SkyDroid's SDK documents its combined
    yaw+pitch call as requiring gimbal firmware >= 0.5, so newer firmware may well
    accept the commands we found dead here.  The gimbal's reported firmware version
    is logged at startup (see gimbal_version_analyse()) to make that checkable
 */

#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_SKYDROID_ENABLED

#include "AP_Mount_Backend_TPFrame.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>

#define AP_MOUNT_SKYDROID_PACKETLEN_MAX     28      // maximum number of bytes in a packet sent to or received from the gimbal

class AP_Mount_SkyDroid : public AP_Mount_Backend_TPFrame
{

public:
    // Constructor
    using AP_Mount_Backend_TPFrame::AP_Mount_Backend_TPFrame;

    // Do not allow copies
    CLASS_NO_COPY(AP_Mount_SkyDroid);

    // update mount position - should be called periodically
    void update() override;

    // return true if healthy
    bool healthy() const override;

    // has_pan_control - returns true if this mount can control its pan (required for multicopters)
    bool has_pan_control() const override { return yaw_range_valid(); };

    // has_roll_control - always false: confirmed directly with SkyDroid that roll is
    // self-stabilized by the gimbal and has no control command on any model in this
    // family, regardless of what MNT1_ROLL_MIN/MAX is set to
    bool has_roll_control() const override { return false; };

    //
    // camera controls
    //

    // take a picture.  returns true on success
    bool take_picture() override;

    // start or stop video recording
    // set start_recording = true to start record, false to stop recording
    bool record_video(bool start_recording) override;

    // set zoom specified as a rate.  SkyDroid's zoom is stepped (not continuous) so
    // each non-zero call sends a single zoom-in/zoom-out pulse
    bool set_zoom(ZoomType zoom_type, float zoom_value) override;

    bool has_camera_information() const override { return true; }
    // return camera vendor name
    void get_camera_vendor_name(char *buf, uint8_t buflen) const override { strncpy(buf, "SkyDroid", buflen); }
    // return camera model name (e.g. "C11", "C13"), queried from the gimbal via the "MOD" command.
    // this same driver supports every model in SkyDroid's "TOP protocol" gimbal camera family;
    // the model name lets the GCS show which one is actually connected
    void get_camera_model_name(char *buf, uint8_t buflen) const override {
        if (!_got_model_name) {
            return;
        }
        strncpy(buf, _model_name, buflen);
    }
    // return camera firmware version
    uint32_t get_camera_firmware_version() const override { return _firmware_ver; }
    // return camera capability flags
    uint32_t get_camera_cap_flags() const override {
        return (CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM);
    }

    // send camera settings message to GCS
    void send_camera_settings(mavlink_channel_t chan) const override;

protected:

    // get attitude as a quaternion.  returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override;

    // SkyDroid can send either rates or angles, and also has a dedicated one-shot
    // "center" command for retract/neutral (see send_target_retracted()/
    // send_target_neutral()) rather than falling through to the angle-based
    // conversion every other target type uses
    uint8_t natively_supported_mount_target_types() const override {
        return NATIVE_ANGLES_AND_RATES_ONLY |
               (1U << uint8_t(MountTargetType::RETRACTED)) |
               (1U << uint8_t(MountTargetType::NEUTRAL));
    };

    // move to a "retracted" position: SkyDroid has no separate stow position, so
    // this uses the same one-shot "center" command as send_target_neutral()
    void send_target_retracted() override;

    // move to a neutral (forward-pointing) position using the gimbal's own one-shot
    // "center" command (PTZ data byte 0x05), rather than the angle-based conversion
    // every other target type falls through to (which would drive our own P-controller
    // toward _params.neutral_angles, requiring GAC attitude feedback to converge - see
    // send_target_angles()).  This is deliberately independent of that feedback loop:
    // a switch mapped to RC_TARGETING->NEUTRAL should reliably point the gimbal
    // forward using the gimbal's own logic, not depend on our closed loop ever
    // having received a GAC packet
    void send_target_neutral() override;

private:

    // address (2nd and 3rd bytes of packet)
    // first byte is always U (external control unit, whether connected over
    // UART or UDP - SkyDroid's protocol doesn't distinguish the two at the
    // address-byte level) for our outgoing packets
    enum class AddressByte : uint8_t {
        SYSTEM_AND_IMAGE = 68,      // 'D'
        GIMBAL = 71,                // 'G'
        LENS = 77,                  // 'M'
        UDP = 85,                   // 'U'
    };

    // steps of the 1hz round-robin housekeeping loop in update() - not every value is
    // used every cycle (some requests only fire until answered once), and a few
    // numbers are deliberately left spare for anything added later without needing
    // to renumber the rest
    enum class ReqStep : uint8_t {
        VERSION = 0,            // request_gimbal_version(), until _got_gimbal_version
        TIME_SYNC = 1,          // send_time_sync()
        ATTITUDE_ENABLE = 2,    // request_gimbal_attitude()
        MODEL = 3,              // request_gimbal_model(), until _got_model_name
        SDCARD = 4,             // request_gimbal_sdcard_info()
        ATTITUDE_ACCEPT = 6,    // send_attitude_enable() - note the gap at 5, spare
        NUM_STEPS = 10,         // wraps back to VERSION after this - note the gap at 7-9, spare
    };

    // send text prefix string
    static const char* send_message_prefix;

    // AP_Mount_Backend_TPFrame overrides - see that class for what each means
    void handle_message(const char* msg_id) override;
    uint8_t packetlen_max() const override { return AP_MOUNT_SKYDROID_PACKETLEN_MAX; }
    bool is_valid_address_byte(uint8_t b) const override {
        return b == (uint8_t)AddressByte::UDP || b == (uint8_t)AddressByte::LENS ||
               b == (uint8_t)AddressByte::SYSTEM_AND_IMAGE || b == (uint8_t)AddressByte::GIMBAL;
    }
    uint8_t source_address_byte() const override { return (uint8_t)AddressByte::UDP; }

    // request gimbal to start sending attitude at 10hz
    void request_gimbal_attitude();

    // request gimbal memory card information
    void request_gimbal_sdcard_info();

    // request gimbal version
    void request_gimbal_version();

    // request gimbal model name (e.g. "C11", "C13")
    void request_gimbal_model();

    // send current UTC date/time to the gimbal (TIM command) so it can correctly
    // timestamp photos/videos - the camera has no RTC of its own and defaults to
    // 1970-01-01 without this (confirmed on real hardware).  Resent periodically,
    // same as request_gimbal_attitude()/send_attitude_enable(), both as a guard
    // against UDP packet loss and to recover if the camera reboots independently
    // of the flight controller (also confirmed to happen on real hardware)
    bool send_time_sync();

    // enable the gimbal to receive our attitude (FAE) and send it to us (GAA)
    bool send_attitude_enable();

    // send our current attitude to the gimbal (FAI)
    bool send_attitude_to_gimbal();

    // send angle target in radians to gimbal.  Closes the loop ourselves with a
    // P-controller over the gimbal's own "GAC" attitude feedback, driving GSY/GSP as
    // the rate actuator - there is no absolute-angle command that works on this
    // hardware (see this file's header comment)
    void send_target_angles(const MountAngleTarget& angle_rad) override;

    // send rate target in rad/s to gimbal, directly via GSY/GSP
    void send_target_rates(const MountRateTarget& rate_rads) override;

    // send a single-axis rate command (GSY or GSP) for rate_dps, converted to the
    // wire's signed 8bit LSB units using the real-world calibrated scale (see
    // AP_MOUNT_SKYDROID_AXIS_DPS_PER_LSB).  Caller is responsible for any
    // axis-specific sign compensation (GSY's sign is inverted vs AP_Mount's
    // convention - see send_target_rates())
    void send_axis_rate(const Identifier id, float rate_dps);

    // attitude information analysis of gimbal (response to GAA, arrives as "GAC")
    void gimbal_angle_analyse();

    // gimbal video information analysis
    void gimbal_record_analyse();

    // information analysis of gimbal storage card
    void gimbal_sdcard_analyse();

    // gimbal basic information analysis
    void gimbal_version_analyse();

    // gimbal model name analysis (raw ASCII text, e.g. "C13")
    void gimbal_model_analyse();

    // thin wrappers keeping call sites typed on our own AddressByte rather than
    // the base class's raw uint8_t (which must accommodate every product's own,
    // differently-valued AddressByte enum)
    bool send_fixedlen_packet(AddressByte address, const Identifier id, bool write, uint8_t value) {
        return AP_Mount_Backend_TPFrame::send_fixedlen_packet((uint8_t)address, id, write, value);
    }
    bool send_variablelen_packet(HeaderType header, AddressByte address, const Identifier id, bool write, const uint8_t* databuff, uint8_t databuff_len) {
        return AP_Mount_Backend_TPFrame::send_variablelen_packet(header, (uint8_t)address, id, write, databuff, databuff_len);
    }

    // set gimbal's lock vs follow mode
    // lock should be true if gimbal should maintain an earth-frame target
    // lock is false to follow / maintain a body-frame target
    bool set_gimbal_lock(bool lock);

    // send the gimbal's own one-shot "center" command (PTZ data byte 0x05).  Used by
    // both send_target_retracted() and send_target_neutral() - see their comments
    bool send_center_command();

    // members
    bool _recording;                                            // recording status, tracked locally from commands we've sent
    bool _sdcard_healthy;                                        // true if a memory card is present and OK (received from gimbal)
    bool _last_lock;                                            // last lock mode sent to gimbal
    bool _got_gimbal_version;                                   // true if gimbal's version has been received
    bool _got_model_name;                                       // true if gimbal's model name has been received
    bool _announced_connected;                                  // true once we've told the user the gimbal is connected
    uint32_t _firmware_ver;                                     // firmware version
    char _model_name[8];                                        // gimbal model name (e.g. "C11", "C13"), always null-terminated
    Vector3f _current_angle_rad;                                // current angles in radians received from gimbal (x=roll, y=pitch, z=yaw).  roll is reported by the gimbal's own self-stabilization and is not controllable - see has_roll_control()
    uint32_t _last_current_angle_ms;                            // system time (in milliseconds) that angle information received from the gimbal
    uint32_t _last_req_current_info_ms;                         // system time that this driver last requested current gimbal information
    uint8_t _last_req_step;                                     // 10hz request loop step (different requests are sent at various steps)
};

#endif // HAL_MOUNT_SKYDROID_ENABLED
