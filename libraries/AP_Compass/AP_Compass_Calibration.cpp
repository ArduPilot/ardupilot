#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_InternalError/AP_InternalError.h>

#include "AP_Compass.h"

const extern AP_HAL::HAL& hal;

#if COMPASS_CAL_ENABLED

void Compass::cal_update()
{
    if (hal.util->get_soft_armed()) {
        return;
    }

    bool running = false;

    for (Priority i(0); i<COMPASS_MAX_INSTANCES; i++) {
        if (_calibrator[i] == nullptr) {
            continue;
        }
        if (_calibrator[i]->failed()) {
            AP_Notify::events.compass_cal_failed = 1;
        }

        if (_calibrator[i]->running()) {
            running = true;
        } else if (_cal_autosave && !_cal_saved[i] && _calibrator[i]->get_state().status == CompassCalibrator::Status::SUCCESS) {
            _accept_calibration(uint8_t(i));
        }
    }

    AP_Notify::flags.compass_cal_running = running;

    if (is_calibrating()) {
        _cal_has_run = true;
        return;
    } else if (_cal_has_run && _auto_reboot()) {
        hal.scheduler->delay(1000);
        hal.scheduler->reboot(false);
    }
}

bool Compass::_start_calibration(uint8_t i, bool retry, float delay)
{
    if (!healthy(i)) {
        return false;
    }
    if (!use_for_yaw(i)) {
        return false;
    }
    Priority prio = Priority(i);

#if COMPASS_MAX_INSTANCES > 1
    if (_priority_did_list[prio] != _priority_did_stored_list[prio]) {
        gcs().send_text(MAV_SEVERITY_ERROR, "Compass cal requires reboot after priority change");
        return false;
    }
#endif
    
    if (_calibrator[prio] == nullptr) {
        _calibrator[prio] = new CompassCalibrator();
        if (_calibrator[prio] == nullptr) {
            gcs().send_text(MAV_SEVERITY_ERROR, "Compass cal object not initialised");
            return false;
        }
    }

    if (_options.get() & uint16_t(Option::CAL_REQUIRE_GPS)) {
        if (AP::gps().status() < AP_GPS::GPS_OK_FIX_2D) {
            gcs().send_text(MAV_SEVERITY_ERROR, "Compass cal requires GPS lock");
            return false;
        }
    }
    if (!is_calibrating()) {
        AP_Notify::events.initiated_compass_cal = 1;
    }

    if (_rotate_auto) {
        enum Rotation r = _get_state(prio).external?(enum Rotation)_get_state(prio).orientation.get():ROTATION_NONE;
        if (r < ROTATION_MAX) {
            _calibrator[prio]->set_orientation(r, _get_state(prio).external, _rotate_auto>=2, _rotate_auto>=3);
        }
    }
    _cal_saved[prio] = false;
    if (i == 0 && _get_state(prio).external != 0) {
        _calibrator[prio]->start(retry, delay, get_offsets_max(), i, _calibration_threshold);
    } else {
        // internal compasses or secondary compasses get twice the
        // threshold. This is because internal compasses tend to be a
        // lot noisier
        _calibrator[prio]->start(retry, delay, get_offsets_max(), i, _calibration_threshold*2);
    }
    if (!_cal_thread_started) {
        _cal_requires_reboot = true;
        if (!hal.scheduler->thread_create(FUNCTOR_BIND(this, &Compass::_update_calibration_trampoline, void), "compasscal", 2048, AP_HAL::Scheduler::PRIORITY_IO, 0)) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "CompassCalibrator: Cannot start compass thread.");
            return false;
        }
        _cal_thread_started = true;
    }

    // disable compass learning both for calibration and after completion
    _learn.set_and_save(0);

    return true;
}

void Compass::_update_calibration_trampoline() {
    while(true) {
        for (Priority i(0); i<COMPASS_MAX_INSTANCES; i++) {
            if (_calibrator[i] == nullptr) {
                continue;
            }
            _calibrator[i]->update();
        }
        hal.scheduler->delay(1);
    }
}

bool Compass::_start_calibration_mask(uint8_t mask, bool retry, bool autosave, float delay, bool autoreboot)
{
    _cal_autosave = autosave;
    _compass_cal_autoreboot = autoreboot;

    bool at_least_one_started = false;
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if ((1<<i) & mask) {
            if (!_start_calibration(i,retry,delay)) {
                _cancel_calibration_mask(mask);
                return false;
            }
            at_least_one_started = true;
        }
    }
    return at_least_one_started;
}

bool Compass::start_calibration_all(bool retry, bool autosave, float delay, bool autoreboot)
{
    _cal_autosave = autosave;
    _compass_cal_autoreboot = autoreboot;

    bool at_least_one_started = false;
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        // ignore any compasses that fail to start calibrating
        // start all should only calibrate compasses that are being used
        if (_start_calibration(i,retry,delay)) {
            at_least_one_started = true;
        }
    }
    return at_least_one_started;
}

void Compass::_cancel_calibration(uint8_t i)
{
    AP_Notify::events.initiated_compass_cal = 0;
    Priority prio = Priority(i);
    if (_calibrator[prio] == nullptr) {
        return;
    }
    if (_calibrator[prio]->running() || _calibrator[prio]->get_state().status == CompassCalibrator::Status::WAITING_TO_START) {
        AP_Notify::events.compass_cal_canceled = 1;
    }
    _cal_saved[prio] = false;
    _calibrator[prio]->stop();
}

void Compass::_cancel_calibration_mask(uint8_t mask)
{
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if ((1<<i) & mask) {
            _cancel_calibration(i);
        }
    }
}

void Compass::cancel_calibration_all()
{
    _cancel_calibration_mask(0xFF);
}

bool Compass::_accept_calibration(uint8_t i)
{
    Priority prio = Priority(i);

    CompassCalibrator* cal = _calibrator[prio];
    if (cal == nullptr) {
        return false;
    }
    const CompassCalibrator::Report cal_report = cal->get_report();

    if (_cal_saved[prio] || cal_report.status == CompassCalibrator::Status::NOT_STARTED) {
        return true;
    } else if (cal_report.status == CompassCalibrator::Status::SUCCESS) {
        _cal_saved[prio] = true;

        Vector3f ofs(cal_report.ofs), diag(cal_report.diag), offdiag(cal_report.offdiag);
        float scale_factor = cal_report.scale_factor;

        set_and_save_offsets(i, ofs);
#if AP_COMPASS_DIAGONALS_ENABLED
        set_and_save_diagonals(i,diag);
        set_and_save_offdiagonals(i,offdiag);
#endif
        set_and_save_scale_factor(i,scale_factor);

        if (cal_report.check_orientation && _get_state(prio).external && _rotate_auto >= 2) {
            set_and_save_orientation(i, cal_report.orientation);
        }

        if (!is_calibrating()) {
            AP_Notify::events.compass_cal_saved = 1;
        }
        return true;
    } else {
        return false;
    }
}

bool Compass::_accept_calibration_mask(uint8_t mask)
{
    bool success = true;
    for (Priority i(0); i<COMPASS_MAX_INSTANCES; i++) {
        if (_calibrator[i] == nullptr) {
            continue;
        }
        if ((1<<uint8_t(i)) & mask) {
            if (!_accept_calibration(uint8_t(i))) {
                success = false;
            }
            _calibrator[i]->stop();
        }
    }

    return success;
}

bool Compass::send_mag_cal_progress(const GCS_MAVLINK& link)
{
    const mavlink_channel_t chan = link.get_chan();

    for (uint8_t i = 0; i < COMPASS_MAX_INSTANCES; i++) {
        const Priority compass_id = (next_cal_progress_idx[chan] + 1) % COMPASS_MAX_INSTANCES;
        
        auto& calibrator = _calibrator[compass_id];
        if (calibrator == nullptr) {
            next_cal_progress_idx[chan] = compass_id;
            continue;
        }
        const CompassCalibrator::State cal_state = calibrator->get_state();

        if (cal_state.status == CompassCalibrator::Status::WAITING_TO_START  ||
            cal_state.status == CompassCalibrator::Status::RUNNING_STEP_ONE ||
            cal_state.status == CompassCalibrator::Status::RUNNING_STEP_TWO) {
            // ensure we don't try to send with no space available
            if (!HAVE_PAYLOAD_SPACE(chan, MAG_CAL_PROGRESS)) {
                return false;
            }

            next_cal_progress_idx[chan] = compass_id;

            mavlink_msg_mag_cal_progress_send(
                link.get_chan(),
                uint8_t(compass_id),
                _get_cal_mask(),
                (uint8_t)cal_state.status, cal_state.attempt, cal_state.completion_pct, cal_state.completion_mask,
                0.0f, 0.0f, 0.0f
            );
        } else {
            next_cal_progress_idx[chan] = compass_id;
        }
    }

    return true;
}

bool Compass::send_mag_cal_report(const GCS_MAVLINK& link)
{
    const mavlink_channel_t chan = link.get_chan();

    for (uint8_t i = 0; i < COMPASS_MAX_INSTANCES; i++) {
        const Priority compass_id = (next_cal_report_idx[chan] + 1) % COMPASS_MAX_INSTANCES;

        if (_calibrator[compass_id] == nullptr) {
            next_cal_report_idx[chan] = compass_id;
            continue;
        }
        const CompassCalibrator::Report cal_report = _calibrator[compass_id]->get_report();
        switch (cal_report.status) {
        case CompassCalibrator::Status::NOT_STARTED:
        case CompassCalibrator::Status::WAITING_TO_START:
        case CompassCalibrator::Status::RUNNING_STEP_ONE:
        case CompassCalibrator::Status::RUNNING_STEP_TWO:
            // calibration has not finished ergo no report
            next_cal_report_idx[chan] = compass_id;
            continue;
        case CompassCalibrator::Status::SUCCESS:
        case CompassCalibrator::Status::FAILED:
        case CompassCalibrator::Status::BAD_ORIENTATION:
        case CompassCalibrator::Status::BAD_RADIUS:
            // ensure we don't try to send with no space available
            if (!HAVE_PAYLOAD_SPACE(chan, MAG_CAL_REPORT)) {
                return false;
            }

            next_cal_report_idx[chan] = compass_id;

            mavlink_msg_mag_cal_report_send(
                link.get_chan(),
                uint8_t(compass_id),
                _get_cal_mask(),
                (uint8_t)cal_report.status,
                _cal_saved[compass_id],
                cal_report.fitness,
                cal_report.ofs.x, cal_report.ofs.y, cal_report.ofs.z,
                cal_report.diag.x, cal_report.diag.y, cal_report.diag.z,
                cal_report.offdiag.x, cal_report.offdiag.y, cal_report.offdiag.z,
                cal_report.orientation_confidence,
                cal_report.original_orientation,
                cal_report.orientation,
                cal_report.scale_factor
            );
        }
    }
    return true;
}

bool Compass::is_calibrating() const
{
    for (Priority i(0); i<COMPASS_MAX_INSTANCES; i++) {
        if (_calibrator[i] == nullptr) {
            continue;
        }
        switch(_calibrator[i]->get_state().status) {
            case CompassCalibrator::Status::NOT_STARTED:
            case CompassCalibrator::Status::SUCCESS:
            case CompassCalibrator::Status::FAILED:
            case CompassCalibrator::Status::BAD_ORIENTATION:
            case CompassCalibrator::Status::BAD_RADIUS:
                // this backend isn't calibrating,
                // but maybe the next one is:
                continue;
            case CompassCalibrator::Status::WAITING_TO_START:
            case CompassCalibrator::Status::RUNNING_STEP_ONE:
            case CompassCalibrator::Status::RUNNING_STEP_TWO:
                return true;
        }
    }
    return false;
}

uint8_t Compass::_get_cal_mask()
{
    uint8_t cal_mask = 0;
    for (Priority i(0); i<COMPASS_MAX_INSTANCES; i++) {
        if (_calibrator[i] == nullptr) {
            continue;
        }
        if (_calibrator[i]->get_state().status != CompassCalibrator::Status::NOT_STARTED) {
            cal_mask |= 1 << uint8_t(i);
        }
    }
    return cal_mask;
}

/*
  handle an incoming MAG_CAL command
 */
MAV_RESULT Compass::handle_mag_cal_command(const mavlink_command_long_t &packet)
{
    MAV_RESULT result = MAV_RESULT_FAILED;

    switch (packet.command) {
    case MAV_CMD_DO_START_MAG_CAL: {
        result = MAV_RESULT_ACCEPTED;
        if (hal.util->get_soft_armed()) {
            gcs().send_text(MAV_SEVERITY_NOTICE, "Disarm to allow compass calibration");
            result = MAV_RESULT_FAILED;
            break;
        }
        if (packet.param1 < 0 || packet.param1 > 255) {
            result = MAV_RESULT_FAILED;
            break;
        }

        uint8_t mag_mask = packet.param1;
        bool retry = !is_zero(packet.param2);
        bool autosave = !is_zero(packet.param3);
        float delay = packet.param4;
        bool autoreboot = !is_zero(packet.param5);

        if (mag_mask == 0) { // 0 means all
            _reset_compass_id();
            if (!start_calibration_all(retry, autosave, delay, autoreboot)) {
                result = MAV_RESULT_FAILED;
            }
        } else {
            if (!_start_calibration_mask(mag_mask, retry, autosave, delay, autoreboot)) {
                result = MAV_RESULT_FAILED;
            }
        }

        break;
    }

    case MAV_CMD_DO_ACCEPT_MAG_CAL: {
        result = MAV_RESULT_ACCEPTED;
        if (packet.param1 < 0 || packet.param1 > 255) {
            result = MAV_RESULT_FAILED;
            break;
        }

        uint8_t mag_mask = packet.param1;

        if (mag_mask == 0) { // 0 means all
            mag_mask = 0xFF;
        }

        if (!_accept_calibration_mask(mag_mask)) {
            result = MAV_RESULT_FAILED;
        }
        break;
    }

    case MAV_CMD_DO_CANCEL_MAG_CAL: {
        result = MAV_RESULT_ACCEPTED;
        if (packet.param1 < 0 || packet.param1 > 255) {
            result = MAV_RESULT_FAILED;
            break;
        }
        
        uint8_t mag_mask = packet.param1;
        
        if (mag_mask == 0) { // 0 means all
            cancel_calibration_all();
            break;
        }
        
        _cancel_calibration_mask(mag_mask);
        break;
    }
    }
    
    return result;
}

/*
  get mag field with the effects of offsets, diagonals and
  off-diagonals removed
 */
bool Compass::get_uncorrected_field(uint8_t instance, Vector3f &field) const
{
    // get corrected field
    field = get_field(instance);

#if AP_COMPASS_DIAGONALS_ENABLED
    // form eliptical correction matrix and invert it. This is
    // needed to remove the effects of the eliptical correction
    // when calculating new offsets
    const Vector3f &diagonals = get_diagonals(instance);
    const Vector3f &offdiagonals = get_offdiagonals(instance);
    Matrix3f mat {
        diagonals.x, offdiagonals.x, offdiagonals.y,
        offdiagonals.x,    diagonals.y, offdiagonals.z,
        offdiagonals.y, offdiagonals.z,    diagonals.z
    };
    if (!mat.invert()) {
        return false;
    }

    // remove impact of diagonals and off-diagonals
    field = mat * field;
#endif

    // remove impact of offsets
    field -= get_offsets(instance);

    return true;
}

/*
  fast compass calibration given vehicle position and yaw. This
  results in zero diagonal and off-diagonal elements, so is only
  suitable for vehicles where the field is close to spherical. It is
  useful for large vehicles where moving the vehicle to calibrate it
  is difficult.

  The offsets of the selected compasses are set to values to bring
  them into consistency with the WMM tables at the given latitude and
  longitude. If compass_mask is zero then all enabled compasses are
  calibrated.

  This assumes that the compass is correctly scaled in milliGauss
*/
MAV_RESULT Compass::mag_cal_fixed_yaw(float yaw_deg, uint8_t compass_mask,
                                      float lat_deg, float lon_deg, bool force_use)
{
    _reset_compass_id();
    if (is_zero(lat_deg) && is_zero(lon_deg)) {
        Location loc;
        // get AHRS position. If unavailable then try GPS location
        if (!AP::ahrs().get_location(loc)) {
            if (AP::gps().status() < AP_GPS::GPS_OK_FIX_3D) {
                gcs().send_text(MAV_SEVERITY_ERROR, "Mag: no position available");
                return MAV_RESULT_FAILED;
            }
            loc = AP::gps().location();
        }
        lat_deg = loc.lat * 1.0e-7;
        lon_deg = loc.lng * 1.0e-7;
    }

    // get the magnetic field intensity and orientation
    float intensity;
    float declination;
    float inclination;
    if (!AP_Declination::get_mag_field_ef(lat_deg, lon_deg, intensity, declination, inclination)) {
        gcs().send_text(MAV_SEVERITY_ERROR, "Mag: WMM table error");
        return MAV_RESULT_FAILED;
    }

    // create a field vector and rotate to the required orientation
    Vector3f field(1e3f * intensity, 0.0f, 0.0f);
    Matrix3f R;
    R.from_euler(0.0f, -ToRad(inclination), ToRad(declination));
    field = R * field;

    Matrix3f dcm;
    dcm.from_euler(AP::ahrs().roll, AP::ahrs().pitch, radians(yaw_deg));

    // Rotate into body frame using provided yaw
    field = dcm.transposed() * field;

    for (uint8_t i=0; i<get_count(); i++) {
        if (compass_mask != 0 && ((1U<<i) & compass_mask) == 0) {
            // skip this compass
            continue;
        }
        if (_use_for_yaw[Priority(i)] == 0 || (!force_use && !use_for_yaw(i))) {
            continue;
        }
        if (!healthy(i)) {
            gcs().send_text(MAV_SEVERITY_ERROR, "Mag[%u]: unhealthy\n", i);
            return MAV_RESULT_FAILED;
        }

        Vector3f measurement;
        if (!get_uncorrected_field(i, measurement)) {
            gcs().send_text(MAV_SEVERITY_ERROR, "Mag[%u]: bad uncorrected field", i);
            return MAV_RESULT_FAILED;
        }

        Vector3f offsets = field - measurement;
        set_and_save_offsets(i, offsets);
#if AP_COMPASS_DIAGONALS_ENABLED
        Vector3f one{1,1,1};
        set_and_save_diagonals(i, one);
        Vector3f zero{0,0,0};
        set_and_save_offdiagonals(i, zero);
#endif
    }

    return MAV_RESULT_ACCEPTED;
}


#endif // COMPASS_CAL_ENABLED
