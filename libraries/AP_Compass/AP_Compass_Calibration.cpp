/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include "Compass.h"
#include <AP_Notify.h>

extern AP_HAL::HAL& hal;

void
Compass::compass_cal_update()
{
    AP_Notify::flags.compass_cal_running = 0;

    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        bool failure;
        _calibrator[i].update(failure);
        if (failure) {
            AP_Notify::events.compass_cal_failed = 1;
        }

        if (_calibrator[i].check_for_timeout()) {
            AP_Notify::events.compass_cal_failed = 1;
            cancel_calibration_all();
        }

        if (_calibrator[i].running()) {
            AP_Notify::flags.compass_cal_running = 1;
        }
    }
}

bool
Compass::start_calibration(uint8_t i, bool retry, bool autosave, float delay)
{
    if (healthy(i)) {
        if (!is_calibrating() && delay > 0.5f) {
            AP_Notify::events.initiated_compass_cal = 1;
        }
        if (i == get_primary()) {
            _calibrator[i].set_tolerance(8);
        } else {
            _calibrator[i].set_tolerance(16);
        }
        _calibrator[i].start(retry, autosave, delay);
        return true;
    } else {
        return false;
    }
}

bool
Compass::start_calibration_mask(uint8_t mask, bool retry, bool autosave, float delay)
{
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if ((1<<i) & mask) {
            if (!start_calibration(i,retry,autosave,delay)) {
                cancel_calibration_mask(mask);
                return false;
            }
        }
    }
    return true;
}

bool
Compass::start_calibration_all(bool retry, bool autosave, float delay)
{
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if (healthy(i) && use_for_yaw(i)) {
            if (!start_calibration(i,retry,autosave,delay)) {
                cancel_calibration_all();
                return false;
            }
        }
    }
    return true;
}

void
Compass::cancel_calibration(uint8_t i)
{
    _calibrator[i].clear();
    AP_Notify::events.compass_cal_canceled = 1;
    AP_Notify::events.initiated_compass_cal = 0;
}

void
Compass::cancel_calibration_mask(uint8_t mask)
{
    for(uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if((1<<i) & mask) {
            cancel_calibration(i);
        }
    }
}

void
Compass::cancel_calibration_all()
{
    cancel_calibration_mask(0xFF);
}

bool
Compass::accept_calibration(uint8_t i)
{
    CompassCalibrator& cal = _calibrator[i];
    uint8_t cal_status = cal.get_status();

    if (cal_status == COMPASS_CAL_SUCCESS) {
        Vector3f ofs, diag, offdiag;
        cal.get_calibration(ofs, diag, offdiag);
        cal.clear();

        set_and_save_offsets(i, ofs);
        set_and_save_diagonals(i,diag);
        set_and_save_offdiagonals(i,offdiag);

        if (!is_calibrating()) {
            AP_Notify::events.compass_cal_saved = 1;
        }
        hal.scheduler->delay(1000);
        hal.scheduler->reboot(false);
        return true;
    } else {
        return false;
    }
}

bool
Compass::accept_calibration_mask(uint8_t mask)
{
    for(uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if ((1<<i) & mask) {
            CompassCalibrator& cal = _calibrator[i];
            uint8_t cal_status = cal.get_status();
            if (cal_status != COMPASS_CAL_SUCCESS && cal_status != COMPASS_CAL_NOT_STARTED) {
                // a compass failed or is still in progress
                return false;
            }
        }
    }

    bool success = true;
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if ((1<<i) & mask) {
            if (!accept_calibration(i)) {
                success = false;
            }
        }
    }

    return success;
}

bool
Compass::accept_calibration_all()
{
    return accept_calibration_mask(0xFF);
}

void
Compass::send_mag_cal_progress(mavlink_channel_t chan)
{
    uint8_t cal_mask = get_cal_mask();

    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        CompassCalibrator& cal = _calibrator[i];

        uint8_t& compass_id = i;
        uint8_t cal_status = cal.get_status();

        if (cal_status == COMPASS_CAL_WAITING_TO_START  ||
            cal_status == COMPASS_CAL_RUNNING_STEP_ONE ||
            cal_status == COMPASS_CAL_RUNNING_STEP_TWO) {
            uint8_t completion_pct = cal.get_completion_percent();
            uint8_t completion_mask[10];
            Vector3f direction(0.0f,0.0f,0.0f);
            uint8_t attempt = cal.get_attempt();

            memset(completion_mask, 0, sizeof(completion_mask));

            mavlink_msg_mag_cal_progress_send(
                chan,
                compass_id, cal_mask,
                cal_status, attempt, completion_pct, completion_mask,
                direction.x, direction.y, direction.z
            );
        }
    }
}

void Compass::send_mag_cal_report(mavlink_channel_t chan)
{
    uint8_t cal_mask = get_cal_mask();

    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
         CompassCalibrator& cal = _calibrator[i];

        uint8_t& compass_id = i;
        uint8_t cal_status = cal.get_status();

        if (cal_status == COMPASS_CAL_SUCCESS ||
            cal_status == COMPASS_CAL_FAILED) {
            float fitness = cal.get_fitness();
            Vector3f ofs, diag, offdiag;
            cal.get_calibration(ofs, diag, offdiag);
            uint8_t autosaved = cal.get_autosave();

            mavlink_msg_mag_cal_report_send(
                chan,
                compass_id, cal_mask,
                cal_status, autosaved,
                fitness,
                ofs.x, ofs.y, ofs.z,
                diag.x, diag.y, diag.z,
                offdiag.x, offdiag.y, offdiag.z
            );
        }

        if (cal_status == COMPASS_CAL_SUCCESS && cal.get_autosave()) {
            accept_calibration(i);
        }
    }
}

bool
Compass::is_calibrating() const
{
    return get_cal_mask();
}

uint8_t
Compass::get_cal_mask() const
{
    uint8_t cal_mask = 0;
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if (_calibrator[i].get_status() != COMPASS_CAL_NOT_STARTED) {
            cal_mask |= 1 << i;
        }
    }
    return cal_mask;
}
