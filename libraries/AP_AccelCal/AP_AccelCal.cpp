/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "AP_AccelCal.h"

#if HAL_INS_ACCELCAL_ENABLED

#include <stdarg.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

#define AP_ACCELCAL_POSITION_REQUEST_INTERVAL_MS 1000

#define _printf(fmt, args ...) do {                                     \
        if (_gcs != nullptr) {                                          \
            _gcs->send_text(MAV_SEVERITY_CRITICAL, fmt, ## args);       \
        }                                                               \
    } while (0)


const extern AP_HAL::HAL& hal;
static bool _start_collect_sample;

uint8_t AP_AccelCal::_num_clients = 0;
AP_AccelCal_Client* AP_AccelCal::_clients[AP_ACCELCAL_MAX_NUM_CLIENTS] {};

void AP_AccelCal::update()
{
    if (!get_calibrator(0)) {
        // no calibrators
        return;
    }

    if (_started) {
        update_status();

        uint8_t num_active_calibrators = 0;
        for(uint8_t i=0; get_calibrator(i) != nullptr; i++) {
            num_active_calibrators++;
        }
        if (num_active_calibrators != _num_active_calibrators) {
            fail();
            return;
        }
        if(_start_collect_sample) {
            collect_sample();
        }
        AccelCalibrator *cal;
        switch(_status) {
            case ACCEL_CAL_NOT_STARTED:
                fail();
                return;
            case ACCEL_CAL_WAITING_FOR_ORIENTATION: {
                // if we're waiting for orientation, first ensure that all calibrators are on the same step
                uint8_t step;
                if ((cal = get_calibrator(0)) == nullptr) {
                    fail();
                    return;
                }
                step = cal->get_num_samples_collected()+1;

                for(uint8_t i=1 ; (cal = get_calibrator(i))  ; i++) {
                    if (step != cal->get_num_samples_collected()+1) {
                        fail();
                        return;
                    }
                }
                // if we're on a new step, print a message describing the step
                if (step != _step) {
                    _step = step;

                    if(_use_gcs_snoop) {
                        const char *msg;
                        switch (step) {
                            case ACCELCAL_VEHICLE_POS_LEVEL:
                                msg = "level";
                                break;
                            case ACCELCAL_VEHICLE_POS_LEFT:
                                msg = "on its LEFT side";
                                break;
                            case ACCELCAL_VEHICLE_POS_RIGHT:
                                msg = "on its RIGHT side";
                                break;
                            case ACCELCAL_VEHICLE_POS_NOSEDOWN:
                                msg = "nose DOWN";
                                break;
                            case ACCELCAL_VEHICLE_POS_NOSEUP:
                                msg = "nose UP";
                                break;
                            case ACCELCAL_VEHICLE_POS_BACK:
                                msg = "on its BACK";
                                break;
                            default:
                                fail();
                                return;
                        }
                        _printf("Place vehicle %s and press any key.", msg);
                        _waiting_for_mavlink_ack = true;
                    }
                }

                uint32_t now = AP_HAL::millis();
                if (now - _last_position_request_ms > AP_ACCELCAL_POSITION_REQUEST_INTERVAL_MS) {
                    _last_position_request_ms = now;
                    _gcs->send_accelcal_vehicle_position(step);
                }
                break;
            }
            case ACCEL_CAL_COLLECTING_SAMPLE:
                // check for timeout

                for(uint8_t i=0; (cal = get_calibrator(i)); i++) {
                    cal->check_for_timeout();
                }

                update_status();

                if (_status == ACCEL_CAL_FAILED) {
                    fail();
                }
                return;
            case ACCEL_CAL_SUCCESS:
                // save
                if (_saving) {
                    bool done = true;
                    for(uint8_t i=0; i<_num_clients; i++) {
                        if (client_active(i) && _clients[i]->_acal_get_saving()) {
                            done = false;
                            break;
                        }
                    }
                    if (done) {
                        success();
                    }
                    return;
                } else {
                    for(uint8_t i=0; i<_num_clients; i++) {
                        if(client_active(i) && _clients[i]->_acal_get_fail()) {
                            fail();
                            return;
                        }
                    }
                    for(uint8_t i=0; i<_num_clients; i++) {
                        if(client_active(i)) {
                            _clients[i]->_acal_save_calibrations();
                        }
                    }
                    _saving = true;
                }
                return;
            default:
            case ACCEL_CAL_FAILED:
                fail();
                return;
        }
    } else if (_last_result != ACCEL_CAL_NOT_STARTED) {
        // only continuously report if we have ever completed a calibration
        uint32_t now = AP_HAL::millis();
        if (now - _last_position_request_ms > AP_ACCELCAL_POSITION_REQUEST_INTERVAL_MS) {
            _last_position_request_ms = now;
            switch (_last_result) {
                case ACCEL_CAL_SUCCESS:
                    _gcs->send_accelcal_vehicle_position(ACCELCAL_VEHICLE_POS_SUCCESS);
                    break;
                case ACCEL_CAL_FAILED:
                    _gcs->send_accelcal_vehicle_position(ACCELCAL_VEHICLE_POS_FAILED);
                    break;
                default:
                    // should never hit this state
                    break;
            }
        }
    }
}

void AP_AccelCal::start(GCS_MAVLINK *gcs)
{
    if (gcs == nullptr || _started) {
        return;
    }
    _start_collect_sample = false;
    _num_active_calibrators = 0;

    AccelCalibrator *cal;
    for(uint8_t i=0; (cal = get_calibrator(i)); i++) {
        cal->clear();
        cal->start(ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID, 6, 0.5f);
        _num_active_calibrators++;
    }

    _started = true;
    _saving = false;
    _gcs = gcs;
    _use_gcs_snoop = true;
    _last_position_request_ms = 0;
    _step = 0;

    _last_result = ACCEL_CAL_NOT_STARTED;

    update_status();
}

void AP_AccelCal::success()
{
    _printf("Calibration successful");

    for(uint8_t i=0 ; i < _num_clients ; i++) {
        _clients[i]->_acal_event_success();
    }

    _last_result = ACCEL_CAL_SUCCESS;

    clear();
}

void AP_AccelCal::cancel()
{
    _printf("Calibration cancelled");

    for(uint8_t i=0 ; i < _num_clients ; i++) {
        _clients[i]->_acal_event_cancellation();
    }

    _last_result = ACCEL_CAL_NOT_STARTED;

    clear();
}

void AP_AccelCal::fail()
{
    _printf("Calibration FAILED");

    for(uint8_t i=0 ; i < _num_clients ; i++) {
        _clients[i]->_acal_event_failure();
    }

    _last_result = ACCEL_CAL_FAILED;

    clear();
}

void AP_AccelCal::clear()
{
    if (!_started) {
        return;
    }

    AccelCalibrator *cal;
    for(uint8_t i=0 ; (cal = get_calibrator(i))  ; i++) {
        cal->clear();
    }

    _step = 0;
    _started = false;
    _saving = false;

    update_status();
}

void AP_AccelCal::collect_sample()
{
    if (_status != ACCEL_CAL_WAITING_FOR_ORIENTATION) {
        return;
    }

    for(uint8_t i=0; i<_num_clients; i++) {
        if (client_active(i) && !_clients[i]->_acal_get_ready_to_sample()) {
            _printf("Not ready to sample");
            return;
        }
    }

    AccelCalibrator *cal;
    for(uint8_t i=0 ; (cal = get_calibrator(i))  ; i++) {
        cal->collect_sample();
    }
    _start_collect_sample = false;
    update_status();
}

void AP_AccelCal::register_client(AP_AccelCal_Client* client) {
    if (client == nullptr || _num_clients >= AP_ACCELCAL_MAX_NUM_CLIENTS) {
        return;
    }


    for(uint8_t i=0; i<_num_clients; i++) {
        if(_clients[i] == client) {
            return;
        }
    }
    _clients[_num_clients] = client;
    _num_clients++;
}

AccelCalibrator* AP_AccelCal::get_calibrator(uint8_t index) {
    AccelCalibrator* ret;
    for(uint8_t i=0; i<_num_clients; i++) {
        for(uint8_t j=0 ; (ret = _clients[i]->_acal_get_calibrator(j)) ; j++) {
            if (index == 0) {
                return ret;
            }
            index--;
        }
    }
    return nullptr;
}

void AP_AccelCal::update_status() {
    AccelCalibrator *cal;

    if (!get_calibrator(0)) {
        // no calibrators
        _status = ACCEL_CAL_NOT_STARTED;
        return;
    }

    for(uint8_t i=0 ; (cal = get_calibrator(i))  ; i++) {
        if (cal->get_status() == ACCEL_CAL_FAILED) {
            _status = ACCEL_CAL_FAILED;         //fail if even one of the calibration has
            return;
        }
    }

    for(uint8_t i=0 ; (cal = get_calibrator(i))  ; i++) {
        if (cal->get_status() == ACCEL_CAL_COLLECTING_SAMPLE) {
            _status = ACCEL_CAL_COLLECTING_SAMPLE;          // move to Collecting sample state if all the callibrators have
            return;
        }
    }

    for(uint8_t i=0 ; (cal = get_calibrator(i))  ; i++) {
        if (cal->get_status() == ACCEL_CAL_WAITING_FOR_ORIENTATION) {
            _status = ACCEL_CAL_WAITING_FOR_ORIENTATION;    // move to waiting for user ack for orientation confirmation
            return;
        }
    }

    for(uint8_t i=0 ; (cal = get_calibrator(i))  ; i++) {
        if (cal->get_status() == ACCEL_CAL_NOT_STARTED) {
            _status = ACCEL_CAL_NOT_STARTED;    // we haven't started if all the calibrators haven't
            return;
        }
    }

    _status = ACCEL_CAL_SUCCESS;    // we have succeeded calibration if all the calibrators have
    return;
}

bool AP_AccelCal::client_active(uint8_t client_num)
{
    return (bool)_clients[client_num]->_acal_get_calibrator(0);
}

#if HAL_GCS_ENABLED
void AP_AccelCal::handle_command_ack(const mavlink_command_ack_t &packet)
{
    if (!_waiting_for_mavlink_ack) {
        return;
    }
    // this is support for the old, non-accelcal-specific calibration.
    // The GCS is expected to send back a COMMAND_ACK when the vehicle
    // is posed, but we placed no constraints on the result code or
    // the command field in the ack packet.  That meant that any ACK
    // would move the cal process forward - and since we don't even
    // check the source system/component here the process could easily
    // fail due to other ACKs floating around the mavlink network.
    // GCSs should be moved to using the non-gcs-snoop method.  As a
    // round-up:
    // MAVProxy: command=1-6 depending on pose, result=1
    // QGC: command=0, result=1
    // MissionPlanner: uses new ACCELCAL_VEHICLE_POS
    if (packet.command > 6) {
        // not an acknowledgement for a vehicle position
        return;
    }
    if (packet.result != MAV_RESULT_TEMPORARILY_REJECTED) {
        // not an acknowledgement for a vehicle position
        return;
    }
    _waiting_for_mavlink_ack = false;
    _start_collect_sample = true;
}

bool AP_AccelCal::gcs_vehicle_position(float position)
{
    _use_gcs_snoop = false;

    if (_status == ACCEL_CAL_WAITING_FOR_ORIENTATION && is_equal((float) _step, position)) {
        _start_collect_sample = true;
        return true;
    }

    return false;
}
#endif

// true if we are in a calibration process
bool AP_AccelCal::running(void) const
{
    return _status == ACCEL_CAL_WAITING_FOR_ORIENTATION || _status == ACCEL_CAL_COLLECTING_SAMPLE;
}
#endif //HAL_INS_ACCELCAL_ENABLED
