/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
#include <stdarg.h>
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_HAL/AP_HAL.h>

const extern AP_HAL::HAL& hal;
static bool _start_collect_sample;
static void _snoop(const mavlink_message_t* msg);

// Run an iteration of all registered calibrations
void AP_AccelCal::update()
{
    if (!get_calibrator(0)) {
        // no calibrators
        return;
    }

    if (_started) {
        update_status();

        AccelCalibrator *cal;
        uint8_t num_active_calibrators = 0;
        for(uint8_t i=0; (cal = get_calibrator(i)); i++) {
            num_active_calibrators++;
        }
        if (num_active_calibrators != _num_active_calibrators) {
            fail();
            return;
        }
        if(_start_collect_sample) {
            collect_sample();
            _gcs->set_snoop(NULL);
            _start_collect_sample = false;
        }
        switch(_status) {
            case ACCEL_CAL_NOT_STARTED:
                fail();
                return;
            case ACCEL_CAL_WAITING_FOR_ORIENTATION: {
                // if we're waiting for orientation, first ensure that all calibrators are on the same step
                uint8_t step;
                cal = get_calibrator(0);
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

                    const char *msg;
                    switch (step) {
                        case 1:
                            msg = "level";
                            break;
                        case 2:
                            msg = "on its LEFT side";
                            break;
                        case 3:
                            msg = "on its RIGHT side";
                            break;
                        case 4:
                            msg = "nose DOWN";
                            break;
                        case 5:
                            msg = "nose UP";
                            break;
                        case 6:
                            msg = "on its BACK";
                            break;
                        default:
                            fail();
                            return;
                    }
                    _printf("Place vehicle %s and press any key.", msg);
                }
                // setup snooping of packets so we can see the COMMAND_ACK
                _gcs->set_snoop(_snoop);
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
                        if (client_active(i) && _clients[i]->_acal_saving()) {
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
                        if(client_active(i) && _clients[i]->_acal_failed()) {
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
    }
}

// start all the registered calibrations
void AP_AccelCal::start(GCS_MAVLINK *gcs)
{
    if (gcs == NULL || _started) {
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
    _step = 0;

    update_status();
}

// interface to the user in case all the calibrators succeed
void AP_AccelCal::success()
{
    AccelCalibrator *cal;
    for(uint8_t i=0; (cal = get_calibrator(i)); i++) {
        _printf("Calibration of acc %d successful with fitness %f\n",i,cal->get_fitness());
    }
    clear();
}

// interface to the user in case of the faillure to even a single calibrator
// TODO: return reason of faillure for each calibrator and make distinction
//       b/w passed and failed calibration
void AP_AccelCal::fail()
{
    AccelCalibrator *cal;
    for(uint8_t i=0; (cal = get_calibrator(i)); i++) {
        _printf("Calibration of acc %d FAILED with fitness %f\n",i,cal->get_fitness());
    }
    clear();
}

// reset all the calibrators to there pre calibration stage so as to make them ready for next calibration request
void AP_AccelCal::clear()
{
    if (!_started) {
        return;
    }

    for(uint8_t i=0 ; i < _num_clients ; i++) {
        _clients[i]->_acal_cancelled();
    }

    AccelCalibrator *cal;
    for(uint8_t i=0 ; (cal = get_calibrator(i))  ; i++) {
        cal->clear();
    }

    _gcs = NULL;

    _step = 0;
    _started = false;
    _saving = false;

    update_status();
}

// proceed through the collection step for each of the registered calibrators
void AP_AccelCal::collect_sample()
{
    if (_status != ACCEL_CAL_WAITING_FOR_ORIENTATION) {
        return;
    }

    for(uint8_t i=0; i<_num_clients; i++) {
        if (client_active(i) && !_clients[i]->_acal_ready_to_sample()) {
            _printf("Not ready to sample");
            return;
        }
    }

    AccelCalibrator *cal;
    for(uint8_t i=0 ; (cal = get_calibrator(i))  ; i++) {
        cal->collect_sample();
    }
    // setup snooping of packets so we can see the COMMAND_ACK
    _gcs->set_snoop(NULL);
    update_status();
}

// interface to the clients for registration
void AP_AccelCal::register_client(AP_AccelCal_Client* client) {
    if (client == NULL || _num_clients == AP_ACCELCAL_MAX_NUM_CLIENTS) {
        return;
    }

    if (_started) {
        fail();
    }

    for(uint8_t i=0; i<_num_clients; i++) {
        if(_clients[i] == client) {
            return;
        }
    }
    _clients[_num_clients] = client;
    _num_clients++;
}

// get the calibrator associated with the ID, ID is given to clients in the order of there
// registration
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
    return NULL;
}

// update the state of the Accel calibrator server
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

// check if client's calibrator is active
bool AP_AccelCal::client_active(uint8_t client_num)
{
    return (bool)_clients[client_num]->_acal_get_calibrator(0);
}

/*
  watch for COMMAND_ACK messages
 */
static void _snoop(const mavlink_message_t* msg)
{
    if (msg->msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
        _start_collect_sample = true;
    }
}

void AP_AccelCal::_printf(const char* fmt, ...)
{
    if (!_gcs) {
        return;
    }
    char msg[50];
    va_list ap;
    va_start(ap, fmt);
    hal.util->vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    if (msg[strlen(msg)-1] == '\n') {
        // STATUSTEXT messages should not add linefeed
        msg[strlen(msg)-1] = 0;
    }
    AP_HAL::UARTDriver *uart = _gcs->get_uart();
    /*
     *     to ensure these messages get to the user we need to wait for the
     *     port send buffer to have enough room
     */
    while (uart->txspace() < MAVLINK_NUM_NON_PAYLOAD_BYTES+MAVLINK_MSG_ID_STATUSTEXT_LEN) {
        hal.scheduler->delay(1);
    }

#if APM_BUILD_TYPE(APM_BUILD_Replay)
    _gcs->send_text(MAV_SEVERITY_CRITICAL, msg);
#endif
}
