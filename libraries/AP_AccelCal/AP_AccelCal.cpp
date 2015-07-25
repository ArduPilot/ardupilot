#include "AP_AccelCal.h"
#include <stdarg.h>
#include <GCS.h>
#include <AP_HAL.h>

const extern AP_HAL::HAL& hal;

void AP_AccelCal::update()
{
    if (!get_calibrator(0)) {
        // no calibrators
        return;
    }

    if (_started) {
        update_status();

        AccelCalibrator *cal;
        uint8_t num_calibrators = 0;
        for(uint8_t i=0; (cal = get_calibrator(i)); i++) {
            num_calibrators++;
        }
        if (num_calibrators != _num_calibrators) {
            clear();
            return;
        }

        switch(_status) {
            case ACCEL_CAL_NOT_STARTED:
                clear();
                return;
            case ACCEL_CAL_WAITING_FOR_ORIENTATION: {
                // if we're waiting for orientation, first ensure that all calibrators are on the same step
                uint8_t step;
                cal = get_calibrator(0);
                step = cal->get_num_samples_collected()+1;

                for(uint8_t i=1 ; (cal = get_calibrator(i))  ; i++) {
                    if (step != cal->get_num_samples_collected()+1) {
                        clear();
                        return;
                    }
                }

                // if we're on a new step, print a message describing the step
                if (step != _step) {
                    _step = step;

                    const prog_char_t *msg;
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
                            clear();
                            return;
                    }
                    _printf("Place vehicle %s and press any key.", msg);
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
                    clear();
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
                        _printf("Calibration successful");
                        clear();
                        hal.scheduler->delay(1000);
                        hal.scheduler->reboot(false);
                    }
                    return;
                } else {
                    for(uint8_t i=0; i<_num_clients; i++) {
                        if(client_active(i) && _clients[i]->_acal_failed()) {
                            _status = ACCEL_CAL_FAILED;
                            clear();
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
                clear();
                return;
        }
    }
}

void AP_AccelCal::start(GCS_MAVLINK *gcs)
{
    if (gcs == NULL || _started) {
        return;
    }

    _num_calibrators = 0;

    AccelCalibrator *cal;
    for(uint8_t i=0; (cal = get_calibrator(i)); i++) {
        cal->clear();
        cal->start(ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID, 6, 0.5f);
        _num_calibrators++;
    }

    _started = true;
    _saving = false;
    _gcs = gcs;
    _step = 0;

    update_status();
}

void AP_AccelCal::clear()
{
    if (!_started) {
        return;
    }

    if (_status != ACCEL_CAL_SUCCESS) {
        _printf("Calibration FAILED");
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
    update_status();
}

void AP_AccelCal::register_client(AP_AccelCal_Client* client) {
    if (client == NULL || _num_clients == AP_ACCELCAL_MAX_NUM_CLIENTS) {
        return;
    }

    if (_started) {
        clear();
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
    return NULL;
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
            _status = ACCEL_CAL_FAILED;
            return;
        }
    }

    for(uint8_t i=0 ; (cal = get_calibrator(i))  ; i++) {
        if (cal->get_status() == ACCEL_CAL_COLLECTING_SAMPLE) {
            _status = ACCEL_CAL_COLLECTING_SAMPLE;
            return;
        }
    }

    for(uint8_t i=0 ; (cal = get_calibrator(i))  ; i++) {
        if (cal->get_status() == ACCEL_CAL_WAITING_FOR_ORIENTATION) {
            _status = ACCEL_CAL_WAITING_FOR_ORIENTATION;
            return;
        }
    }

    for(uint8_t i=0 ; (cal = get_calibrator(i))  ; i++) {
        if (cal->get_status() == ACCEL_CAL_NOT_STARTED) {
            _status = ACCEL_CAL_NOT_STARTED;
            return;
        }
    }

    _status = ACCEL_CAL_SUCCESS;
    return;
}

bool AP_AccelCal::client_active(uint8_t client_num)
{
    return (bool)_clients[client_num]->_acal_get_calibrator(0);
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
    _gcs->send_text(SEVERITY_HIGH, msg);
}
