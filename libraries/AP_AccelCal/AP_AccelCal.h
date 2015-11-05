#ifndef __AP_ACCELCAL_H__
#define __AP_ACCELCAL_H__

#include <GCS_MAVLink/GCS_MAVLink.h>
#include "AccelCalibrator.h"

#define AP_ACCELCAL_MAX_NUM_CLIENTS 4
class GCS_MAVLINK;
class AP_AccelCal_Client;

class AP_AccelCal {
public:
    AP_AccelCal():
    _num_clients(0),
    _started(false),
    _saving(false)
    { update_status(); }

    // start all the registered calibrations
    void start(GCS_MAVLINK *gcs);

    // interface to the user in case of the faillure to even a single calibrator
    void fail();

    // interface to the user in case all the calibrators succeed
    void success();

    // reset all the calibrators to there pre calibration stage so as to make them ready for next calibration request
    void clear();

    // Run an iteration of all registered calibrations
    void update();

    // interface to the clients for registration
    void register_client(AP_AccelCal_Client* client);

    // proceed through the collection step for each of the registered calibrators
    void collect_sample();

    // get the status of the calibrator server as a whole
    accel_cal_status_t get_status() { return _status; }

private:
    GCS_MAVLINK *_gcs;
    uint8_t _step;
    accel_cal_status_t _status;
    uint8_t _num_clients;
    AP_AccelCal_Client* _clients[AP_ACCELCAL_MAX_NUM_CLIENTS];

    // update the state of the Accel calibrator server
    void update_status();

    // checks if no new sample has been recieved for considerable amount of time
    bool check_for_timeout();

    // check if client's calibrator is active
    bool client_active(uint8_t client_num);

    bool _started;
    bool _saving;

    uint8_t _num_calibrators;

    AccelCalibrator* get_calibrator(uint8_t i);
    void _printf(const char*, ...);
};

class AP_AccelCal_Client {
friend class AP_AccelCal;
private:
    virtual void _acal_save_calibrations() = 0;
    virtual bool _acal_saving() { return false; }
    virtual bool _acal_ready_to_sample() { return true; }
    virtual bool _acal_failed() { return false; }
    virtual void _acal_cancelled() {};
    virtual AccelCalibrator* _acal_get_calibrator(uint8_t instance) = 0;
};

#endif
