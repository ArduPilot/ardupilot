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
    _started(false),
    _saving(false)
    { update_status(); }

    // start all the registered calibrations
    void start(GCS_MAVLINK *gcs);

    // called on calibration cancellation
    void cancel();

    // Run an iteration of all registered calibrations
    void update();

    // get the status of the calibrator server as a whole
    accel_cal_status_t get_status() { return _status; }

    // interface to the clients for registration
    static void register_client(AP_AccelCal_Client* client);

private:
    GCS_MAVLINK *_gcs;
    uint8_t _step;
    accel_cal_status_t _status;

    static uint8_t _num_clients;
    static AP_AccelCal_Client* _clients[AP_ACCELCAL_MAX_NUM_CLIENTS];

    // called on calibration success
    void success();

    // called on calibration failure
    void fail();

    // reset all the calibrators to there pre calibration stage so as to make them ready for next calibration request
    void clear();

    // proceed through the collection step for each of the registered calibrators
    void collect_sample();

    // update the state of the Accel calibrator server
    void update_status();

    // checks if no new sample has been recieved for considerable amount of time
    bool check_for_timeout();

    // check if client's calibrator is active
    bool client_active(uint8_t client_num);

    bool _started;
    bool _saving;

    uint8_t _num_active_calibrators;

    AccelCalibrator* get_calibrator(uint8_t i);
    void _printf(const char*, ...);
};

class AP_AccelCal_Client {
friend class AP_AccelCal;
private:
    // getters
    virtual bool _acal_get_saving() { return false; }
    virtual bool _acal_get_ready_to_sample() { return true; }
    virtual bool _acal_get_fail() { return false; }
    virtual AccelCalibrator* _acal_get_calibrator(uint8_t instance) = 0;

    // events
    virtual void _acal_save_calibrations() = 0;
    virtual void _acal_event_success() {};
    virtual void _acal_event_cancellation() {};
    virtual void _acal_event_failure() {};
};

#endif
