#ifndef __AP_ACCELCAL_H__
#define __AP_ACCELCAL_H__

#include <GCS_MAVLink.h>
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

    void start(GCS_MAVLINK *gcs);
    void fail();
    void success();
    void clear();
    void update();
    void register_client(AP_AccelCal_Client* client);
    void collect_sample();
    accel_cal_status_t get_status() { return _status; }

private:
    GCS_MAVLINK *_gcs;
    uint8_t _step;
    accel_cal_status_t _status;
    uint8_t _num_clients;
    AP_AccelCal_Client* _clients[AP_ACCELCAL_MAX_NUM_CLIENTS];

    void update_status();
    bool check_for_timeout();

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
