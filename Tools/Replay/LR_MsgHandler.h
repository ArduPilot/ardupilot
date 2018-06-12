#pragma once

#include "MsgHandler.h"

#include <functional>

class LR_MsgHandler : public MsgHandler {
public:
    LR_MsgHandler(struct log_Format &f,
                  DataFlash_Class &_dataflash,
                  uint64_t &last_timestamp_usec);
    virtual void process_message(uint8_t *msg) = 0;

    // state for CHEK message
    struct CheckState {
        uint64_t time_us;
        Vector3f euler;
        Location pos;
        Vector3f velocity;
    };

protected:
    DataFlash_Class &dataflash;
    void wait_timestamp(uint32_t timestamp);
    void wait_timestamp_usec(uint64_t timestamp);
    void wait_timestamp_from_msg(uint8_t *msg);

    uint64_t &last_timestamp_usec;

};

/* subclasses below this point */

class LR_MsgHandler_AHR2 : public LR_MsgHandler
{
public:
    LR_MsgHandler_AHR2(log_Format &_f, DataFlash_Class &_dataflash,
                    uint64_t &_last_timestamp_usec, Vector3f &_ahr2_attitude)
        : LR_MsgHandler(_f, _dataflash,_last_timestamp_usec),
          ahr2_attitude(_ahr2_attitude) { };

    virtual void process_message(uint8_t *msg);

private:
    Vector3f &ahr2_attitude;
};


class LR_MsgHandler_ARM : public LR_MsgHandler
{
public:
    LR_MsgHandler_ARM(log_Format &_f, DataFlash_Class &_dataflash,
                   uint64_t &_last_timestamp_usec)
        : LR_MsgHandler(_f, _dataflash, _last_timestamp_usec) { };

    virtual void process_message(uint8_t *msg);
};


class LR_MsgHandler_ARSP : public LR_MsgHandler
{
public:
    LR_MsgHandler_ARSP(log_Format &_f, DataFlash_Class &_dataflash,
		    uint64_t &_last_timestamp_usec, AP_Airspeed &_airspeed) :
	LR_MsgHandler(_f, _dataflash, _last_timestamp_usec), airspeed(_airspeed) { };

    virtual void process_message(uint8_t *msg);

private:
    AP_Airspeed &airspeed;
};

class LR_MsgHandler_NKF1 : public LR_MsgHandler
{
public:
    LR_MsgHandler_NKF1(log_Format &_f, DataFlash_Class &_dataflash,
		    uint64_t &_last_timestamp_usec) :
	LR_MsgHandler(_f, _dataflash, _last_timestamp_usec) { };

    virtual void process_message(uint8_t *msg);
};


class LR_MsgHandler_ATT : public LR_MsgHandler
{
public:
    LR_MsgHandler_ATT(log_Format &_f, DataFlash_Class &_dataflash,
                   uint64_t &_last_timestamp_usec, Vector3f &_attitude)
        : LR_MsgHandler(_f, _dataflash, _last_timestamp_usec), attitude(_attitude)
        { };
    virtual void process_message(uint8_t *msg);

private:
    Vector3f &attitude;
};


class LR_MsgHandler_CHEK : public LR_MsgHandler
{
public:
    LR_MsgHandler_CHEK(log_Format &_f, DataFlash_Class &_dataflash,
                       uint64_t &_last_timestamp_usec, CheckState &_check_state)
        : LR_MsgHandler(_f, _dataflash, _last_timestamp_usec), 
          check_state(_check_state)
        { };
    virtual void process_message(uint8_t *msg);

private:
    CheckState &check_state;
};

class LR_MsgHandler_BARO : public LR_MsgHandler
{
public:
    LR_MsgHandler_BARO(log_Format &_f, DataFlash_Class &_dataflash,
                    uint64_t &_last_timestamp_usec)
        : LR_MsgHandler(_f, _dataflash, _last_timestamp_usec)
        { };

    virtual void process_message(uint8_t *msg);

};


class LR_MsgHandler_Event : public LR_MsgHandler
{
public:
    LR_MsgHandler_Event(log_Format &_f, DataFlash_Class &_dataflash,
                   uint64_t &_last_timestamp_usec)
        : LR_MsgHandler(_f, _dataflash, _last_timestamp_usec) { };

    virtual void process_message(uint8_t *msg);
};




class LR_MsgHandler_GPS_Base : public LR_MsgHandler
{

public:
    LR_MsgHandler_GPS_Base(log_Format &_f, DataFlash_Class &_dataflash,
                           uint64_t &_last_timestamp_usec, AP_GPS &_gps,
                           uint32_t &_ground_alt_cm)
        : LR_MsgHandler(_f, _dataflash, _last_timestamp_usec),
          gps(_gps), ground_alt_cm(_ground_alt_cm) { };

protected:
    void update_from_msg_gps(uint8_t imu_offset, uint8_t *data);

private:
    AP_GPS &gps;
    uint32_t &ground_alt_cm;
};

class LR_MsgHandler_GPS : public LR_MsgHandler_GPS_Base
{
public:
    LR_MsgHandler_GPS(log_Format &_f, DataFlash_Class &_dataflash,
                   uint64_t &_last_timestamp_usec, AP_GPS &_gps,
                   uint32_t &_ground_alt_cm)
        : LR_MsgHandler_GPS_Base(_f, _dataflash,_last_timestamp_usec,
                              _gps, _ground_alt_cm),
        gps(_gps), ground_alt_cm(_ground_alt_cm) { };

    void process_message(uint8_t *msg);

private:
    AP_GPS &gps;
    uint32_t &ground_alt_cm;
};

// it would be nice to use the same parser for both GPS message types
// (and other packets, too...).  I*think* the contructor can simply
// take e.g. &gps[1]... problems are going to arise if we don't
// actually have that many gps' compiled in!
class LR_MsgHandler_GPS2 : public LR_MsgHandler_GPS_Base
{
public:
    LR_MsgHandler_GPS2(log_Format &_f, DataFlash_Class &_dataflash,
                    uint64_t &_last_timestamp_usec, AP_GPS &_gps,
                    uint32_t &_ground_alt_cm)
        : LR_MsgHandler_GPS_Base(_f, _dataflash, _last_timestamp_usec,
                                 _gps, _ground_alt_cm), gps(_gps),
        ground_alt_cm(_ground_alt_cm) { };
    virtual void process_message(uint8_t *msg);
private:
    AP_GPS &gps;
    uint32_t &ground_alt_cm;
};

class LR_MsgHandler_GPA_Base : public LR_MsgHandler
{

public:
    LR_MsgHandler_GPA_Base(log_Format &_f, DataFlash_Class &_dataflash,
                           uint64_t &_last_timestamp_usec, AP_GPS &_gps)
        : LR_MsgHandler(_f, _dataflash, _last_timestamp_usec), gps(_gps) { };

protected:
    void update_from_msg_gpa(uint8_t imu_offset, uint8_t *data);

private:
    AP_GPS &gps;
};


class LR_MsgHandler_GPA : public LR_MsgHandler_GPA_Base
{
public:
    LR_MsgHandler_GPA(log_Format &_f, DataFlash_Class &_dataflash,
                      uint64_t &_last_timestamp_usec, AP_GPS &_gps)
        : LR_MsgHandler_GPA_Base(_f, _dataflash,_last_timestamp_usec,
                              _gps), gps(_gps) { };

    void process_message(uint8_t *msg);

private:
    AP_GPS &gps;
};

class LR_MsgHandler_GPA2 : public LR_MsgHandler_GPA_Base
{
public:
    LR_MsgHandler_GPA2(log_Format &_f, DataFlash_Class &_dataflash,
                       uint64_t &_last_timestamp_usec, AP_GPS &_gps)
        : LR_MsgHandler_GPA_Base(_f, _dataflash, _last_timestamp_usec,
                                 _gps), gps(_gps) { };
    virtual void process_message(uint8_t *msg);
private:
    AP_GPS &gps;
};





class LR_MsgHandler_IMU_Base : public LR_MsgHandler
{
public:
    LR_MsgHandler_IMU_Base(log_Format &_f, DataFlash_Class &_dataflash,
                        uint64_t &_last_timestamp_usec,
                        uint8_t &_accel_mask, uint8_t &_gyro_mask,
                        AP_InertialSensor &_ins) :
        LR_MsgHandler(_f, _dataflash, _last_timestamp_usec),
        accel_mask(_accel_mask),
        gyro_mask(_gyro_mask),
        ins(_ins) { };
    void update_from_msg_imu(uint8_t imu_offset, uint8_t *msg);

private:
    uint8_t &accel_mask;
    uint8_t &gyro_mask;
    AP_InertialSensor &ins;
};

class LR_MsgHandler_IMU : public LR_MsgHandler_IMU_Base
{
public:
    LR_MsgHandler_IMU(log_Format &_f, DataFlash_Class &_dataflash,
                   uint64_t &_last_timestamp_usec,
                   uint8_t &_accel_mask, uint8_t &_gyro_mask,
                   AP_InertialSensor &_ins)
        : LR_MsgHandler_IMU_Base(_f, _dataflash, _last_timestamp_usec,
                              _accel_mask, _gyro_mask, _ins) { };

    void process_message(uint8_t *msg);
};

class LR_MsgHandler_IMU2 : public LR_MsgHandler_IMU_Base
{
public:
    LR_MsgHandler_IMU2(log_Format &_f, DataFlash_Class &_dataflash,
                    uint64_t &_last_timestamp_usec,
                    uint8_t &_accel_mask, uint8_t &_gyro_mask,
                    AP_InertialSensor &_ins)
        : LR_MsgHandler_IMU_Base(_f, _dataflash, _last_timestamp_usec,
                              _accel_mask, _gyro_mask, _ins) {};

    virtual void process_message(uint8_t *msg);
};

class LR_MsgHandler_IMU3 : public LR_MsgHandler_IMU_Base
{
public:
    LR_MsgHandler_IMU3(log_Format &_f, DataFlash_Class &_dataflash,
                    uint64_t &_last_timestamp_usec,
                    uint8_t &_accel_mask, uint8_t &_gyro_mask,
                    AP_InertialSensor &_ins)
        : LR_MsgHandler_IMU_Base(_f, _dataflash, _last_timestamp_usec,
                              _accel_mask, _gyro_mask, _ins) {};

    virtual void process_message(uint8_t *msg);
};


class LR_MsgHandler_IMT_Base : public LR_MsgHandler
{
public:
    LR_MsgHandler_IMT_Base(log_Format &_f, DataFlash_Class &_dataflash,
                           uint64_t &_last_timestamp_usec,
                           uint8_t &_accel_mask, uint8_t &_gyro_mask,
                           bool &_use_imt,
                           AP_InertialSensor &_ins) :
        LR_MsgHandler(_f, _dataflash, _last_timestamp_usec),
        accel_mask(_accel_mask),
        gyro_mask(_gyro_mask),
        use_imt(_use_imt),
        ins(_ins) { };
    void update_from_msg_imt(uint8_t imu_offset, uint8_t *msg);

private:
    uint8_t &accel_mask;
    uint8_t &gyro_mask;
    bool &use_imt;
    AP_InertialSensor &ins;
};

class LR_MsgHandler_IMT : public LR_MsgHandler_IMT_Base
{
public:
    LR_MsgHandler_IMT(log_Format &_f, DataFlash_Class &_dataflash,
                      uint64_t &_last_timestamp_usec,
                      uint8_t &_accel_mask, uint8_t &_gyro_mask,
                      bool &_use_imt,
                      AP_InertialSensor &_ins)
        : LR_MsgHandler_IMT_Base(_f, _dataflash, _last_timestamp_usec,
                                 _accel_mask, _gyro_mask, _use_imt, _ins) { };

    void process_message(uint8_t *msg);
};

class LR_MsgHandler_IMT2 : public LR_MsgHandler_IMT_Base
{
public:
    LR_MsgHandler_IMT2(log_Format &_f, DataFlash_Class &_dataflash,
                       uint64_t &_last_timestamp_usec,
                       uint8_t &_accel_mask, uint8_t &_gyro_mask,
                       bool &_use_imt,
                       AP_InertialSensor &_ins)
        : LR_MsgHandler_IMT_Base(_f, _dataflash, _last_timestamp_usec,
                                 _accel_mask, _gyro_mask, _use_imt, _ins) { };

    void process_message(uint8_t *msg);
};

class LR_MsgHandler_IMT3 : public LR_MsgHandler_IMT_Base
{
public:
    LR_MsgHandler_IMT3(log_Format &_f, DataFlash_Class &_dataflash,
                       uint64_t &_last_timestamp_usec,
                       uint8_t &_accel_mask, uint8_t &_gyro_mask,
                       bool &_use_imt,
                       AP_InertialSensor &_ins)
        : LR_MsgHandler_IMT_Base(_f, _dataflash, _last_timestamp_usec,
                                 _accel_mask, _gyro_mask, _use_imt, _ins) { };

    void process_message(uint8_t *msg);
};


class LR_MsgHandler_MAG_Base : public LR_MsgHandler
{
public:
    LR_MsgHandler_MAG_Base(log_Format &_f, DataFlash_Class &_dataflash,
                        uint64_t &_last_timestamp_usec, Compass &_compass)
	: LR_MsgHandler(_f, _dataflash, _last_timestamp_usec), compass(_compass) { };

protected:
    void update_from_msg_compass(uint8_t compass_offset, uint8_t *msg);

private:
    Compass &compass;
};

class LR_MsgHandler_MAG : public LR_MsgHandler_MAG_Base
{
public:
    LR_MsgHandler_MAG(log_Format &_f, DataFlash_Class &_dataflash,
                   uint64_t &_last_timestamp_usec, Compass &_compass)
        : LR_MsgHandler_MAG_Base(_f, _dataflash, _last_timestamp_usec,_compass) {};

    virtual void process_message(uint8_t *msg);
};

class LR_MsgHandler_MAG2 : public LR_MsgHandler_MAG_Base
{
public:
    LR_MsgHandler_MAG2(log_Format &_f, DataFlash_Class &_dataflash,
                    uint64_t &_last_timestamp_usec, Compass &_compass)
        : LR_MsgHandler_MAG_Base(_f, _dataflash, _last_timestamp_usec,_compass) {};

    virtual void process_message(uint8_t *msg);
};



class LR_MsgHandler_MSG : public LR_MsgHandler
{
public:
    LR_MsgHandler_MSG(log_Format &_f, DataFlash_Class &_dataflash,
                   uint64_t &_last_timestamp_usec,
                   VehicleType::vehicle_type &_vehicle, AP_AHRS &_ahrs) :
        LR_MsgHandler(_f, _dataflash, _last_timestamp_usec),
        vehicle(_vehicle), ahrs(_ahrs) { }


    virtual void process_message(uint8_t *msg);

private:
    VehicleType::vehicle_type &vehicle;
    AP_AHRS &ahrs;
};


class LR_MsgHandler_NTUN_Copter : public LR_MsgHandler
{
public:
    LR_MsgHandler_NTUN_Copter(log_Format &_f, DataFlash_Class &_dataflash,
			   uint64_t &_last_timestamp_usec, Vector3f &_inavpos)
	: LR_MsgHandler(_f, _dataflash, _last_timestamp_usec), inavpos(_inavpos) {};

    virtual void process_message(uint8_t *msg);

private:
    Vector3f &inavpos;
};


class LR_MsgHandler_PARM : public LR_MsgHandler
{
public:
    LR_MsgHandler_PARM(log_Format &_f, DataFlash_Class &_dataflash,
                       uint64_t &_last_timestamp_usec,
                       const std::function<bool(const char *name, const float)>&set_parameter_callback) :
        LR_MsgHandler(_f, _dataflash, _last_timestamp_usec),
        _set_parameter_callback(set_parameter_callback)
        {};

    virtual void process_message(uint8_t *msg);

private:
    bool set_parameter(const char *name, const float value);
    const std::function<bool(const char *name, const float)>_set_parameter_callback;
};

class LR_MsgHandler_PM : public LR_MsgHandler
{
public:
    LR_MsgHandler_PM(log_Format &_f, DataFlash_Class &_dataflash,
                     uint64_t &_last_timestamp_usec)
        : LR_MsgHandler(_f, _dataflash, _last_timestamp_usec) { };

    virtual void process_message(uint8_t *msg);

private:

};

class LR_MsgHandler_SIM : public LR_MsgHandler
{
public:
    LR_MsgHandler_SIM(log_Format &_f, DataFlash_Class &_dataflash,
                   uint64_t &_last_timestamp_usec,
                   Vector3f &_sim_attitude)
        : LR_MsgHandler(_f, _dataflash, _last_timestamp_usec),
          sim_attitude(_sim_attitude)
        { };

    virtual void process_message(uint8_t *msg);

private:
    Vector3f &sim_attitude;
};
