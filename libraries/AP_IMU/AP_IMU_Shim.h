// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_IMU_Shim.h
/// @brief	IMU shim driver, used when the IMU data is coming from somewhere else.

#ifndef AP_IMU_Shim_h
#define AP_IMU_Shim_h

#include "IMU.h"

#define AP_IMU_SHIM_UPDATE_MICROS 10000     // 10000 micrxos = 10 ms = 100hz

class AP_IMU_Shim : public IMU
{
public:
    AP_IMU_Shim(void) {
        _product_id = AP_PRODUCT_ID_NONE;
        _first_sample_time_micros = 0;
        _last_sample_time_micros = micros();        
    }


    /// @name IMU protocol
    //@{
    virtual void        init(Start_style            style = COLD_START,
                             void                   (*callback)(unsigned long t) = delay,
                             void                   (*flash_leds_cb)(bool on) = NULL,
                             AP_PeriodicProcess *   scheduler = NULL) {
                                scheduler->register_process( AP_IMU_Shim::read );
    };
    virtual void        init_accel(void (*callback)(unsigned long t) = delay,
                                   void (*flash_leds_cb)(bool on) = NULL) {
    };
    virtual void        init_gyro(void  (*callback)(unsigned long t) = delay,
                                  void  (*flash_leds_cb)(bool on) = NULL) {
    };
    virtual bool        update(void) {
        bool        updated = _updated;
        _updated = false;
        _count = 0;
        _sample_time = _last_sample_time_micros - _first_sample_time_micros;
        _first_sample_time_micros = _last_sample_time_micros;
        return updated;
    }
    //@}

    virtual bool        new_data_available(void) {
        return _count > 0;
    }

    /// Get number of samples read from the sensors
    /// @returns    number of samples read from the sensors
    uint16_t     num_samples_available(void) {
        return _count;
    }

    float           gx()                            {
        return 0;
    }
    float           gy()                            {
        return 0;
    }
    float           gz()                            {
        return 0;
    }
    float           ax()                            {
        return 0;
    }
    float           ay()                            {
        return 0;
    }
    float           az()                            {
        return 0;
    }

    void        ax(const int v)         {
    }
    void        ay(const int v)         {
    }
    void        az(const int v)         {
    }

    /// Set the gyro vector.  ::update will return
    /// true once after this call.
    ///
    /// @param	v		The new gyro vector.
    ///
    void        set_gyro(Vector3f v) {
        _gyro = v; _updated = true;
    }

    /// Set the accelerometer vector.  ::update will return
    /// true once after this call.
    ///
    /// @param	v		The new accelerometer vector.
    ///
    void        set_accel(Vector3f v) {
        _accel = v; _updated = true;
    }

    // dummy save method
    void            save(void) {
    }

    float           get_gyro_drift_rate(void) {
        return 0;
    }


private:

    // read function that pretends to capture new data
    static void read(uint32_t tnow) {
        _last_sample_time_micros = tnow;
       _count++;
    }

    static uint16_t  _count;                    // number of samples captured
    static uint32_t  _first_sample_time_micros; // time first sample began (equal to the last sample time of the previous iteration)
    static uint32_t  _last_sample_time_micros;  // time that the latest sample was captured

    /// set true when new data is delivered
    bool            _updated;

};

#endif
