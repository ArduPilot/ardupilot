// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_IMU_Shim.h
/// @brief	IMU shim driver, used when the IMU data is coming from somewhere else.

#ifndef AP_IMU_Shim_h
#define AP_IMU_Shim_h

#include "IMU.h"

class AP_IMU_Shim : public IMU
{
public:
    AP_IMU_Shim(void) {
        _product_id = AP_PRODUCT_ID_NONE;
    }


    /// @name IMU protocol
    //@{
    virtual void        init(Start_style            style = COLD_START,
                             void                   (*callback)(unsigned long t) = delay,
                             void                   (*flash_leds_cb)(bool on) = NULL,
                             AP_PeriodicProcess *   scheduler = NULL) {
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

        // return number of microseconds since last call
        uint32_t        us = micros();
        uint32_t        ret = us - last_ch6_micros;
        last_ch6_micros = us;

        _sample_time = ret;
        return updated;
    }
    //@}

    virtual bool        new_data_available(void) {
        return true;
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
    /// set true when new data is delivered
    bool            _updated;
    uint32_t        last_ch6_micros;
};

#endif
