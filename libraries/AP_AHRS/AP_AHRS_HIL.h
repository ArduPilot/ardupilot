#ifndef __AP_AHRS_HIL_H__
#define __AP_AHRS_HIL_H__

class AP_AHRS_HIL : public AP_AHRS
{
public:
    // Constructors
    AP_AHRS_HIL(AP_InertialSensor *ins, GPS *&gps) : 
	    AP_AHRS(ins, gps),
	    _drift()
		{}

    // Accessors
    virtual Vector3f get_gyro(void) const override {
        return _omega;
    }

    virtual const Matrix3f &get_dcm_matrix(void) const override {
	    return _dcm_matrix;
    }

    // Methods
    virtual void update(void) override {
        _ins->update();
    }
    
    void setHil(float roll, float pitch, float yaw,
                float rollRate, float pitchRate, float yawRate);

    // return the current estimate of the gyro drift
    virtual const Vector3f &get_gyro_drift(void) const override {
	   return  _drift;
    }

    // reset the current attitude, used on new IMU calibration
    virtual void reset(bool recover_eulers=false) override {
    }

    virtual float get_error_rp(void) override {
        return 0;
    }
    virtual float get_error_yaw(void) override {
        return 0;
    }

private:
    Vector3f _omega;
    Matrix3f _dcm_matrix;
    Vector3f _drift;
};

#endif // __AP_AHRS_HIL_H__
