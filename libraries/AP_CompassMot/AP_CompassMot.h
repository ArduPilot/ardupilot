#ifndef AP_CompassMot_h
#define AP_CompassMot_h

#include <AP_Math.h>
#include <AP_Buffer.h>

// larger buffer sizes are more susceptible to errors from rotating compass error, smaller buffer sizes are more susceptible to errors from noise and dont take advantage of enough data.
#define AP_COMPASSMOT_BUFFER_SIZE 6

// the filter constant isn't really a time constant. it gets multiplied by delta amperes. if delta amperes * r^2 * (maxrot-rot)/maxrot is high enough, we can do a full jump to the value
#define AP_COMPASSMOT_FILT_CONST 0.05f

// this is the minimum delta amperes to work on. this is here to keep the algorithm from working on a copter that has a current sensor that doesn't measure motor current
#define AP_COMPASSMOT_MIN_DELTA 4

// this is the maximum total rotation over a buffer. if there is more rotation than this, we will wait to compute until there is less.
#define AP_COMPASSMOT_MAX_ROT ToRad(15)

#define AP_COMPASSMOT_MAG_DELAY 0.04f

class DataFlash_Class;

class AP_CompassMot {
public:
    AP_CompassMot():
    _current (0.0f),
    _gyro_deltat(0.0f),
    _i(0),
    _DataFlash(NULL) {}

    void set_i(uint8_t i) {
        _i = i;
    }

    void set_motfactors(Vector3f motfactors) {
        _motfactors = motfactors;
    }

    const Vector3f &get_motfactors() {
        return _motfactors;
    }

    void set_dataflash(DataFlash_Class *df) {
        _DataFlash = df;
    }
    void set_gyro_deltat(float);
    void update_current(float);
    void update_gyro(Vector3f);
    bool update_compass(Vector3f);

private:
    AP_Buffer<Vector3f, AP_COMPASSMOT_BUFFER_SIZE> _sample_buffer_m;
    AP_Buffer<float, AP_COMPASSMOT_BUFFER_SIZE> _sample_buffer_c;
    AP_Buffer<float, AP_COMPASSMOT_BUFFER_SIZE> _sample_buffer_rot;
    AP_Buffer<Vector3f,16> _gyro_buffer;

    float _current;
    float _gyro_deltat;
    Quaternion _rot_quat;
    Matrix3f   _rot_matrix;
    float _rottotal;
    uint8_t _i;

    Vector3f _motfactors;

    Vector3f _square_vector(const Vector3f& v) {
        return Vector3f(v.x*v.x, v.y*v.y, v.z*v.z);
    }

    Vector3f _sqrt_vector(const Vector3f& v) {
        return Vector3f(safe_sqrt(v.x),safe_sqrt(v.y),safe_sqrt(v.z));
    }

    Vector3f _div_vector(const Vector3f& n, const Vector3f& d) {
        return Vector3f(n.x/d.x,n.y/d.y,n.z/d.z);
    }

    bool _update();
    void _clear_samples() {
        _sample_buffer_m.clear();
        _sample_buffer_c.clear();
        _sample_buffer_rot.clear();
        _rottotal = 0.0f;
    }

    DataFlash_Class *_DataFlash;
    bool _logging_started;
    void write_logging_headers();
    void log_CMOT(uint8_t i, const Vector3f& mf, const Vector3f& m, const Vector3f& r, float curr, float rot);
};
#endif
