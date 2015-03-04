#include <AP_CompassMot.h>
#include <AP_Common.h>
#include <AP_Buffer.h>
#include <AP_Param.h>
#include <AP_HAL.h>
#include <inttypes.h>
#include <DataFlash.h>
#include <matrix3.h>

extern const AP_HAL::HAL& hal;

void AP_CompassMot::set_gyro_deltat(float deltat) {
    _gyro_deltat = deltat;
}

void AP_CompassMot::update_current(float current) {
    _current = current;
}

void AP_CompassMot::update_gyro(Vector3f gyro) {
    if(_gyro_deltat == 0) {
        _clear_samples();
        _gyro_buffer.clear();
        return;
    }
    // Definable delay w/ interpolation. Max @ 400hz = 0.04sec
    float delay_elements = constrain_float(AP_COMPASSMOT_MAG_DELAY/_gyro_deltat,0.0f,16.0f);
    Vector3f omega;
    if(_gyro_buffer.size() >= delay_elements && delay_elements != 0.0f) {
        //interpolate
        float interp = delay_elements-(int8_t)delay_elements;
        Vector3f prev_gyro;
        _gyro_buffer.pop_front(prev_gyro);
        const Vector3f &next_gyro = _gyro_buffer.front();

        omega = prev_gyro*interp + prev_gyro*(1.0f-interp);
    } else {
        omega = gyro;
    }

    _gyro_buffer.push_back(gyro);
    _rottotal += omega.length() * _gyro_deltat;
    _rot_quat.rotate_fast(omega * _gyro_deltat);
    _rot_quat.normalize();
    _rot_quat.rotation_matrix(_rot_matrix);
}

bool AP_CompassMot::update_compass(Vector3f mag) {
    if(_gyro_buffer.is_empty()) {
        //don't do anything without gyro.
        return false;
    }
    //add sample
    _sample_buffer_m.push_back(_rot_matrix * mag);
    _sample_buffer_c.push_back(_current);
    _sample_buffer_rot.push_back(_rottotal);
    return _update();
}

bool AP_CompassMot::_update() {
    if(!_sample_buffer_c.is_full()) {
        return false;
    }

    uint8_t n = _sample_buffer_c.size();

    float sumx = 0.0f;
    for(uint8_t i=0; i<n; i++) {
        sumx += _sample_buffer_c.peek(i);
    }
    float meancurrent = sumx/n;

    float stdcurrent = 0.0f;
    uint8_t abovecount = 0;
    for(uint8_t i=0; i<n; i++) {
        float residual = _sample_buffer_c.peek(i) - meancurrent;
        if(residual > 0) {
            abovecount++;
        }
        stdcurrent += sq(residual);
    }
    stdcurrent /= n-1;
    stdcurrent = sqrt(stdcurrent);

    float rotation = _sample_buffer_rot.peek(n-1) - _sample_buffer_rot.peek(0);
    
    // check if we're ready to do the linear regression
    if(rotation > AP_COMPASSMOT_MAX_ROT || stdcurrent < AP_COMPASSMOT_MIN_DELTA || abovecount < ((float)n-1.0f)/2.0f || abovecount > ((float)n+1.0f)/2.0f) {
        return false;
    }

    // sum up for linear regression
    // sumx calculated above
    float    sumx2 = 0;
    Vector3f sumxy = Vector3f(0.0f,0.0f,0.0f);
    Vector3f sumy  = Vector3f(0.0f,0.0f,0.0f);
    Vector3f sumy2 = Vector3f(0.0f,0.0f,0.0f);

    //rotate all samples in-place, add up the sums
    for(uint8_t i=0; i<n; i++) {
        Vector3f& mag = _sample_buffer_m.peek_mutable(i);
        float curr = _sample_buffer_c.peek(i);
        mag = _rot_matrix.mul_transpose(mag);

        sumx2 += sq(curr);
        sumxy += mag * curr;
        sumy += mag;
        sumy2 += _square_vector(mag);
    }

    float denom = n*sumx2-sq(sumx);

    if(denom==0.0f) { // fail
        _clear_samples();
        return false;
    }

    Vector3f motfactors_computed = (sumxy*n - sumy*sumx)/denom;

    Vector3f r2;
    Vector3f r_denom_squared = (sumy2 - _square_vector(sumy)/n) * (sumx2 - sq(sumx)/n);
    Vector3f r_numerator = sumxy - sumy*sumx/n;

    if(r_denom_squared.x > 0.0f) {
        r2.x = sq(r_numerator.x / safe_sqrt(r_denom_squared.x));
    } else {
        r2.x = 0.0f;
    }
    if(r_denom_squared.y > 0.0f) {
        r2.y = sq(r_numerator.y / safe_sqrt(r_denom_squared.y));
    } else {
        r2.y = 0.0f;
    }
    if(r_denom_squared.z > 0.0f) {
        r2.z = sq(r_numerator.z / safe_sqrt(r_denom_squared.z));
    } else {
        r2.z = 0.0f;
    }

    // Filter:
    Vector3f motfactors_delta = motfactors_computed - _motfactors;

    Vector3f weights = r2 * AP_COMPASSMOT_FILT_CONST * (stdcurrent-AP_COMPASSMOT_MIN_DELTA) * (AP_COMPASSMOT_MAX_ROT-rotation)/AP_COMPASSMOT_MAX_ROT;

    _motfactors.x += constrain_float(weights.x,0.0f,1.0f) * motfactors_delta.x;
    _motfactors.y += constrain_float(weights.y,0.0f,1.0f) * motfactors_delta.y;
    _motfactors.z += constrain_float(weights.z,0.0f,1.0f) * motfactors_delta.z;

    log_CMOT(_i, _motfactors, motfactors_computed, weights, stdcurrent, rotation);

    //clear the buffer so that we don't reuse any data
    _clear_samples();
    return true;
}

#define LOG_MSG_CMOT 204
#define LOG_MSG_CMO2 205

struct PACKED log_CMOT {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    float fmx;
    float fmy;
    float fmz;
    float mx;
    float my;
    float mz;
    float wx;
    float wy;
    float wz;
    float curr;
    float rot;
};

static const struct LogStructure cmot_log_structures[] PROGMEM = {
    { LOG_MSG_CMOT, sizeof(log_CMOT), "CMOT", "Ifffffffffff", "TimeMS,fMX,fMY,fMZ,MX,MY,MZ,WX,WY,WZ,Curr,Rot" },
    { LOG_MSG_CMO2, sizeof(log_CMOT), "CMO2", "Ifffffffffff", "TimeMS,fMX,fMY,fMZ,MX,MY,MZ,WX,WY,WZ,Curr,Rot" }
};

void AP_CompassMot::write_logging_headers() {
    if (!_logging_started) {
        _logging_started = true;
        _DataFlash->AddLogFormats(cmot_log_structures, 2);
    }
}

void AP_CompassMot::log_CMOT(uint8_t i, const Vector3f &mf, const Vector3f &m, const Vector3f &w, float curr, float rot) {
    if (_DataFlash == NULL || !_DataFlash->logging_started()) {
        return;
    }

    write_logging_headers();
    struct log_CMOT pkt;
    if(i==0) {
        pkt = {
            LOG_PACKET_HEADER_INIT(LOG_MSG_CMOT),
            timestamp : hal.scheduler -> millis(),
            fmx : mf.x,
            fmy : mf.y,
            fmz : mf.z,
            mx : m.x,
            my : m.y,
            mz : m.z,
            wx : w.x,
            wy : w.y,
            wz : w.z,
            curr : curr,
            rot : rot
        };
    } else if(i==1) {
        pkt = {
            LOG_PACKET_HEADER_INIT(LOG_MSG_CMO2),
            timestamp : hal.scheduler -> millis(),
            fmx : mf.x,
            fmy : mf.y,
            fmz : mf.z,
            mx : m.x,
            my : m.y,
            mz : m.z,
            wx : w.x,
            wy : w.y,
            wz : w.z,
            curr : curr,
            rot : rot
        };
    } else {
        return;
    }

    _DataFlash->WriteBlock(&pkt, sizeof(pkt));
}
