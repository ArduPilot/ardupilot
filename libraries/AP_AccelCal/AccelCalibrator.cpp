/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "AccelCalibrator.h"
#include <stdio.h>
#include <AP_HAL.h>

const extern AP_HAL::HAL& hal;

/*
 * TODO
 * - time out when not receiving samples
 */

////////////////////////////////////////////////////////////
///////////////////// PUBLIC INTERFACE /////////////////////
////////////////////////////////////////////////////////////

AccelCalibrator::AccelCalibrator() :
_conf_tolerance(0.1f),
_sample_buffer(NULL)
{
    clear();
}

void AccelCalibrator::start(enum accel_cal_fit_type_t fit_type, uint8_t num_samples, float sample_time) {
    start(fit_type, num_samples, sample_time, Vector3f(0,0,0), Vector3f(1,1,1), Vector3f(0,0,0));
}

void AccelCalibrator::start(enum accel_cal_fit_type_t fit_type, uint8_t num_samples, float sample_time, Vector3f offset, Vector3f diag, Vector3f offdiag) {
    if(_status == ACCEL_CAL_FAILED || _status == ACCEL_CAL_SUCCESS) {
        clear();
    }
    if(_status != ACCEL_CAL_NOT_STARTED) {
        return;
    }

    _conf_num_samples = num_samples;
    _conf_sample_time = sample_time;
    _conf_fit_type = fit_type;

    _params.offset = offset;
    _params.diag = diag;
    _params.offdiag = offdiag;

    switch (_conf_fit_type) {
        case ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID:
            if (_conf_num_samples < 6) {
                set_status(ACCEL_CAL_FAILED);
                return;
            }
            break;
        case ACCEL_CAL_ELLIPSOID:
            if (_conf_num_samples < 8) {
                set_status(ACCEL_CAL_FAILED);
                return;
            }
            break;
    }

    set_status(ACCEL_CAL_WAITING_FOR_ORIENTATION);
}

void AccelCalibrator::clear() {
    set_status(ACCEL_CAL_NOT_STARTED);
}

bool AccelCalibrator::running() {
    return _status == ACCEL_CAL_WAITING_FOR_ORIENTATION || _status == ACCEL_CAL_COLLECTING_SAMPLE;
}

void AccelCalibrator::collect_sample() {
    set_status(ACCEL_CAL_COLLECTING_SAMPLE);
}

void AccelCalibrator::new_sample(Vector3f delta_velocity, float dt) {
    if (_status != ACCEL_CAL_COLLECTING_SAMPLE) {
        return;
    }

    if (_samples_collected >= _conf_num_samples) {
        set_status(ACCEL_CAL_FAILED);
        return;
    }

    _sample_buffer[_samples_collected].delta_velocity += delta_velocity;
    _sample_buffer[_samples_collected].delta_time += dt;

    _last_samp_frag_collected_ms = hal.scheduler->millis();

    if (_sample_buffer[_samples_collected].delta_time > _conf_sample_time) {
        Vector3f sample = _sample_buffer[_samples_collected].delta_velocity/_sample_buffer[_samples_collected].delta_time;
        if(!accept_sample(sample)) {
            set_status(ACCEL_CAL_FAILED);
            return;
        }

        _samples_collected++;

        if (_samples_collected >= _conf_num_samples) {
            run_fit(50, _fitness);

            if (_fitness < _conf_tolerance && accept_result()) {
                set_status(ACCEL_CAL_SUCCESS);
            } else {
                set_status(ACCEL_CAL_FAILED);
            }
        } else {
            set_status(ACCEL_CAL_WAITING_FOR_ORIENTATION);
        }
    }
}

bool AccelCalibrator::accept_result() const {
    if (fabsf(_params.offset.x) > GRAVITY_MSS ||
        fabsf(_params.offset.y) > GRAVITY_MSS ||
        fabsf(_params.offset.z) > GRAVITY_MSS ||
        _params.diag.x < 0.8f || _params.diag.x > 1.2f ||
        _params.diag.y < 0.8f || _params.diag.y > 1.2f ||
        _params.diag.z < 0.8f || _params.diag.z > 1.2f) {
        return false;
    } else {
        return true;
    }
}

bool AccelCalibrator::get_sample(uint8_t i, Vector3f& s) const {
    if (i >= _samples_collected) {
        return false;
    }
    s = _sample_buffer[i].delta_velocity / _sample_buffer[i].delta_time;
    return true;
}
bool AccelCalibrator::get_sample_corrected(uint8_t i, Vector3f& s) const {
    if (_status != ACCEL_CAL_SUCCESS || !get_sample(i,s)) {
        return false;
    }

    Matrix3f M (
        _params.diag.x    , _params.offdiag.x , _params.offdiag.y,
        _params.offdiag.x , _params.diag.y    , _params.offdiag.z,
        _params.offdiag.y , _params.offdiag.z , _params.diag.z
    );

    s = M*(s+_params.offset);

    return true;
}

void AccelCalibrator::check_for_timeout() {
    static const uint32_t timeout = _conf_sample_time*2*1000 + 500;
    if (_status == ACCEL_CAL_COLLECTING_SAMPLE && hal.scheduler->millis() - _last_samp_frag_collected_ms > timeout) {
        set_status(ACCEL_CAL_FAILED);
    }
}

void AccelCalibrator::get_calibration(Vector3f& offset) {
    offset = -_params.offset;
}

void AccelCalibrator::get_calibration(Vector3f& offset, Vector3f& diag) {
    offset = -_params.offset;
    diag = _params.diag;
}

void AccelCalibrator::get_calibration(Vector3f& offset, Vector3f& diag, Vector3f& offdiag) {
    offset = -_params.offset;
    diag = _params.diag;
    offdiag = _params.offdiag;
}

/////////////////////////////////////////////////////////////
////////////////////// PRIVATE METHODS //////////////////////
/////////////////////////////////////////////////////////////

bool AccelCalibrator::accept_sample(const Vector3f& sample)
{
    if(_sample_buffer == NULL) {
        return false;
    }

    uint8_t faces = 2*_conf_num_samples-4;
    float theta = acosf(cosf((4.0f*M_PI_F/(3.0f*faces)) + M_PI_F/3.0f)/(1.0f-cosf((4.0f*M_PI_F/(3.0f*faces)) + M_PI_F/3.0f)));
    theta *= 0.5f;
    float min_distance = GRAVITY_MSS * 2*sinf(theta/2);

    for(uint8_t i=0; i < _samples_collected; i++) {
        Vector3f other_sample;
        get_sample(i, other_sample);

        if((other_sample - sample).length() < min_distance){
            return false;
        }
    }
    return true;
}

void AccelCalibrator::set_status(enum accel_cal_status_t status) {
    switch (status) {
        case ACCEL_CAL_NOT_STARTED:
            _status = ACCEL_CAL_NOT_STARTED;

            _samples_collected = 0;
            if (_sample_buffer != NULL) {
                free(_sample_buffer);
                _sample_buffer = NULL;
            }

            break;

        case ACCEL_CAL_WAITING_FOR_ORIENTATION:
            if (!running()) {
                _samples_collected = 0;
                if (_sample_buffer == NULL) {
                    _sample_buffer = (struct AccelSample*)malloc(sizeof(struct AccelSample)*_conf_num_samples);
                    if (_sample_buffer == NULL) {
                        set_status(ACCEL_CAL_FAILED);
                        break;
                    }
                    memset(_sample_buffer, 0, sizeof(struct AccelSample)*_conf_num_samples);
                }
            }
            if (_samples_collected >= _conf_num_samples) {
                break;
            }
            _status = ACCEL_CAL_WAITING_FOR_ORIENTATION;
            break;

        case ACCEL_CAL_COLLECTING_SAMPLE:
            if (_status != ACCEL_CAL_WAITING_FOR_ORIENTATION) {
                break;
            }
            _last_samp_frag_collected_ms = hal.scheduler->millis();
            _status = ACCEL_CAL_COLLECTING_SAMPLE;
            break;

        case ACCEL_CAL_SUCCESS:
            if (_status != ACCEL_CAL_COLLECTING_SAMPLE) {
                break;
            }

            _status = ACCEL_CAL_SUCCESS;
            break;

        case ACCEL_CAL_FAILED:
            if (_status == ACCEL_CAL_NOT_STARTED) {
                break;
            }

            _status = ACCEL_CAL_FAILED;
            break;
    };
}

void AccelCalibrator::run_fit(uint8_t max_iterations, float& fitness)
{
    if(_sample_buffer == NULL) {
        return;
    }
    fitness = calc_mean_squared_residuals(_params);
    float min_fitness = fitness;

    struct param_t fit_param = _params;
    float* param_array = (float*)&fit_param;
    uint8_t num_iterations = 0;

    while(num_iterations < max_iterations) {
        float last_fitness = fitness;

        float JTJ[ACCEL_CAL_MAX_NUM_PARAMS*ACCEL_CAL_MAX_NUM_PARAMS];
        float JTFI[ACCEL_CAL_MAX_NUM_PARAMS];

        memset(&JTJ,0,sizeof(JTJ));
        memset(&JTFI,0,sizeof(JTFI));

        for(uint16_t k = 0; k<_samples_collected; k++) {
            Vector3f sample;
            get_sample(k, sample);

            float jacob[ACCEL_CAL_MAX_NUM_PARAMS];

            calc_jacob(sample, fit_param, jacob);

            for(uint8_t i = 0; i < get_num_params(); i++) {
                // compute JTJ
                for(uint8_t j = 0; j < get_num_params(); j++) {
                    JTJ[i*get_num_params()+j] += jacob[i] * jacob[j];
                }
                // compute JTFI
                JTFI[i] += jacob[i] * calc_residual(sample, fit_param);
            }
        }

        if(!inverse(JTJ, JTJ, get_num_params())) {
            return;
        }

        for(uint8_t row=0; row < get_num_params(); row++) {
            for(uint8_t col=0; col < get_num_params(); col++) {
                param_array[row] -= JTFI[col] * JTJ[row*get_num_params()+col];
            }
        }

        fitness = calc_mean_squared_residuals(fit_param);

        if(isnan(fitness) || isinf(fitness)) {
            return;
        }

        if(fitness < min_fitness) {
            min_fitness = fitness;
            _params = fit_param;
        }

        num_iterations++;
        if (fitness - last_fitness < 1.0e-9f) {
            break;
        }
    }
}

float AccelCalibrator::calc_residual(const Vector3f& sample, const struct param_t& params) const {
    Matrix3f M (
        params.diag.x    , params.offdiag.x , params.offdiag.y,
        params.offdiag.x , params.diag.y    , params.offdiag.z,
        params.offdiag.y , params.offdiag.z , params.diag.z
    );
    return GRAVITY_MSS - (M*(sample+params.offset)).length();
}

float AccelCalibrator::calc_mean_squared_residuals() const
{
    return calc_mean_squared_residuals(_params);
}

float AccelCalibrator::calc_mean_squared_residuals(const struct param_t& params) const
{
    if(_sample_buffer == NULL || _samples_collected == 0) {
        return 1.0e30f;
    }
    float sum = 0.0f;
    for(uint16_t i=0; i < _samples_collected; i++){
        Vector3f sample;
        get_sample(i, sample);
        float resid = calc_residual(sample, params);
        sum += sq(resid);
    }
    sum /= _samples_collected;
    return sum;
}

void AccelCalibrator::calc_jacob(const Vector3f& sample, const struct param_t& params, float* ret) const {
    switch (_conf_fit_type) {
        case ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID:
        case ACCEL_CAL_ELLIPSOID: {
            const Vector3f &offset = params.offset;
            const Vector3f &diag = params.diag;
            const Vector3f &offdiag = params.offdiag;
            Matrix3f M(
                diag.x    , offdiag.x , offdiag.y,
                offdiag.x , diag.y    , offdiag.z,
                offdiag.y , offdiag.z , diag.z
            );

            float A =  (diag.x    * (sample.x + offset.x)) + (offdiag.x * (sample.y + offset.y)) + (offdiag.y * (sample.z + offset.z));
            float B =  (offdiag.x * (sample.x + offset.x)) + (diag.y    * (sample.y + offset.y)) + (offdiag.z * (sample.z + offset.z));
            float C =  (offdiag.y * (sample.x + offset.x)) + (offdiag.z * (sample.y + offset.y)) + (diag.z    * (sample.z + offset.z));
            float length = (M*(sample+offset)).length();

            // 0-2: offsets
            ret[0] = -1.0f * (((diag.x    * A) + (offdiag.x * B) + (offdiag.y * C))/length);
            ret[1] = -1.0f * (((offdiag.x * A) + (diag.y    * B) + (offdiag.z * C))/length);
            ret[2] = -1.0f * (((offdiag.y * A) + (offdiag.z * B) + (diag.z    * C))/length);
            // 3-5: diagonals
            ret[3] = -1.0f * ((sample.x + offset.x) * A)/length;
            ret[4] = -1.0f * ((sample.y + offset.y) * B)/length;
            ret[5] = -1.0f * ((sample.z + offset.z) * C)/length;
            // 6-8: off-diagonals
            ret[6] = -1.0f * (((sample.y + offset.y) * A) + ((sample.x + offset.x) * B))/length;
            ret[7] = -1.0f * (((sample.z + offset.z) * A) + ((sample.x + offset.x) * C))/length;
            ret[8] = -1.0f * (((sample.z + offset.z) * B) + ((sample.y + offset.y) * C))/length;
            return;
        }
    };
}

uint8_t AccelCalibrator::get_num_params() {
    switch (_conf_fit_type) {
        case ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID:
            return 6;
        case ACCEL_CAL_ELLIPSOID:
            return 9;
        default:
            return 6;
    }
}
