#include <AP_ADRC/AP_ADRC.h>
#include <AP_Math/AP_Math.h>

const AP_Param::GroupInfo AP_ADRC::var_info[] = {

    // @Param: WC
    // @DisplayName: ADRC control bandwidth(rad/s)
    AP_GROUPINFO("WC",1,AP_ADRC,wc_,10),

    // @Param: WO
    // @DisplayName: ADRC ESO bandwidth(rad/s)
    AP_GROUPINFO("WO",2,AP_ADRC,wo_,15),

    // @Param: B0
    // @DisplayName: ADRC control input gain
    AP_GROUPINFO("B0",3,AP_ADRC,b0_,10),

    // @Param: DELT
    // @DisplayName: ADRC control linear zone length
    AP_GROUPINFO("DELT",4,AP_ADRC,delta_,1.0),

    // @Param: ORDR
    // @DisplayName: ADRC control model order
    AP_GROUPINFO("ORDR",5,AP_ADRC,order_,1),

    // @Param: LM
    // @DisplayName: ADRC control output limit
    AP_GROUPINFO("LM",6,AP_ADRC,limit_,1.0f),

    AP_GROUPEND
};

AP_ADRC::AP_ADRC(float dt) : AC_PID(1, 1, 1, 1.0f, 1, 1, 1.0f, 1, dt)
{
    AP_Param::setup_object_defaults(this, var_info);
    dt_ = dt;

    // reset input filter to first value received
    flags_.reset_filter_ = true;

    memset(&_pid_info, 0, sizeof(_pid_info));
}

float AP_ADRC::update_all(float target, float measurement, bool limit)
{
    // don't process inf or NaN
    if (!isfinite(target) || !isfinite(measurement)) {
        return 0.0f;
    }

    if(flags_.reset_filter_){
        flags_.reset_filter_ = false;
    }

        // Get controller error
        float e1 = target - z1_;

        // control derivation error
        float e2 = -z2_;

        // state estimation error
        float e  = z1_ - measurement;

        float output = 0.0f;
        float output_limited = 0;
        float dmod = 1.0f;

        float sigma = 1.0f/(sq(e) + 1.0f);

        switch (order_)
        {
        case 1:
            {
                // Nonlinear control law
                output = (wc_ * fal(e1,0.5f,delta_)  - sigma * z2_)/b0_;

                // Limit output
                if(is_zero(limit_.get())){
                    output_limited = output;
                }else{
                    output_limited = constrain_float(output * dmod,-limit_,limit_);
                }

                // State estimation
                float fe = fal(e,0.5,delta_);
                float beta1 = 2 * wo_;
                float beta2 = wo_ * wo_;
                z1_ = z1_ + dt_ * (z2_ - beta1*e + b0_ * output_limited);
                z2_ = z2_ + dt_ * (-beta2 * fe);

                _pid_info.P      = z1_;
                _pid_info.I      = z2_;
                _pid_info.D      = z3_;
                _pid_info.FF     = output_limited;
            }
            break;
        case 2:
            {
                float kp  = sq(wc_);
                float kd  = 2*wc_;

                // Nonlinear control law
                output = (kp * fal(e1,0.5f,delta_) + kd * fal(e2,0.25,delta_) - sigma * z3_)/b0_;

                // Limit output
                if(is_zero(limit_.get())){
                    output_limited = output * dmod;
                }else{
                    output_limited = constrain_float(output * dmod,-limit_,limit_);
                }

                // State estimation
                float beta1 = 3 * wo_;
                float beta2 = 3 * wo_ * wo_;
                float beta3 = wo_ * wo_ * wo_;
                float fe  = fal(e,0.5,delta_);
                float fe1 = fal(e,0.25,delta_);
                z1_  = z1_ + dt_ * (z2_ - beta1 * e);
                z2_  = z2_ + dt_ * (z3_ - beta2 * fe + b0_ * output_limited);
                z3_  = z3_ + dt_ * (- beta3 * fe1);

                _pid_info.P      = z1_;
                _pid_info.I      = z2_;
                _pid_info.D      = z3_;
                _pid_info.FF     = output_limited;
            }
            break;
        default:
            output_limited = 0.0f;
            break;
        }


    // For loggers
    _pid_info.target = target;
    _pid_info.actual = measurement;
    _pid_info.error  = target - measurement;

    return output_limited;
}



void AP_ADRC::set_dt(float dt)
{
    dt_ = dt;
}

void AP_ADRC::reset_eso(float measurement)
{
    z1_ = measurement;
    z2_ = 0.0f;
    z3_ = 0.0f;
}

float AP_ADRC::sign(float x)
{
    if (x < 0) return -1;
    else return 1;
}

float AP_ADRC::fal(float e, float alpha, float delta)
{
    if(fabsf(e) < delta){
        return e / (powf(delta, 1.0f-alpha));
    }else{
        return powf(fabsf(e), alpha) * sign(e);
    }
}
