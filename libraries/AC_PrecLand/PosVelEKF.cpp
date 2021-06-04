#include "PosVelEKF.h"
#include <math.h>
#include <string.h>

#define abiasNoise 0.005f

// Initialize the covariance and state matrix
// This is called when the landing target is located for the first time or it was lost, then relocated
void PosVelEKF::init(float pos, float posVar, float vel, float velVar)
{
    _state[0] = pos;
    _state[1] = vel;
    _state[2] = 0.0f;

    _cov[0] = posVar;
    _cov[1] = 0.0f;
    _cov[2] = 0.0f;
    _cov[3] = velVar;
    _cov[4] = 0.0f;
    _cov[5] = 0.05f * 0.05f;
}

// This functions runs the Prediction Step of the EKF
// This is called at 400 hz
void PosVelEKF::predict(float dt, float dVel, float dVelNoise)
{
    // Newly predicted state and covariance matrix at next time step
    float newState[3];
    float newCov[6];

    // We assume the following state model for this problem
    newState[0] = dt*_state[1] + _state[0];
    newState[1] = dVel - dt*_state[2] + _state[1];
    newState[2] = _state[2];

    /*
        The above state model is broken down into the needed EKF form:
        newState = A*OldState + B*u

        Taking jacobian with respect to state, we derive the A (or F) matrix.

        A = F = |1  dt   0|
                |0  1  -dt|
                |0  0    1|

        B = |0|
            |1|
            |0|

        u = dVel

        Covariance Matrix is ALWAYS symmetric, therefore the following matrix is assumed:
        P = Covariance Matrix = |_cov[0]  _cov[1] _cov[2]|
                                |_cov[1]  _cov[3] _cov[4]|
                                |_cov[2]  _cov[4] _cov[5]|

        newCov = F * P * F.transpose + Q
        Q = |0       0             0      |
            |0   dVelNoise^2       0      |
            |0       0        abiasNoise^2|
        Post algebraic operations, and converting it to a upper triangular matrix (because of symmetry)
        The Updated covariance matrix is of the following form:
    */

    newCov[0] = dt*_cov[1] + dt*(dt*_cov[3] + _cov[1]) + _cov[0];
    newCov[1] = dt*_cov[3] - dt*(dt*_cov[4] + _cov[2]) + _cov[1];
    newCov[2] = dt*_cov[4] + _cov[2];
    newCov[3] = -dt*_cov[4] - dt*(-dt*_cov[5] + _cov[4]) + _cov[3]  + dVelNoise*dVelNoise;
    newCov[4] = -dt*_cov[5] + _cov[4];
    newCov[5] = _cov[5] + (abiasNoise*abiasNoise*dt*dt);

    // store the predicted matrices
    memcpy(_state,newState,sizeof(_state));
    memcpy(_cov,newCov,sizeof(_cov));
}

// fuse the new sensor measurement into the EKF calculations
// This is called whenever we have a new measurement available
void PosVelEKF::fusePos(float pos, float posVar)
{
    float newState[3];
    float newCov[6];

    // innovation_residual = new_sensor_readings - OldState
    const float innovation_residual = pos - _state[0];

    /*
    Measurement matrix H = [1 0 0] since we are directly measuring pos only
    Innovation Covariance = S = H * P * H.Transpose + R
    Since this is a 1-D measurement, R = posVar, which is expected variance in postion sensor reading
    Post multiplication this becomes:
    */
    const float innovation_covariance = _cov[0] + posVar;

    /*
    Next step involves calculating the kalman gain "K"
    K = P * H.transpose * S.inverse
    After solving, this comes out to be:
    K = | cov[0]/innovation_covariance |
        | cov[1]/innovation_covariance |
        | cov[2]/innovation_covariance |

    Updated state estimate = OldState + K * innovation residual
    This is calculated and simplified below
    */

    newState[0] = _cov[0]*(innovation_residual)/(innovation_covariance) + _state[0];
    newState[1] = _cov[1]*(innovation_residual)/(innovation_covariance) + _state[1];
    newState[2] = _cov[2]*(innovation_residual)/(innovation_covariance) + _state[2];

    /*
    Updated covariance matrix = (I-K*H)*P
    This is calculated and simplified below. Again, this is converted to upper triangular matrix (because of symmetry)
    */

    newCov[0] = _cov[0] * posVar / innovation_covariance;
    newCov[1] = _cov[1] * posVar / innovation_covariance;
    newCov[2] = _cov[2] * posVar / innovation_covariance;
    newCov[3] = -_cov[1] * _cov[1] / innovation_covariance + _cov[3];
    newCov[4] = -_cov[1] * _cov[2] / innovation_covariance + _cov[4];
    newCov[5] = -_cov[2] * _cov[2] / innovation_covariance + _cov[5];

    memcpy(_state,newState,sizeof(_state));
    memcpy(_cov,newCov,sizeof(_cov));
}

// Returns normalized innovation squared
float PosVelEKF::getPosNIS(float pos, float posVar)
{
    // NIS = innovation_residual.Transpose * Innovation_Covariance.Inverse * innovation_residual
    const float innovation_residual = pos - _state[0];
    const float innovation_covariance = _cov[0] + posVar;

    const float NIS = (innovation_residual*innovation_residual)/(innovation_covariance);
    return NIS;
}
