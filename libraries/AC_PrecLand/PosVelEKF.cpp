#include "PosVelEKF.h"
#include <math.h>
#include <string.h>

// Initialize the covariance and state matrix
// This is called when the landing target is located for the first time or it was lost, then relocated
void PosVelEKF::init(float pos, float posVar, float vel, float velVar)
{
    _state[0] = pos;
    _state[1] = vel;
    _cov[0] = posVar;
    _cov[1] = 0.0f;
    _cov[2] = velVar;
}

// This functions runs the Prediction Step of the EKF
// This is called at 400 hz
void PosVelEKF::predict(float dt, float dVel, float dVelNoise)
{
    // Newly predicted state and covariance matrix at next time step
    float newState[2];
    float newCov[3];

    // We assume the following state model for this problem
    newState[0] = dt*_state[1] + _state[0];
    newState[1] = dVel + _state[1];

    /*
        The above state model is broken down into the needed EKF form:
        newState = A*OldState + B*u

        Taking jacobian with respect to state, we derive the A (or F) matrix.

        A = F = |1 dt|
                |0  1|

        B = |0|
            |1|

        u = dVel

        Covariance Matrix is ALWAYS symmetric, therefore the following matrix is assumed:
        P = Covariance Matrix = |cov[0]  cov[1]|
                                |cov[1]  cov[2]|

        newCov = F * P * F.transpose + Q
        Q = |0       0      |
            |0   dVelNoise^2|

        Post algebraic operations, and converting it to a upper triangular matrix (because of symmetry)
        The Updated covariance matrix is of the following form:
    */

    newCov[0] = dt*_cov[1] + dt*(dt*_cov[2] + _cov[1]) + _cov[0];
    newCov[1] = dt*_cov[2] + _cov[1];
    newCov[2] = ((dVelNoise)*(dVelNoise)) + _cov[2];

    // store the predicted matrices
    memcpy(_state,newState,sizeof(_state));
    memcpy(_cov,newCov,sizeof(_cov));
}

// fuse the new sensor measurement into the EKF calculations
// This is called whenever we have a new measurement available
void PosVelEKF::fusePos(float pos, float posVar)
{
    float newState[2];
    float newCov[3];

    // innovation_residual = new_sensor_readings - OldState
    const float innovation_residual = pos - _state[0];

    /*
    Measurement matrix H = [1 0] since we are directly measuring pos only
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

    Updated state estimate = OldState + K * innovation residual
    This is calculated and simplified below
    */

    newState[0] = _cov[0]*(innovation_residual)/(innovation_covariance) + _state[0];
    newState[1] = _cov[1]*(innovation_residual)/(innovation_covariance) + _state[1];

    /*
    Updated covariance matrix = (I-K*H)*P
    This is calculated and simplified below. Again, this is converted to upper triangular matrix (because of symmetry)
    */

    newCov[0] = _cov[0] * posVar / innovation_covariance;
    newCov[1] = _cov[1] * posVar / innovation_covariance;
    newCov[2] = -_cov[1] * _cov[1] / innovation_covariance + _cov[2];

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
