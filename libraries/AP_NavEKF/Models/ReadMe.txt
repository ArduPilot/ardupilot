Matlab derivations and simulations relating to development of the small EKF containing the following material:

testData: bench and flight test data used by the various estimator test harnesses
QuaternionMathExample: Small EKF using the same quaternion attitude representation as the main EKF and with a in-flight alignment test using real data. The inability to perform and in-flight alignment demonstrated by this example shows why we have change to a different attitude representation for the samll EKF.
AttErrVecExample: Small EKF using a new error vector representation for the vehicle attitude and subjected to in-flight alignment tests using both simulated and real data. The robust alignment achieved shows why this new math has been selected for use in the small EKF.
GimbalEstimatorExample: A simulation of 3 axis stabilised gimbal estimator showing application of the small EKF combined with a high rate predictor running on the gimbal that is corrected using the EKF attitude estimates.
Common: Common Matlab functions