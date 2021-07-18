# Copied from https://github.com/PX4/ecl/commit/264c8c4e8681704e4719d0a03b848df8617c0863
# and modified for ArduPilot
from sympy import *
from code_gen import *
import numpy as np

# q: quaternion describing rotation from frame 1 to frame 2
# returns a rotation matrix derived form q which describes the same
# rotation
def quat2Rot(q):
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    # This form is the one normally used in flight dynamics and inertial navigation texts, eg
    # Aircraft Control and Simulation, Stevens,B.L, Lewis,F.L, Johnson,E.N, Third Edition, eqn 1.8-18
    # It does produce second order terms in the covariance prediction that can be problematic
    # with single precision processing.
    # It requires the quternion to be unit length.
    # Rot = Matrix([[q0**2 + q1**2 - q2**2 - q3**2, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
    #               [2*(q1*q2 + q0*q3), q0**2 - q1**2 + q2**2 - q3**2, 2*(q2*q3 - q0*q1)],
    #                [2*(q1*q3-q0*q2), 2*(q2*q3 + q0*q1), q0**2 - q1**2 - q2**2 + q3**2]])

    # This form removes q1 from the 0,0, q2 from the 1,1 and q3 from the 2,2 entry and results
    # in a covariance prediction that is better conditioned.
    # It requires the quaternion to be unit length and is mathematically identical
    # to the alternate form when q0**2 + q1**2 + q2**2 + q3**2 = 1
    # See https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
    Rot = Matrix([[1 - 2*(q2**2 + q3**2), 2*(q1*q2 - q0*q3)    , 2*(q1*q3 + q0*q2)    ],
                 [2*(q1*q2 + q0*q3)     , 1 - 2*(q1**2 + q3**2), 2*(q2*q3 - q0*q1)    ],
                 [2*(q1*q3-q0*q2)       , 2*(q2*q3 + q0*q1)    , 1 - 2*(q1**2 + q2**2)]])

    return Rot

def create_cov_matrix(i, j):
    if j >= i:
        # return Symbol("P(" + str(i) + "," + str(j) + ")", real=True)
        # legacy array format
        return Symbol("P[" + str(i) + "][" + str(j) + "]", real=True)
    else:
        return 0

def create_yaw_estimator_cov_matrix():
    # define a symbolic covariance matrix
    P = Matrix(3,3,create_cov_matrix)

    for index in range(3):
        for j in range(3):
            if index > j:
                P[index,j] = P[j,index]

    return P

def create_Tbs_matrix(i, j):
    # return Symbol("Tbs(" + str(i) + "," + str(j) + ")", real=True)
    # legacy array format
    return Symbol("Tbs[" + str(i) + "][" + str(j) + "]", real=True)

def quat_mult(p,q):
    r = Matrix([p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3],
                p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2],
                p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1],
                p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0]])

    return r

def create_symmetric_cov_matrix(n):
    # define a symbolic covariance matrix
    P = Matrix(n,n,create_cov_matrix)

    for index in range(n):
        for j in range(n):
            if index > j:
                P[index,j] = P[j,index]

    return P

# generate equations for observation vector innovation variances
def generate_observation_vector_innovation_variances(P,state,observation,variance,n_obs):
    H = observation.jacobian(state)
    innovation_variance = zeros(n_obs,1)
    for index in range(n_obs):
        H[index,:] = Matrix([observation[index]]).jacobian(state)
        innovation_variance[index] = H[index,:] * P * H[index,:].T + Matrix([variance])

    IV_simple = cse(innovation_variance, symbols("IV0:1000"), optimizations='basic')

    return IV_simple

# generate equations for observation Jacobian and Kalman gain
def generate_observation_equations(P,state,observation,variance,varname="HK"):
    H = Matrix([observation]).jacobian(state)
    innov_var = H * P * H.T + Matrix([variance])
    assert(innov_var.shape[0] == 1)
    assert(innov_var.shape[1] == 1)
    K = P * H.T / innov_var[0,0]
    extension="0:1000"
    var_string = varname+extension
    HK_simple = cse(Matrix([H.transpose(), K]), symbols(var_string), optimizations='basic')

    return HK_simple

# generate equations for observation vector Jacobian and Kalman gain
# n_obs is the vector dimension and must be >= 2
def generate_observation_vector_equations(P,state,observation,variance,n_obs):
    K = zeros(24,n_obs)
    H = observation.jacobian(state)
    HK = zeros(n_obs*48,1)
    for index in range(n_obs):
        H[index,:] = Matrix([observation[index]]).jacobian(state)
        innov_var = H[index,:] * P * H[index,:].T + Matrix([variance])
        assert(innov_var.shape[0] == 1)
        assert(innov_var.shape[1] == 1)
        K[:,index] = P * H[index,:].T / innov_var[0,0]
        HK[index*48:(index+1)*48,0] = Matrix([H[index,:].transpose(), K[:,index]])

    HK_simple = cse(HK, symbols("HK0:1000"), optimizations='basic')

    return HK_simple

# write single observation equations to file
def write_equations_to_file(equations,code_generator_id,n_obs):
    if (n_obs < 1):
        return

    if (n_obs == 1):
        code_generator_id.print_string("Sub Expressions")
        code_generator_id.write_subexpressions(equations[0])
        code_generator_id.print_string("Observation Jacobians")
        code_generator_id.write_matrix(Matrix(equations[1][0][0:24]), "Hfusion", False)
        code_generator_id.print_string("Kalman gains")
        code_generator_id.write_matrix(Matrix(equations[1][0][24:]), "Kfusion", False)
    else:
        code_generator_id.print_string("Sub Expressions")
        code_generator_id.write_subexpressions(equations[0])
        for axis_index in range(n_obs):
            start_index = axis_index*48
            code_generator_id.print_string("Observation Jacobians - axis %i" % axis_index)
            code_generator_id.write_matrix(Matrix(equations[1][0][start_index:start_index+24]), "Hfusion", False)
            code_generator_id.print_string("Kalman gains - axis %i" % axis_index)
            code_generator_id.write_matrix(Matrix(equations[1][0][start_index+24:start_index+48]), "Kfusion", False)

    return

# derive equations for sequential fusion of optical flow measurements
def optical_flow_observation(P,state,R_to_body,vx,vy,vz):
    flow_code_generator = CodeGenerator("./generated/flow_generated.cpp")
    range = symbols("range", real=True) # range from camera focal point to ground along sensor Z axis
    obs_var = symbols("R_LOS", real=True) # optical flow line of sight rate measurement noise variance

    # Define rotation matrix from body to sensor frame
    Tbs = Matrix(3,3,create_Tbs_matrix)

    # Calculate earth relative velocity in a non-rotating sensor frame
    relVelSensor = Tbs * R_to_body * Matrix([vx,vy,vz])

    # Divide by range to get predicted angular LOS rates relative to X and Y
    # axes. Note these are rates in a non-rotating sensor frame
    losRateSensorX = +relVelSensor[1]/range
    losRateSensorY = -relVelSensor[0]/range

    # calculate the observation Jacobian and Kalman gains for the X axis
    equations = generate_observation_equations(P,state,losRateSensorX,obs_var)

    flow_code_generator.print_string("X Axis Equations")
    write_equations_to_file(equations,flow_code_generator,1)

    # calculate the observation Jacobian and Kalman gains for the Y axis
    equations = generate_observation_equations(P,state,losRateSensorY,obs_var)

    flow_code_generator.print_string("Y Axis Equations")
    write_equations_to_file(equations,flow_code_generator,1)

    flow_code_generator.close()

    # calculate a combined result for a possible reduction in operations, but will use more stack
    observation = Matrix([relVelSensor[1]/range,-relVelSensor[0]/range])
    equations = generate_observation_vector_equations(P,state,observation,obs_var,2)
    flow_code_generator_alt = CodeGenerator("./generated/flow_generated_alt.cpp")
    write_equations_to_file(equations,flow_code_generator_alt,2)
    flow_code_generator_alt.close()

    return

# Derive equations for sequential fusion of body frame velocity measurements
def body_frame_velocity_observation(P,state,R_to_body,vx,vy,vz):
    obs_var = symbols("R_VEL", real=True) # measurement noise variance

    # Calculate earth relative velocity in a non-rotating sensor frame
    vel_bf = R_to_body * Matrix([vx,vy,vz])

    vel_bf_code_generator = CodeGenerator("./generated/vel_bf_generated.cpp")
    axes = [0,1,2]
    H_obs = vel_bf.jacobian(state) # observation Jacobians
    K_gain = zeros(24,3)
    for index in axes:
        equations = generate_observation_equations(P,state,vel_bf[index],obs_var)

        vel_bf_code_generator.print_string("axis %i" % index)
        vel_bf_code_generator.write_subexpressions(equations[0])
        vel_bf_code_generator.write_matrix(Matrix(equations[1][0][0:24]), "H_VEL", False)
        vel_bf_code_generator.write_matrix(Matrix(equations[1][0][24:]), "Kfusion", False)

    vel_bf_code_generator.close()

    # calculate a combined result for a possible reduction in operations, but will use more stack
    equations = generate_observation_vector_equations(P,state,vel_bf,obs_var,3)

    vel_bf_code_generator_alt = CodeGenerator("./generated/vel_bf_generated_alt.cpp")
    write_equations_to_file(equations,vel_bf_code_generator_alt,3)
    vel_bf_code_generator_alt.close()

# derive equations for fusion of dual antenna yaw measurement
def gps_yaw_observation(P,state,R_to_body):
    obs_var = symbols("R_YAW", real=True) # measurement noise variance
    ant_yaw = symbols("ant_yaw", real=True) # yaw angle of antenna array axis wrt X body axis

    # define antenna vector in body frame
    ant_vec_bf = Matrix([cos(ant_yaw),sin(ant_yaw),0])

    # rotate into earth frame
    ant_vec_ef = R_to_body.T * ant_vec_bf

    # Calculate the yaw angle from the projection
    observation = atan(ant_vec_ef[1]/ant_vec_ef[0])

    equations = generate_observation_equations(P,state,observation,obs_var)

    gps_yaw_code_generator = CodeGenerator("./generated/gps_yaw_generated.cpp")
    write_equations_to_file(equations,gps_yaw_code_generator,1)
    gps_yaw_code_generator.close()

    return

# derive equations for fusion of declination
def declination_observation(P,state,ix,iy):
    obs_var = symbols("R_DECL", real=True) # measurement noise variance

    # the predicted measurement is the angle wrt magnetic north of the horizontal
    # component of the measured field
    observation = atan(iy/ix)

    equations = generate_observation_equations(P,state,observation,obs_var)

    mag_decl_code_generator = CodeGenerator("./generated/mag_decl_generated.cpp")
    write_equations_to_file(equations,mag_decl_code_generator,1)
    mag_decl_code_generator.close()

    return

# derive equations for fusion of lateral body acceleration (multirotors only)
def body_frame_accel_observation(P,state,R_to_body,vx,vy,vz,wx,wy):
    obs_var = symbols("R_ACC", real=True) # measurement noise variance
    Kaccx = symbols("Kaccx", real=True) # measurement noise variance
    Kaccy = symbols("Kaccy", real=True) # measurement noise variance

    # use relationship between airspeed along the X and Y body axis and the
    # drag to predict the lateral acceleration for a multirotor vehicle type
    # where propulsion forces are generated primarily along the Z body axis

    vrel = R_to_body*Matrix([vx-wx,vy-wy,vz]) # predicted wind relative velocity

    # Use this nonlinear model for the prediction in the implementation only
    # It uses a ballistic coefficient for each axis
    # accXpred = -0.5*rho*vrel[0]*vrel[0]*BCXinv # predicted acceleration measured along X body axis
    # accYpred = -0.5*rho*vrel[1]*vrel[1]*BCYinv # predicted acceleration measured along Y body axis

    # Use a simple viscous drag model for the linear estimator equations
    # Use the the derivative from speed to acceleration averaged across the
    # speed range. This avoids the generation of a dirac function in the derivation
    # The nonlinear equation will be used to calculate the predicted measurement in implementation
    observation = Matrix([-Kaccx*vrel[0],-Kaccy*vrel[1]])

    acc_bf_code_generator  = CodeGenerator("./generated/acc_bf_generated.cpp")
    H = observation.jacobian(state)
    K = zeros(24,2)
    axes = [0,1]
    for index in axes:
        equations = generate_observation_equations(P,state,observation[index],obs_var)
        acc_bf_code_generator.print_string("Axis %i equations" % index)
        write_equations_to_file(equations,acc_bf_code_generator,1)

    acc_bf_code_generator.close()

    # calculate a combined result for a possible reduction in operations, but will use more stack
    equations = generate_observation_vector_equations(P,state,observation,obs_var,2)

    acc_bf_code_generator_alt  = CodeGenerator("./generated/acc_bf_generated_alt.cpp")
    write_equations_to_file(equations,acc_bf_code_generator_alt,3)
    acc_bf_code_generator_alt.close()

    return

# yaw fusion
def yaw_observation(P,state,R_to_earth):
    yaw_code_generator = CodeGenerator("./generated/yaw_generated.cpp")

    # Derive observation Jacobian for fusion of 321 sequence yaw measurement
    # Calculate the yaw (first rotation) angle from the 321 rotation sequence
    # Provide alternative angle that avoids singularity at +-pi/2 yaw
    angMeasA = atan(R_to_earth[1,0]/R_to_earth[0,0])
    H_YAW321_A = Matrix([angMeasA]).jacobian(state)
    H_YAW321_A_simple = cse(H_YAW321_A, symbols('SA0:200'))

    angMeasB = pi/2 - atan(R_to_earth[0,0]/R_to_earth[1,0])
    H_YAW321_B = Matrix([angMeasB]).jacobian(state)
    H_YAW321_B_simple = cse(H_YAW321_B, symbols('SB0:200'))

    yaw_code_generator.print_string("calculate 321 yaw observation matrix - option A")
    yaw_code_generator.write_subexpressions(H_YAW321_A_simple[0])
    yaw_code_generator.write_matrix(Matrix(H_YAW321_A_simple[1]).T, "H_YAW", False)

    yaw_code_generator.print_string("calculate 321 yaw observation matrix - option B")
    yaw_code_generator.write_subexpressions(H_YAW321_B_simple[0])
    yaw_code_generator.write_matrix(Matrix(H_YAW321_B_simple[1]).T, "H_YAW", False)

    # Derive observation Jacobian for fusion of 312 sequence yaw measurement
    # Calculate the yaw (first rotation) angle from an Euler 312 sequence
    # Provide alternative angle that avoids singularity at +-pi/2 yaw
    angMeasA = atan(-R_to_earth[0,1]/R_to_earth[1,1])
    H_YAW312_A = Matrix([angMeasA]).jacobian(state)
    H_YAW312_A_simple = cse(H_YAW312_A, symbols('SA0:200'))

    angMeasB = pi/2 - atan(-R_to_earth[1,1]/R_to_earth[0,1])
    H_YAW312_B = Matrix([angMeasB]).jacobian(state)
    H_YAW312_B_simple = cse(H_YAW312_B, symbols('SB0:200'))

    yaw_code_generator.print_string("calculate 312 yaw observation matrix - option A")
    yaw_code_generator.write_subexpressions(H_YAW312_A_simple[0])
    yaw_code_generator.write_matrix(Matrix(H_YAW312_A_simple[1]).T, "H_YAW", False)

    yaw_code_generator.print_string("calculate 312 yaw observation matrix - option B")
    yaw_code_generator.write_subexpressions(H_YAW312_B_simple[0])
    yaw_code_generator.write_matrix(Matrix(H_YAW312_B_simple[1]).T, "H_YAW", False)

    yaw_code_generator.close()

    return

# 3D magnetometer fusion
def mag_observation_variance(P,state,R_to_body,i,ib):
    obs_var = symbols("R_MAG", real=True)  # magnetometer measurement noise variance

    m_mag = R_to_body * i + ib

    # separate calculation of innovation variance equations for the y and z axes
    m_mag[0]=0
    innov_var_equations = generate_observation_vector_innovation_variances(P,state,m_mag,obs_var,3)
    mag_innov_var_code_generator = CodeGenerator("./generated/3Dmag_innov_var_generated.cpp")
    write_equations_to_file(innov_var_equations,mag_innov_var_code_generator,3)
    mag_innov_var_code_generator.close()

    return

# 3D magnetometer fusion
def mag_observation(P,state,R_to_body,i,ib):
    obs_var = symbols("R_MAG", real=True)  # magnetometer measurement noise variance

    m_mag = R_to_body * i + ib

    # calculate a separate set of equations for each axis
    mag_code_generator = CodeGenerator("./generated/3Dmag_generated.cpp")

    axes = [0,1,2]
    label="HK"
    for index in axes:
        if (index==0):
            label="HKX"
        elif (index==1):
            label="HKY"
        elif (index==2):
            label="HKZ"
        else:
            return
        equations = generate_observation_equations(P,state,m_mag[index],obs_var,varname=label)
        mag_code_generator.print_string("Axis %i equations" % index)
        write_equations_to_file(equations,mag_code_generator,1)

    mag_code_generator.close()

    # calculate a combined set of equations for a possible reduction in operations, but will use slighlty more stack
    equations = generate_observation_vector_equations(P,state,m_mag,obs_var,3)

    mag_code_generator_alt  = CodeGenerator("./generated/3Dmag_generated_alt.cpp")
    write_equations_to_file(equations,mag_code_generator_alt,3)
    mag_code_generator_alt.close()

    return

# airspeed fusion
def tas_observation(P,state,vx,vy,vz,wx,wy):
    obs_var = symbols("R_TAS", real=True) # true airspeed measurement noise variance

    observation = sqrt((vx-wx)*(vx-wx)+(vy-wy)*(vy-wy)+vz*vz)

    equations = generate_observation_equations(P,state,observation,obs_var)

    tas_code_generator = CodeGenerator("./generated/tas_generated.cpp")
    write_equations_to_file(equations,tas_code_generator,1)
    tas_code_generator.close()

    return

# sideslip fusion
def beta_observation(P,state,R_to_body,vx,vy,vz,wx,wy):
    obs_var = symbols("R_BETA", real=True) # sideslip measurement noise variance

    v_rel_ef = Matrix([vx-wx,vy-wy,vz])
    v_rel_bf = R_to_body * v_rel_ef
    observation = v_rel_bf[1]/v_rel_bf[0]

    equations = generate_observation_equations(P,state,observation,obs_var)

    beta_code_generator = CodeGenerator("./generated/beta_generated.cpp")
    write_equations_to_file(equations,beta_code_generator,1)
    beta_code_generator.close()

    return

# yaw estimator prediction and observation code
def yaw_estimator():
    dt = symbols("dt", real=True)  # dt (sec)
    psi = symbols("psi", real=True)  # yaw angle of body frame wrt earth frame
    vn, ve = symbols("vn ve", real=True)  # velocity in world frame (north/east) - m/sec
    daz = symbols("daz", real=True)  # IMU z axis delta angle measurement in body axes - rad
    dazVar = symbols("dazVar", real=True) # IMU Z axis delta angle measurement variance (rad^2)
    dvx, dvy = symbols("dvx dvy", real=True)  # IMU x and y axis delta velocity measurement in body axes - m/sec
    dvxVar, dvyVar = symbols("dvxVar dvyVar", real=True)   # IMU x and y axis delta velocity measurement variance (m/s)^2

    # derive the body to nav direction transformation matrix
    Tbn = Matrix([[cos(psi) , -sin(psi)],
                [sin(psi) ,  cos(psi)]])

    # attitude update equation
    psiNew = psi + daz

    # velocity update equations
    velNew = Matrix([vn,ve]) + Tbn*Matrix([dvx,dvy])

    # Define the state vectors
    stateVector = Matrix([vn,ve,psi])

    # Define vector of process equations
    newStateVector = Matrix([velNew,psiNew])

    # Calculate state transition matrix
    F = newStateVector.jacobian(stateVector)

    # Derive the covariance prediction equations
    # Error growth in the inertial solution is assumed to be driven by 'noise' in the delta angles and
    # velocities, after bias effects have been removed.

    # derive the control(disturbance) influence matrix from IMU noise to state noise
    G = newStateVector.jacobian(Matrix([dvx,dvy,daz]))

    # derive the state error matrix
    distMatrix = Matrix([[dvxVar , 0 , 0],
                        [0 , dvyVar , 0],
                        [0 , 0 , dazVar]])

    Q = G * distMatrix * G.T

    # propagate covariance matrix
    P = create_yaw_estimator_cov_matrix()

    P_new = F * P * F.T + Q

    P_new_simple = cse(P_new, symbols("S0:1000"), optimizations='basic')

    yaw_estimator_covariance_generator = CodeGenerator("./generated/yaw_estimator_covariance_prediction_generated.cpp")
    yaw_estimator_covariance_generator.print_string("Equations for covariance matrix prediction")
    yaw_estimator_covariance_generator.write_subexpressions(P_new_simple[0])
    yaw_estimator_covariance_generator.write_matrix(Matrix(P_new_simple[1]), "_ekf_gsf[model_index].P", True)
    yaw_estimator_covariance_generator.close()

    # derive the covariance update equation for a NE velocity observation
    velObsVar = symbols("velObsVar", real=True) # velocity observation variance (m/s)^2
    H = Matrix([[1,0,0],
                [0,1,0]])

    R = Matrix([[velObsVar , 0],
                [0 , velObsVar]])

    S = H * P * H.T + R
    S_det_inv = 1 / S.det()
    S_inv = S.inv()
    K = (P * H.T) * S_inv
    P_new = P - K * S * K.T

    # optimize code
    t, [S_det_inv_s, S_inv_s, K_s, P_new_s] = cse([S_det_inv, S_inv, K, P_new], symbols("t0:1000"), optimizations='basic')

    yaw_estimator_observation_generator = CodeGenerator("./generated/yaw_estimator_measurement_update_generated.cpp")
    yaw_estimator_observation_generator.print_string("Intermediate variables")
    yaw_estimator_observation_generator.write_subexpressions(t)
    yaw_estimator_observation_generator.print_string("Equations for NE velocity innovation variance's determinante inverse")
    yaw_estimator_observation_generator.write_matrix(Matrix([[S_det_inv_s]]), "_ekf_gsf[model_index].S_det_inverse", False)
    yaw_estimator_observation_generator.print_string("Equations for NE velocity innovation variance inverse")
    yaw_estimator_observation_generator.write_matrix(Matrix(S_inv_s), "_ekf_gsf[model_index].S_inverse", True)
    yaw_estimator_observation_generator.print_string("Equations for NE velocity Kalman gain")
    yaw_estimator_observation_generator.write_matrix(Matrix(K_s), "K", False)
    yaw_estimator_observation_generator.print_string("Equations for covariance matrix update")
    yaw_estimator_observation_generator.write_matrix(Matrix(P_new_s), "_ekf_gsf[model_index].P", True)
    yaw_estimator_observation_generator.close()

def quaternion_error_propagation():
    # define quaternion state vector
    q0, q1, q2, q3 = symbols("q0 q1 q2 q3", real=True)
    q = Matrix([q0, q1, q2, q3])

    # define truth gravity unit vector in body frame
    R_to_earth = quat2Rot(q)
    R_to_body = R_to_earth.T
    gravity_ef = Matrix([0,0,1])
    gravity_bf = R_to_body * gravity_ef

    # define perturbations to quaternion state vector q
    dq0, dq1, dq2, dq3 = symbols("dq0 dq1 dq2 dq3", real=True)
    q_delta = Matrix([dq0, dq1, dq2, dq3])

    # apply perturbations
    q_perturbed = q + q_delta

    # gravity unit vector in body frame after quaternion perturbation
    R_to_earth_perturbed = quat2Rot(q_perturbed)
    R_to_body_perturbed = R_to_earth_perturbed.T
    gravity_bf_perturbed = R_to_body_perturbed * gravity_ef

    # calculate the angular difference between the perturbed and unperturbed body frame gravity unit vectors
    # assuming small angles
    tilt_error_bf = gravity_bf.cross(gravity_bf_perturbed)

    # calculate the derivative of the perturbation rotation vector wrt the quaternion perturbations
    J = tilt_error_bf.jacobian(q_delta)

    # remove second order terms
    # we don't want the error deltas to appear in the final result
    J.subs(dq0,0)
    J.subs(dq1,0)
    J.subs(dq2,0)
    J.subs(dq3,0)

    # define covaraince matrix for quaternion states
    P = create_symmetric_cov_matrix(4)

    # discard off diagonals
    P_diag = diag(P[0,0],P[1,1],P[2,2],P[3,3])

    # rotate quaternion covariances into rotation vector state space
    P_rot_vec = J * P_diag * J.transpose()
    P_rot_vec_simple = cse(P_rot_vec, symbols("PS0:400"), optimizations='basic')

    quat_code_generator = CodeGenerator("./generated/tilt_error_cov_mat_generated.cpp")
    quat_code_generator.write_subexpressions(P_rot_vec_simple[0])
    quat_code_generator.write_matrix(Matrix(P_rot_vec_simple[1]), "tiltErrCovMat", False, "[", "]")
    quat_code_generator.close()

def generate_code():
    print('Starting code generation:')
    print('Creating symbolic variables ...')

    dt = symbols("dt", real=True)  # dt
    g = symbols("g", real=True) # gravity constant

    r_hor_vel = symbols("R_hor_vel", real=True) # horizontal velocity noise variance
    r_ver_vel = symbols("R_vert_vel", real=True) # vertical velocity noise variance
    r_hor_pos = symbols("R_hor_pos", real=True) # horizontal position noise variance

    # inputs, integrated gyro measurements
    # delta angle x y z
    d_ang_x, d_ang_y, d_ang_z = symbols("dax day daz", real=True)  # delta angle x
    d_ang = Matrix([d_ang_x, d_ang_y, d_ang_z])

    # inputs, integrated accelerometer measurements
    # delta velocity x y z
    d_v_x, d_v_y, d_v_z = symbols("dvx dvy dvz", real=True)
    d_v = Matrix([d_v_x, d_v_y,d_v_z])

    u = Matrix([d_ang, d_v])

    # input noise
    d_ang_x_var, d_ang_y_var, d_ang_z_var = symbols("daxVar dayVar dazVar", real=True)

    d_v_x_var, d_v_y_var, d_v_z_var = symbols("dvxVar dvyVar dvzVar", real=True)

    var_u = Matrix.diag(d_ang_x_var, d_ang_y_var, d_ang_z_var, d_v_x_var, d_v_y_var, d_v_z_var)

    # define state vector

    # attitude quaternion
    qw, qx, qy, qz = symbols("q0 q1 q2 q3", real=True)
    q = Matrix([qw,qx,qy,qz])
    R_to_earth = quat2Rot(q)
    R_to_body = R_to_earth.T

    # velocity in NED local frame (north, east, down)
    vx, vy, vz = symbols("vn ve vd", real=True)
    v = Matrix([vx,vy,vz])

    # position in NED local frame (north, east, down)
    px, py, pz = symbols("pn pe pd", real=True)
    p = Matrix([px,py,pz])

    # delta angle bias x y z
    d_ang_bx, d_ang_by, d_ang_bz = symbols("dax_b day_b daz_b", real=True)
    d_ang_b = Matrix([d_ang_bx, d_ang_by, d_ang_bz])
    d_ang_true = d_ang - d_ang_b

    # delta velocity bias x y z
    d_vel_bx, d_vel_by, d_vel_bz = symbols("dvx_b dvy_b dvz_b", real=True)
    d_vel_b = Matrix([d_vel_bx, d_vel_by, d_vel_bz])
    d_vel_true = d_v - d_vel_b

    # earth magnetic field vector x y z
    ix, iy, iz = symbols("magN magE magD", real=True)
    i = Matrix([ix,iy,iz])

    # earth magnetic field bias in body frame
    ibx, iby, ibz = symbols("ibx iby ibz", real=True)

    ib = Matrix([ibx,iby,ibz])

    # wind in local NE frame (north, east)
    wx, wy = symbols("vwn, vwe", real=True)
    w = Matrix([wx,wy])

    # state vector at arbitrary time t
    state = Matrix([q, v, p, d_ang_b, d_vel_b, i, ib, w])

    print('Defining state propagation ...')
    # kinematic processes driven by IMU 'control inputs'
    q_new = quat_mult(q, Matrix([1, 0.5 * d_ang_true[0],  0.5 * d_ang_true[1],  0.5 * d_ang_true[2]]))
    v_new = v + R_to_earth * d_vel_true + Matrix([0,0,g]) * dt
    p_new = p + v * dt

    # static processes
    d_ang_b_new = d_ang_b
    d_vel_b_new = d_vel_b
    i_new = i
    ib_new = ib
    w_new = w

    # predicted state vector at time t + dt
    state_new = Matrix([q_new, v_new, p_new, d_ang_b_new, d_vel_b_new, i_new, ib_new, w_new])

    print('Computing state propagation jacobian ...')
    A = state_new.jacobian(state)
    G = state_new.jacobian(u)

    P = create_symmetric_cov_matrix(24)

    print('Computing covariance propagation ...')
    P_new = A * P * A.T + G * var_u * G.T

    for index in range(24):
        for j in range(24):
            if index > j:
                P_new[index,j] = 0

    print('Simplifying covariance propagation ...')
    P_new_simple = cse(P_new, symbols("PS0:400"), optimizations='basic')

    print('Writing covariance propagation to file ...')
    cov_code_generator = CodeGenerator("./generated/covariance_generated.cpp")
    cov_code_generator.print_string("Equations for covariance matrix prediction, without process noise!")
    cov_code_generator.write_subexpressions(P_new_simple[0])
    cov_code_generator.write_matrix(Matrix(P_new_simple[1]), "nextP", True, "[", "]")

    cov_code_generator.close()


    # derive autocode for other methods
    print('Computing tilt error covariance matrix ...')
    quaternion_error_propagation()
    print('Generating heading observation code ...')
    yaw_observation(P,state,R_to_earth)
    print('Generating gps heading observation code ...')
    gps_yaw_observation(P,state,R_to_body)
    print('Generating mag observation code ...')
    mag_observation_variance(P,state,R_to_body,i,ib)
    mag_observation(P,state,R_to_body,i,ib)
    print('Generating declination observation code ...')
    declination_observation(P,state,ix,iy)
    print('Generating airspeed observation code ...')
    tas_observation(P,state,vx,vy,vz,wx,wy)
    print('Generating sideslip observation code ...')
    beta_observation(P,state,R_to_body,vx,vy,vz,wx,wy)
    print('Generating optical flow observation code ...')
    optical_flow_observation(P,state,R_to_body,vx,vy,vz)
    print('Generating body frame velocity observation code ...')
    body_frame_velocity_observation(P,state,R_to_body,vx,vy,vz)
    print('Generating body frame acceleration observation code ...')
    body_frame_accel_observation(P,state,R_to_body,vx,vy,vz,wx,wy)
    print('Generating yaw estimator code ...')
    yaw_estimator()
    print('Code generation finished!')


if __name__ == "__main__":
    generate_code()
