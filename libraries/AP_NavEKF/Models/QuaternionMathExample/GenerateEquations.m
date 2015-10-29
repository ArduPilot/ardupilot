% IMPORTANT - This script requires the Matlab symbolic toolbox

% Author:  Paul Riseborough
% Last Modified: 16 Feb 2014

% Derivation of a navigation EKF using a local NED earth Tangent Frame for
% the initial alignment and gyro bias estimation from a moving platform
% Uses quaternions for attitude propagation

% State vector:
% quaternions
% Velocity - North, East, Down (m/s)
% Delta Angle bias - X,Y,Z (rad)

% Observations:
% NED velocity - N,E,D (m/s)
% body fixed magnetic field vector - X,Y,Z

% Time varying parameters:
% XYZ delta angle measurements in body axes - rad
% XYZ delta velocity measurements in body axes - m/sec
% magnetic declination
clear all;

%% define symbolic variables and constants
syms dax day daz real % IMU delta angle measurements in body axes - rad
syms dvx dvy dvz real % IMU delta velocity measurements in body axes - m/sec
syms q0 q1 q2 q3 real % quaternions defining attitude of body axes relative to local NED
syms vn ve vd real % NED velocity - m/sec
syms dax_b day_b daz_b real % delta angle bias - rad
syms dvx_b dvy_b dvz_b real % delta velocity bias - m/sec
syms dt real % IMU time step - sec
syms gravity real % gravity  - m/sec^2
syms daxNoise dayNoise dazNoise dvxNoise dvyNoise dvzNoise real; % IMU delta angle and delta velocity measurement noise
syms vwn vwe real; % NE wind velocity - m/sec
syms magX magY magZ real; % XYZ body fixed magnetic field measurements - milligauss
syms magN magE magD real; % NED earth fixed magnetic field components - milligauss
syms R_VN R_VE R_VD real % variances for NED velocity measurements - (m/sec)^2
syms R_MAG real  % variance for magnetic flux measurements - milligauss^2

%% define the process equations

% define the measured Delta angle and delta velocity vectors
dAngMeas = [dax; day; daz];
dVelMeas = [dvx; dvy; dvz];

% define the delta angle bias errors
dAngBias = [dax_b; day_b; daz_b];

% define the quaternion rotation vector for the state estimate
quat = [q0;q1;q2;q3];

% derive the truth body to nav direction cosine matrix
Tbn = Quat2Tbn(quat);

% define the truth delta angle
% ignore coning acompensation as these effects are negligible in terms of 
% covariance growth for our application and grade of sensor
dAngTruth = dAngMeas - dAngBias;

% Define the truth delta velocity
dVelTruth = dVelMeas;

% define the attitude update equations
% use a first order expansion of rotation to calculate the quaternion increment
% acceptable for propagation of covariances
deltaQuat = [1;
    0.5*dAngTruth(1);
    0.5*dAngTruth(2);
    0.5*dAngTruth(3);
    ];
quatNew = QuatMult(quat,deltaQuat);

% define the velocity update equations
% ignore coriolis terms for linearisation purposes
vNew = [vn;ve;vd] + [0;0;gravity]*dt + Tbn*dVelTruth;

% define the IMU bias error update equations
dabNew = [dax_b; day_b; daz_b];

% Define the state vector & number of states
stateVector = [quat;vn;ve;vd;dAngBias];
nStates=numel(stateVector);

%% derive the covariance prediction equation
% This reduces the number of floating point operations by a factor of 6 or
% more compared to using the standard matrix operations in code

% Define the control (disturbance) vector. Use the measured delta angles
% and velocities (not truth) to simplify the derivation
distVector = [dAngMeas;dVelMeas];

% derive the control(disturbance) influence matrix
G = jacobian([quatNew;vNew;dabNew], distVector);
f = matlabFunction(G,'file','calcG.m');

% derive the state error matrix
imuNoise = diag([daxNoise dayNoise dazNoise dvxNoise dvyNoise dvzNoise]);
Q = G*imuNoise*transpose(G);
f = matlabFunction(Q,'file','calcQ.m');

% derive the state transition matrix
F = jacobian([quatNew;vNew;dabNew], stateVector);
f = matlabFunction(F,'file','calcF.m');

%% derive equations for fusion of magnetic deviation measurement
% rotate body measured field into earth axes
magMeasNED = Tbn*[magX;magY;magZ]; 
% the predicted measurement is the angle wrt true north of the horizontal
% component of the measured field
angMeas = tan(magMeasNED(2)/magMeasNED(1));
H_MAG = jacobian(angMeas,stateVector); % measurement Jacobian
f = matlabFunction(H_MAG,'file','calcH_MAG.m');
