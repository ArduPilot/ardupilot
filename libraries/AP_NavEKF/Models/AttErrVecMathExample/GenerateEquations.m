% IMPORTANT - This script requires the Matlab symbolic toolbox

% Author:  Paul Riseborough
% Last Modified: 16 Feb 2015

% Derivation of a navigation EKF using a local NED earth Tangent Frame for
% the initial alignment and gyro bias estimation from a moving platform
% Based on use of a rotation vector for attitude estimation as described
% here:
%
% Mark E. Pittelkau.  "Rotation Vector in Attitude Estimation", 
% Journal of Guidance, Control, and Dynamics, Vol. 26, No. 6 (2003), 
% pp. 855-860.
% 
% The benefits for use of rotation error vector over use of a four parameter 
% quaternion representation of the estiamted orientation are:
% a) Reduced computational load 
% b) Improved stability 
% c) The ability to recover faster from large orientation errors. This 
% makes this filter particularly suitable where the initial alignment is 
% uncertain 

% State vector:
% error rotation vector
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
syms rotErr1 rotErr2 rotErr3 real; % error rotation vector

%% define the process equations

% define the measured Delta angle and delta velocity vectors
dAngMeas = [dax; day; daz];
dVelMeas = [dvx; dvy; dvz];

% define the delta angle bias errors
dAngBias = [dax_b; day_b; daz_b];

% define the quaternion rotation vector for the state estimate
estQuat = [q0;q1;q2;q3];

% define the attitude error rotation vector, where error = truth - estimate
errRotVec = [rotErr1;rotErr2;rotErr3];

% define the attitude error quaternion using a first order linearisation
errQuat = [1;0.5*errRotVec];

% Define the truth quaternion as the estimate + error
truthQuat = QuatMult(estQuat, errQuat);

% derive the truth body to nav direction cosine matrix
Tbn = Quat2Tbn(truthQuat);

% define the truth delta angle
% ignore coning acompensation as these effects are negligible in terms of 
% covariance growth for our application and grade of sensor
dAngTruth = dAngMeas - dAngBias - [daxNoise;dayNoise;dazNoise];

% Define the truth delta velocity
dVelTruth = dVelMeas - [dvxNoise;dvyNoise;dvzNoise];

% define the attitude update equations
% use a first order expansion of rotation to calculate the quaternion increment
% acceptable for propagation of covariances
deltaQuat = [1;
    0.5*dAngTruth(1);
    0.5*dAngTruth(2);
    0.5*dAngTruth(3);
    ];
truthQuatNew = QuatMult(truthQuat,deltaQuat);
% calculate the updated attitude error quaternion with respect to the previous estimate
errQuatNew = QuatDivide(truthQuatNew,estQuat);
% change to a rotaton vector - this is the error rotation vector updated state
errRotNew = 2 * [errQuatNew(2);errQuatNew(3);errQuatNew(4)];

% define the velocity update equations
% ignore coriolis terms for linearisation purposes
vNew = [vn;ve;vd] + [0;0;gravity]*dt + Tbn*dVelTruth;

% define the IMU bias error update equations
dabNew = [dax_b; day_b; daz_b];

% Define the state vector & number of states
stateVector = [errRotVec;vn;ve;vd;dAngBias];
nStates=numel(stateVector);

%% derive the covariance prediction equation
% This reduces the number of floating point operations by a factor of 6 or
% more compared to using the standard matrix operations in code

% Define the control (disturbance) vector. Error growth in the inertial
% solution is assumed to be driven by 'noise' in the delta angles and
% velocities, after bias effects have been removed. This is OK because we
% have sensor bias accounted for in the state equations.
distVector = [daxNoise;dayNoise;dazNoise;dvxNoise;dvyNoise;dvzNoise];

% derive the control(disturbance) influence matrix
G = jacobian([errRotNew;vNew;dabNew], distVector);
G = subs(G, {'rotErr1', 'rotErr2', 'rotErr3'}, {0,0,0});

% derive the state error matrix
distMatrix = diag(distVector);
Q = G*distMatrix*transpose(G);
f = matlabFunction(Q,'file','calcQ.m');

% derive the state transition matrix
vNew = subs(vNew,{'daxNoise','dayNoise','dazNoise','dvxNoise','dvyNoise','dvzNoise'}, {0,0,0,0,0,0});
errRotNew = subs(errRotNew,{'daxNoise','dayNoise','dazNoise','dvxNoise','dvyNoise','dvzNoise'}, {0,0,0,0,0,0});
F = jacobian([errRotNew;vNew;dabNew], stateVector);
F = subs(F, {'rotErr1', 'rotErr2', 'rotErr3'}, {0,0,0});
f = matlabFunction(F,'file','calcF.m');

% define a symbolic covariance matrix using strings to represent 
% '_l_' to represent '( '
% '_c_' to represent ,
% '_r_' to represent ')' 
% these can be substituted later to create executable code
% for rowIndex = 1:nStates
%     for colIndex = 1:nStates
%         eval(['syms OP_l_',num2str(rowIndex),'_c_',num2str(colIndex), '_r_ real']);
%         eval(['P(',num2str(rowIndex),',',num2str(colIndex), ') = OP_l_',num2str(rowIndex),'_c_',num2str(colIndex),'_r_;']);
%     end
% end

% Derive the predicted covariance matrix using the standard equation
% nextP = F*P*transpose(F) + Q;
% f = matlabFunction(nextP,'file','calcP.m');
%% derive equations for fusion of magnetic deviation measurement
% rotate body measured field into earth axes
magMeasNED = Tbn*[magX;magY;magZ]; 
% the predicted measurement is the angle wrt true north of the horizontal
% component of the measured field
angMeas = tan(magMeasNED(2)/magMeasNED(1));
H_MAG = jacobian(angMeas,stateVector); % measurement Jacobian
H_MAG = subs(H_MAG, {'rotErr1', 'rotErr2', 'rotErr3'}, {0,0,0});
f = matlabFunction(H_MAG,'file','calcH_MAG.m');
