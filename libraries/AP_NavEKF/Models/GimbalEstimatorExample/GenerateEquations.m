% IMPORTANT - This script requires the Matlab symbolic toolbox

% Author:  Paul Riseborough
% Last Modified: 16 Feb 2015

% Derivation of a 3-axis gimbal attitude estimator using a local NED earth Tangent
% Frame. Based on use of a rotation vector for attitude estimation as described
% here:

% Mark E. Pittelkau.  "Rotation Vector in Attitude Estimation", 
% Journal of Guidance, Control, and Dynamics, Vol. 26, No. 6 (2003), 
% pp. 855-860.

% The gimbal is assumed to have the following characteristics:

% A three axis gimbal having a fixed top plate mounted to the vehicle body with a magnetometer
% Yaw, roll and pitch degrees of freedom (yaw, roll, pitch Euler sequence)
% with angle measurements on each gimbal axis
% IMU measuring delta angles and delta velocites mounted on the
% camera/sensor assembly
% When the gimbal joints are all at zero degrees, the sensor assembly X,Y,Z
% axis is aligned with the top plate X,Y,Z axis

% State vector:
% error rotation vector - X,Y,Z (rad)
% Velocity - North, East, Down (m/s)
% Delta Angle bias - X,Y,Z (rad)
% Delta Velocity Bias - X,Y,Z (m/s)

% Observations:
% NED velocity - N,E,D (m/s)
% sensor fixed magnetic field vector of base - X,Y,Z


% Time varying parameters:
% XYZ delta angle measurements in sensor axes - rad
% XYZ delta velocity measurements in sensor axes - m/sec
% yaw, roll, pitch gimbal rotation angles

%% define symbolic variables and constants
clear all;

% specify if we want to incorporate accerometer bias estimation into the
% filter, 0 = no, 1 = yes
f_accBiasEst = 0;

syms dax day daz real % IMU delta angle measurements in sensor axes - rad
syms dvx dvy dvz real % IMU delta velocity measurements in sensor axes - m/sec
syms q0 q1 q2 q3 real % quaternions defining attitude of sensor axes relative to local NED
syms vn ve vd real % NED velocity - m/sec
syms dax_b day_b daz_b real % delta angle bias - rad
syms dvx_b dvy_b dvz_b real % delta velocity bias - m/sec
syms dt real % IMU time step - sec
syms gravity real % gravity  - m/sec^2
syms daxNoise dayNoise dazNoise dvxNoise dvyNoise dvzNoise real; % IMU delta angle and delta velocity measurement noise
syms vwn vwe real; % NE wind velocity - m/sec
syms magX magY magZ real; % XYZ top plate magnetic field measurements - milligauss
syms magN magE magD real; % NED earth fixed magnetic field components - milligauss
syms R_VN R_VE R_VD real % variances for NED velocity measurements - (m/sec)^2
syms R_MAG real  % variance for magnetic flux measurements - milligauss^2
syms rotErr1 rotErr2 rotErr3 real; % error rotation vector
syms decl real; % earth magnetic field declination from true north
syms gPsi gPhi gTheta real; % gimbal joint angles yaw, roll, pitch (rad)

%% define the process equations

% define the measured Delta angle and delta velocity vectors
dAngMeas = [dax; day; daz];
dVelMeas = [dvx; dvy; dvz];

% define the delta angle bias errors
dAngBias = [dax_b; day_b; daz_b];

% define the delta velocity bias errors
if (f_accBiasEst > 0)
    dVelBias = [dvx_b; dvy_b; dvz_b];
else
    dVelBias = [0; 0; 0];
end

% define the quaternion rotation vector for the state estimate
estQuat = [q0;q1;q2;q3];

% define the attitude error rotation vector, where error = truth - estimate
errRotVec = [rotErr1;rotErr2;rotErr3];

% define the attitude error quaternion using a first order linearisation
errQuat = [1;0.5*errRotVec];

% Define the truth quaternion as the estimate + error
truthQuat = QuatMult(estQuat, errQuat);

% derive the truth sensor to nav direction cosine matrix
Tsn = Quat2Tbn(truthQuat);

% define the truth delta angle
% ignore coning acompensation as these effects are negligible in terms of 
% covariance growth for our application and grade of sensor
dAngTruth = dAngMeas - dAngBias - [daxNoise;dayNoise;dazNoise];

% Define the truth delta velocity
dVelTruth = dVelMeas - dVelBias - [dvxNoise;dvyNoise;dvzNoise];

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
vNew = [vn;ve;vd] + [0;0;gravity]*dt + Tsn*dVelTruth;

% define the IMU bias error update equations
dabNew = dAngBias;
dvbNew = dVelBias;

% Define the state vector & number of states
if (f_accBiasEst > 0)
    stateVector = [errRotVec;vn;ve;vd;dAngBias;dVelBias];
else
    stateVector = [errRotVec;vn;ve;vd;dAngBias];
end
nStates=numel(stateVector);

save 'symeqns.mat';

%% derive the filter Jacobians

% Define the control (disturbance) vector. Error growth in the inertial
% solution is assumed to be driven by 'noise' in the delta angles and
% velocities, after bias effects have been removed. This is OK because we
% have sensor bias accounted for in the state equations.
distVector = [daxNoise;dayNoise;dazNoise;dvxNoise;dvyNoise;dvzNoise];

% derive the control(disturbance) influence matrix
if (f_accBiasEst > 0)
    predictedState = [errRotNew;vNew;dabNew;dvbNew];
else
    predictedState = [errRotNew;vNew;dabNew];
end
G = jacobian(predictedState, distVector);
G = subs(G, {'rotErr1', 'rotErr2', 'rotErr3'}, {0,0,0});

% derive the state error matrix
distMatrix = diag(distVector);
Q = G*distMatrix*transpose(G);
%matlabFunction(Q,'file','calcQ.m');

% derive the state transition matrix
vNew = subs(vNew,{'daxNoise','dayNoise','dazNoise','dvxNoise','dvyNoise','dvzNoise'}, {0,0,0,0,0,0});
errRotNew = subs(errRotNew,{'daxNoise','dayNoise','dazNoise','dvxNoise','dvyNoise','dvzNoise'}, {0,0,0,0,0,0});
% Define the state vector & number of states
if (f_accBiasEst)
    predictedState = [errRotNew;vNew;dabNew;dvbNew];
else
    predictedState = [errRotNew;vNew;dabNew];
end
F = jacobian(predictedState, stateVector);
F = subs(F, {'rotErr1', 'rotErr2', 'rotErr3'}, {0,0,0});
%matlabFunction(F,'file','calcF.m');

%% Derive the predicted covariance
% This reduces the number of floating point operations by a factor of 4 or
% more compared to using the standard matrix operations in code
% define a symbolic covariance matrix using strings to represent 
% '_l_' to represent '( '
% '_c_' to represent ,
% '_r_' to represent ')' 
% these can be substituted later to create executable code
for rowIndex = 1:nStates
    for colIndex = 1:nStates
        eval(['syms OP_l_',num2str(rowIndex),'_c_',num2str(colIndex), '_r_ real']);
        eval(['P(',num2str(rowIndex),',',num2str(colIndex), ') = OP_l_',num2str(rowIndex),'_c_',num2str(colIndex),'_r_;']);
    end
end
% Derive the predicted covariance matrix using the standard equation
PP = F*P*transpose(F) + Q;
%matlabFunction(PP,'file','calcP.m');
ccode(PP,'file','calcP.c');
FixCode('calcP');

% free up memory
clear all;
reset(symengine);

%% derive equations for fusion of magnetic deviation measurement

load('symeqns.mat');

% Define rotation from magnetometer to yaw gimbal
T3 = [ cos(gPsi)  sin(gPsi)   0; ...
        -sin(gPsi)  cos(gPsi)   0; ...
         0          0           1];
% Define rotation from yaw gimbal to roll gimbal
T1 = [ 1          0           0; ...
         0          cos(gPhi)   sin(gPhi); ...
         0         -sin(gPhi)   cos(gPhi)];
% Define rotation from roll gimbal to pitch gimbal
T2 = [ cos(gTheta)    0      -sin(gTheta); ...
         0              1       0; ...
         sin(gTheta)    0       cos(gTheta)];
% Define rotation from magnetometer to sensor using a 312 rotation sequence
Tms = T2*T1*T3;
% Define rotation from magnetometer to nav axes
Tmn = Tsn*Tms;

save 'symeqns.mat';

% rotate magnetic field measured at top plate into nav axes
magMeasNED = Tmn*[magX;magY;magZ]; 
% the predicted measurement is the angle wrt magnetic north of the horizontal
% component of the measured field
angMeas = tan(magMeasNED(2)/magMeasNED(1)) - decl;
H_MAG = jacobian(angMeas,stateVector); % measurement Jacobian
H_MAG = subs(H_MAG, {'rotErr1', 'rotErr2', 'rotErr3'}, {0,0,0});
H_MAG = H_MAG(1:3);
H_MAG = simplify(H_MAG);
% matlabFunction(H_MAG,'file','calcH_MAG.m');
ccode(H_MAG,'file','calcH_MAG.c');
FixCode('calcH_MAG');

% free up memory
clear all;
reset(symengine);

%% generate helper functions

load 'symeqns.mat';

matlabFunction(Tms,'file','calcTms.m');
Tmn = subs(Tmn, {'rotErr1', 'rotErr2', 'rotErr3'}, {0,0,0});
matlabFunction(Tmn,'file','calcTmn.m');
