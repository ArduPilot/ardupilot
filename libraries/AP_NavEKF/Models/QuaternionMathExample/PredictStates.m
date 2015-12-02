function [nextQuat, nextStates, Tbn, correctedDelAng, correctedDelVel]  = PredictStates( ...
    quat, ... % previous quaternion states 4x1
    states, ... % previous states (3x1 rotation error, 3x1 velocity, 3x1 gyro bias)
    angRate, ... % IMU rate gyro measurements, 3x1 (rad/sec)
    accel, ... % IMU accelerometer measurements 3x1 (m/s/s)
    dt) % time since last IMU measurement (sec)

% Define parameters used for previous angular rates and acceleration shwich
% are used for trapezoidal integration
persistent prevAngRate;
if isempty(prevAngRate)
    prevAngRate = angRate;
end
persistent prevAccel;
if isempty(prevAccel)
    prevAccel = accel;
end
% define persistent variables for previous delta angle and velocity which
% are required for sculling and coning error corrections
persistent prevDelAng;
if isempty(prevDelAng)
    prevDelAng = single([0;0;0]);
end
persistent prevDelVel;
if isempty(prevDelVel)
    prevDelVel = single([0;0;0]);
end

% Convert IMU data to delta angles and velocities using trapezoidal
% integration
dAng = 0.5*(angRate + prevAngRate)*dt;
dVel = 0.5*(accel   + prevAccel  )*dt;
prevAngRate = angRate;
prevAccel   = accel;

% Remove sensor bias errors
dAng = dAng - states(7:9);

% Apply rotational and skulling corrections
correctedDelVel= dVel + ...
    0.5*cross(prevDelAng + dAng , prevDelVel + dVel) + 1/6*cross(prevDelAng + dAng , cross(prevDelAng + dAng , prevDelVel + dVel)) + ... % rotational correction
    1/12*(cross(prevDelAng , dVel) + cross(prevDelVel , dAng)); % sculling correction

% Apply corrections for coning errors
correctedDelAng   = dAng - 1/12*cross(prevDelAng , dAng);

% Save current measurements
prevDelAng = dAng;
prevDelVel = dVel;

% Initialise the updated state vector
nextQuat = quat;
nextStates   = states;

% Convert the rotation vector to its equivalent quaternion
rotationMag = sqrt(correctedDelAng(1)^2 + correctedDelAng(2)^2 + correctedDelAng(3)^2);
if rotationMag<1e-12
  deltaQuat = single([1;0;0;0]);
else
  deltaQuat = [cos(0.5*rotationMag); correctedDelAng/rotationMag*sin(0.5*rotationMag)];
end

% Update the quaternions by rotating from the previous attitude through
% the delta angle rotation quaternion
qUpdated = [quat(1)*deltaQuat(1)-transpose(quat(2:4))*deltaQuat(2:4); quat(1)*deltaQuat(2:4) + deltaQuat(1)*quat(2:4) + cross(quat(2:4),deltaQuat(2:4))];

% Normalise the quaternions and update the quaternion states
quatMag = sqrt(qUpdated(1)^2 + qUpdated(2)^2 + qUpdated(3)^2 + qUpdated(4)^2);
if (quatMag < 1e-16)
    nextQuat(1:4) = qUpdated;
else
    nextQuat(1:4) = qUpdated / quatMag;
end

% Calculate the body to nav cosine matrix
Tbn = Quat2Tbn(nextQuat);
  
% transform body delta velocities to delta velocities in the nav frame
delVelNav = Tbn * correctedDelVel + [0;0;9.807]*dt;

% Sum delta velocities to get velocity
nextStates(4:6) = states(4:6) + delVelNav(1:3);

end