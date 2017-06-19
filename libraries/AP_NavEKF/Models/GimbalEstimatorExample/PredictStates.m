function [quat, states, Tbn, correctedDelAng, correctedDelVel]  = PredictStates( ...
    quat, ... % previous quaternion states 4x1
    states, ... % previous states (3x1 rotation error, 3x1 velocity, 3x1 gyro bias)
    dAng, ... % IMU delta angles, 3x1 (rad)
    dVel, ... % IMU delta velocities 3x1 (m/s)
    dt) % time since last IMU measurement (sec)

% Define parameters used for previous angular rates and acceleration shwich
% are used for trapezoidal integration
% define persistent variables for previous delta angle and velocity which
% are required for sculling and coning error corrections
persistent prevDelAng;
if isempty(prevDelAng)
    prevDelAng = dAng;
end
persistent prevDelVel;
if isempty(prevDelVel)
    prevDelVel = dVel;
end

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

% Convert the rotation vector to its equivalent quaternion
deltaQuat = RotToQuat(correctedDelAng);

% Update the quaternions by rotating from the previous attitude through
% the delta angle rotation quaternion
quat = QuatMult(quat,deltaQuat);

% Normalise the quaternions
quat = NormQuat(quat);

% Calculate the body to nav cosine matrix
Tbn = Quat2Tbn(quat);
  
% transform body delta velocities to delta velocities in the nav frame
delVelNav = Tbn * correctedDelVel + [0;0;9.807]*dt;

% Sum delta velocities to get velocity
states(4:6) = states(4:6) + delVelNav(1:3);

end