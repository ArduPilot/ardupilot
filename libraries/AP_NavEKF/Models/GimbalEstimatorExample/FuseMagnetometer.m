function [...
    nextQuat, ... % quaternion state vector after fusion of measurements
    nextStates, ... % state vector after fusion of measurements
    nextP, ... % state covariance matrix after fusion of corrections
    innovation, ... % Declination innovation - rad
    varInnov] ... %
    = FuseMagnetometer( ...
    quat, ... % predicted quaternion states
    states, ... % predicted states
    P, ... % predicted covariance
    magData, ... % body frame magnetic flux measurements
    decl, ... % magnetic field declination from true north
    gPhi, gPsi, gTheta) % gimbal yaw, roll, pitch angles

q0 = quat(1);
q1 = quat(2);
q2 = quat(3);
q3 = quat(4);

magX = magData(1);
magY = magData(2);
magZ = magData(3);

R_MAG = 0.1745^2;

% Calculate observation Jacobian
H = calcH_MAG(gPhi,gPsi,gTheta,magX,magY,magZ,q0,q1,q2,q3);
% Calculate innovation variance and Kalman gains
% Take advantage of the fact that only the first 3 elements in H are non zero
varInnov = (H(1,1:3)*P(1:3,1:3)*transpose(H(1,1:3)) + R_MAG);
Kfusion = (P(:,1:3)*transpose(H(1,1:3)))/varInnov;

% Calculate the innovation
innovation = calcMagAng(decl,gPhi,gPsi,gTheta,magX,magY,magZ,q0,q1,q2,q3);

if (innovation > pi)
    innovation = innovation - 2*pi;
elseif (innovation < -pi)
    innovation = innovation + 2*pi;
end
if (innovation > 0.5)
    innovation = 0.5;
elseif (innovation < -0.5)
    innovation = -0.5;
end

% correct the state vector
states(1:3) = 0;
states = states - Kfusion * innovation;

% the first 3 states represent the angular misalignment vector. This is
% is used to correct the estimate quaternion
% Convert the error rotation vector to its equivalent quaternion
% error = truth - estimate
rotationMag = sqrt(states(1)^2 + states(2)^2 + states(3)^2);
if rotationMag<1e-6
    deltaQuat = single([1;0;0;0]);
else
    deltaQuat = [cos(0.5*rotationMag); [states(1);states(2);states(3)]/rotationMag*sin(0.5*rotationMag)];
end

% Update the quaternion states by rotating from the previous attitude through
% the delta angle rotation quaternion
nextQuat = [quat(1)*deltaQuat(1)-transpose(quat(2:4))*deltaQuat(2:4); quat(1)*deltaQuat(2:4) + deltaQuat(1)*quat(2:4) + cross(quat(2:4),deltaQuat(2:4))];

% normalise the updated quaternion states
quatMag = sqrt(nextQuat(1)^2 + nextQuat(2)^2 + nextQuat(3)^2 + nextQuat(4)^2);
if (quatMag > 1e-6)
    nextQuat = nextQuat / quatMag;
end

% correct the covariance P = P - K*H*P
% Take advantage of the fact that only the first 3 elements in H are non zero
P = P - Kfusion*H(1,1:3)*P(1:3,:);

% Force symmetry on the covariance matrix to prevent ill-conditioning
% of the matrix which would cause the filter to blow-up
P = 0.5*(P + transpose(P));

% ensure diagonals are positive
for i=1:9
    if P(i,i) < 0
        P(i,i) = 0;
    end
end

% Set default output for states and covariance
nextP = P;
nextStates = states;

end