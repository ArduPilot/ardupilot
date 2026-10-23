function [...
    states, ... % state vector after fusion of measurements
    P, ... % state covariance matrix after fusion of corrections
    innovation, ... % Declination innovation - rad
    varInnov] ... %
    = FuseMagnetometer( ...
    states, ... % predicted states
    P, ... % predicted covariance
    magData, ... % body frame magnetic flux measurements
    measDec, ... % magnetic field declination - azimuth angle measured from true north (rad)
    Tbn)  % Estimated coordinate transformation matrix from body to NED frame

q0 = states(1);
q1 = states(2);
q2 = states(3);
q3 = states(4);

magX = magData(1);
magY = magData(2);
magZ = magData(3);

R_MAG = 0.05^2;

H = calcH_MAG(magX,magY,magZ,q0,q1,q2,q3);
varInnov = (H*P*transpose(H) + R_MAG);
Kfusion = (P*transpose(H))/varInnov;

% Calculate the predicted magnetic declination
magMeasNED = Tbn*[magX;magY;magZ];
predDec = atan2(magMeasNED(2),magMeasNED(1));

% Calculate the measurement innovation
innovation = predDec - measDec;

% correct the state vector
states = states - Kfusion * innovation;

% re-normalise the quaternion
quatMag = sqrt(states(1)^2 + states(2)^2 + states(3)^2 + states(4)^2);
states(1:4) = states(1:4) / quatMag;

% correct the covariance P = P - K*H*P
P = P - Kfusion*H*P;

% Force symmetry on the covariance matrix to prevent ill-conditioning
% of the matrix which would cause the filter to blow-up
P = 0.5*(P + transpose(P));

% ensure diagonals are positive
for i=1:10
    if P(i,i) < 0
        P(i,i) = 0;
    end
end

end