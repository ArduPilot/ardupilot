function [...
    quat, ... % quaternion state vector after fusion of measurements
    states, ... % state vector after fusion of measurements
    angErr, ... % angle error
    P, ... % state covariance matrix after fusion of corrections
    innovation,... % NED velocity innovations (m/s)
    varInnov] ... % NED velocity innovation variance ((m/s)^2)
    = FuseVelocity( ...
    quat, ... % predicted quaternion states from the INS
    states, ... % predicted states from the INS
    P, ... % predicted covariance
    measVel) % NED velocity measurements (m/s)

R_OBS = 0.5^2;
innovation = zeros(1,3);
varInnov = zeros(1,3);
% Fuse measurements sequentially
angErrVec = [0;0;0];
for obsIndex = 1:3
    stateIndex = 3 + obsIndex;
    % Calculate the velocity measurement innovation
    innovation(obsIndex) = states(stateIndex) - measVel(obsIndex);
    
    % Calculate the Kalman Gain
    H = zeros(1,9);
    H(1,stateIndex) = 1;
    varInnov(obsIndex) = (H*P*transpose(H) + R_OBS);
    K = (P*transpose(H))/varInnov(obsIndex);
    
    % Calculate state corrections
    xk = K * innovation(obsIndex);
    
    % Apply the state corrections
    states(1:3) = 0;
    states = states - xk;
    
    % Store tilt error estimate for external monitoring
    angErrVec = angErrVec + states(1:3);
    
    % the first 3 states represent the angular misalignment vector.
    % This is used to correct the estimated quaternion
    % Convert the error rotation vector to its equivalent quaternion
    % truth = estimate + error
    rotationMag = sqrt(states(1)^2 + states(2)^2 + states(3)^2);
    if rotationMag > 1e-12
        deltaQuat = [cos(0.5*rotationMag); [states(1);states(2);states(3)]/rotationMag*sin(0.5*rotationMag)];
        % Update the quaternion states by rotating from the previous attitude through
        % the error quaternion
        quat = QuatMult(quat,deltaQuat);
        % re-normalise the quaternion
        quatMag = sqrt(quat(1)^2 + quat(2)^2 + quat(3)^2 + quat(4)^2);
        quat = quat / quatMag;
    end
    
    % Update the covariance
    P = P - K*H*P;
    
    % Force symmetry on the covariance matrix to prevent ill-conditioning
    P = 0.5*(P + transpose(P));
    
    % ensure diagonals are positive
    for i=1:9
        if P(i,i) < 0
            P(i,i) = 0;
        end
    end
    
end

angErr = sqrt(dot(angErrVec,angErrVec));

end