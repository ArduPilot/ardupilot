function [...
    states, ... % state vector after fusion of measurements
    P, ... % state covariance matrix after fusion of corrections
    innovation,... % NED velocity innovations (m/s)
    varInnov] ... % NED velocity innovation variance ((m/s)^2)
    = FuseVelocity( ...
    states, ... % predicted states from the INS
    P, ... % predicted covariance
    measVel) % NED velocity measurements (m/s)

R_OBS = 0.5^2;
innovation = zeros(1,3);
varInnov = zeros(1,3);
% Fuse measurements sequentially
for obsIndex = 1:3
    stateIndex = 4 + obsIndex;
    % Calculate the velocity measurement innovation
    innovation(obsIndex) = states(stateIndex) - measVel(obsIndex);
    
    % Calculate the Kalman Gain
    H = zeros(1,10);
    H(1,stateIndex) = 1;
    varInnov(obsIndex) = (H*P*transpose(H) + R_OBS);
    K = (P*transpose(H))/varInnov(obsIndex);
    
    % Calculate state corrections
    xk = K * innovation(obsIndex);
    
    % Apply the state corrections
    states = states - xk;
    
    % re-normalise the quaternion
    quatMag = sqrt(states(1)^2 + states(2)^2 + states(3)^2 + states(4)^2);
    states(1:4) = states(1:4) / quatMag;
    
    % Update the covariance
    P = P - K*H*P;
    
    % Force symmetry on the covariance matrix to prevent ill-conditioning
    P = 0.5*(P + transpose(P));
    
    % ensure diagonals are positive
    for i=1:10
        if P(i,i) < 0
            P(i,i) = 0;
        end
    end
    
end

end