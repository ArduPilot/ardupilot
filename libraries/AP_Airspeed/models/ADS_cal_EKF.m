% Implementation of a simple 3-state EKF that can identify the scale
% factor that needs to be applied to a true airspeed measurement
% Paul Riseborough 27 June 2013

% Inputs:
% Measured true airsped (m/s)

clear all;

% Define wind speed used for truth model
vwn_truth = 4.0;
vwe_truth = 3.0;
vwd_truth = -0.5; % convection can produce values of up to 1.5 m/s, however 
                  % average will zero over longer periods at lower altitudes
                  % Slope lift will be persistent

% Define airspeed scale factor used for truth model
K_truth = 1.2;

% Use a 1 second time step
DT = 1.0;

% Define the initial state error covariance matrix
% Assume initial wind uncertainty of 10 m/s and scale factor uncertainty of
% 0.2
P = diag([10^2 10^2 0.001^2]);

% Define state error growth matrix assuming wind changes at a rate of 0.1
% m/s/s and scale factor drifts at a rate of 0.001 per second
Q = diag([0.1^2 0.1^2 0.001^2])*DT^2;

% Define the initial state matrix assuming zero wind and a scale factor of
% 1.0
x = [0;0;1.0];

for i = 1:1000
    
    %% Calculate truth values
    % calculate ground velocity by simulating a wind relative
    % circular path of 60m radius and 16 m/s airspeed
    time = i*DT;
    radius = 60;
    TAS_truth = 16;
    vwnrel_truth = TAS_truth*cos(TAS_truth*time/radius);
    vwerel_truth = TAS_truth*sin(TAS_truth*time/radius);
    vwdrel_truth = 0.0;
    vgn_truth = vwnrel_truth + vwn_truth;
    vge_truth = vwerel_truth + vwe_truth;
    vgd_truth = vwdrel_truth + vwd_truth;
    
    % calculate measured ground velocity and airspeed, adding some noise and
    % adding a scale factor to the airspeed measurement.
    vgn_mea = vgn_truth + 0.1*rand;
    vge_mea = vge_truth + 0.1*rand;
    vgd_mea = vgd_truth + 0.1*rand;
    TAS_mea = K_truth * TAS_truth + 0.5*rand;
    
    %% Perform filter processing
    % This benefits from a matrix library that can handle up to 3x3
    % matrices
    
    % Perform the covariance prediction
    % Q is a diagonal matrix so only need to add three terms in
    % C code implementation
    P = P + Q;
    
    % Perform the predicted measurement using the current state estimates
    % No state prediction required because states are assumed to be time
    % invariant plus process noise
    % Ignore vertical wind component
    TAS_pred = x(3) * sqrt((vgn_mea - x(1))^2 + (vge_mea - x(2))^2 + vgd_mea^2);
    
    % Calculate the observation Jacobian H_TAS
    SH1 = (vge_mea - x(2))^2 + (vgn_mea - x(1))^2;
    SH2 = 1/sqrt(SH1);
    H_TAS = zeros(1,3);
    H_TAS(1,1) = -(x(3)*SH2*(2*vgn_mea - 2*x(1)))/2;
    H_TAS(1,2) = -(x(3)*SH2*(2*vge_mea - 2*x(2)))/2;
    H_TAS(1,3) = 1/SH2;
    
    % Calculate the fusion innovaton covariance assuming a TAS measurement
    % noise of 1.0 m/s
    S = H_TAS*P*H_TAS' + 1.0; % [1 x 3] * [3 x 3] * [3 x 1] + [1 x 1]
    
    % Calculate the Kalman gain
    KG = P*H_TAS'/S; % [3 x 3] * [3 x 1] / [1 x 1]
    
    % Update the states
    x = x + KG*(TAS_mea - TAS_pred); % [3 x 1] + [3 x 1] * [1 x 1]
    
    % Update the covariance matrix
    P = P - KG*H_TAS*P; % [3 x 3] *
    
    % force symmetry on the covariance matrix - necessary due to rounding
    % errors
    % Implementation will also need a further check to prevent diagonal 
    % terms becoming negative due to rounding errors
    % This step can be made more efficient by excluding diagonal terms 
    % (would reduce processing by 1/3)
    P = 0.5*(P + P'); % [1 x 1] * ( [3 x 3] + [3 x 3])
    
    %% Store results
    output(i,:) = [time,x(1),x(2),x(3),vwn_truth,vwe_truth,K_truth];
   
end

%% Plot output
figure;
subplot(3,1,1);plot(output(:,1),[output(:,2),output(:,5)]);
ylabel('Wind Vel North (m/s)');
xlabel('time (sec)');
grid on;
subplot(3,1,2);plot(output(:,1),[output(:,3),output(:,6)]);
ylabel('Wind Vel East (m/s)');
xlabel('time (sec)');
grid on;
subplot(3,1,3);plot(output(:,1),[output(:,4),output(:,7)]);
ylim([0 1.5]);
ylabel('Airspeed scale factor correction');
xlabel('time (sec)');
grid on;

