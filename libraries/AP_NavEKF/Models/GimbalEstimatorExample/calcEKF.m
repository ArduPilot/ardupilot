function [quatOut,angRateBiasOut,logOut,headingAlignedOut] = calcEKF(delAngSlow,delVelSlow,measVel,gPhi,gPsi,gTheta,magMeas,declParam,time,dtSlow,frameIndex,maxFrameIndex)

persistent velocityAligned;
if isempty(velocityAligned)
    velocityAligned = 0;
end

persistent quat;
if isempty(quat)
    quat = [1;0;0;0];
end

persistent states;
if isempty(states)
    states = zeros(9,1);
end

persistent covariance;
if isempty(covariance)
    % define the state covariances with the exception of the quaternion covariances
    Sigma_velNED = 0.5; % 1 sigma uncertainty in horizontal velocity components
    Sigma_dAngBias  = 1*pi/180*dtSlow; % 1 Sigma uncertainty in delta angle bias
    Sigma_angErr = 1; % 1 Sigma uncertainty in angular misalignment (rad)
    covariance   = single(diag([Sigma_angErr*[1;1;1];Sigma_velNED*[1;1;1];Sigma_dAngBias*[1;1;1]]).^2);
end

persistent headingAligned
if isempty(headingAligned)
    headingAligned = 0;
end

persistent log;
if isempty(log)
    % create data logging variables
    log.time = zeros(1,maxFrameIndex);
    log.states = zeros(9,maxFrameIndex);
    log.quat   = zeros(4,maxFrameIndex);
    log.euler = zeros(3,maxFrameIndex);
    log.tiltCorr = zeros(1,maxFrameIndex);
    log.velInnov = zeros(3,maxFrameIndex);
    log.decInnov = zeros(1,maxFrameIndex);
    log.velInnovVar = log.velInnov;
    log.decInnovVar = log.decInnov;
end

% predict states
[quat, states, Tsn, delAngCorrected, delVelCorrected]  = PredictStates(quat,states,delAngSlow,delVelSlow,dtSlow);

% log state prediction data
log.states(:,frameIndex) = states;
log.quat(:,frameIndex) = quat;
log.euler(:,frameIndex) = QuatToEul(quat);

% predict covariance matrix
covariance = PredictCovarianceOptimised(delAngCorrected,delVelCorrected,quat,states,covariance,dtSlow);

if (velocityAligned == 0)
    % initialise velocity states
    states(4:6) = measVel;
    velocityAligned = 1;
else
    % fuse velocity measurements
    [quat,states,tiltCorrection,covariance,velInnov,velInnovVar] = FuseVelocity(quat,states,covariance,measVel);
    % log velocity fusion data
    log.velInnov(:,frameIndex) = velInnov;
    log.velInnovVar(:,frameIndex) = velInnovVar;
    log.tiltCorr(1,frameIndex) = tiltCorrection;
end


% Align the heading once there has been enough time for the filter to
% settle and the tilt corrections have dropped below a threshold
if (((time > 5.0 && tiltCorrection < 1e-4) || (time > 30.0)) && headingAligned==0)
    % calculate the initial heading using magnetometer, gimbal,
    % estimated tilt and declination
    quat = AlignHeading(gPhi,gPsi,gTheta,Tsn,magMeas,quat,declParam);
    headingAligned = 1;
end

% fuse magnetometer measurements and log fusion data
if (headingAligned == 1)
    [quat,states,covariance,decInnov,decInnovVar] = FuseMagnetometer(quat,states,covariance,magMeas,declParam,gPhi,gPsi,gTheta);
    log.decInnov(:,frameIndex) = decInnov;
    log.decInnovVar(:,frameIndex) = decInnovVar;
end

% time stamp the log data
log.time(frameIndex) = time;

% write to the output data
quatOut = quat;
angRateBiasOut = states(7:9)/dtSlow;
logOut = log;
headingAlignedOut = headingAligned;

end