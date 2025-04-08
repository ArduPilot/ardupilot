%% Set initial conditions
clear all;
load('fltTest.mat');
startDelayTime = 100; % number of seconds to delay filter start (used to simulate in-flight restart)
dt = 1/50;
indexLimit = length(IMU);
magIndexlimit = length(MAG);
statesLog = zeros(10,indexLimit);
eulLog   = zeros(4,indexLimit);
velInnovLog = zeros(4,indexLimit);
angErrLog = zeros(2,indexLimit);
decInnovLog = zeros(2,magIndexlimit);
velInnovVarLog = velInnovLog;
decInnovVarLog = decInnovLog;
% initialise the state vector and quaternion
states = zeros(9,1);
quat = [1;0;0;0];
Tbn = Quat2Tbn(quat);
% average last 10 accel readings to reduce effect of noise
initAccel(1) = mean(IMU(1:10,6));
initAccel(2) = mean(IMU(1:10,7));
initAccel(3) = mean(IMU(1:10,8));
% Use averaged accel readings to align tilt
quat = AlignTilt(quat,initAccel);
% Set the expected declination
measDec = 0.18;
% define the state covariances
Sigma_velNED = 0.5; % 1 sigma uncertainty in horizontal velocity components
Sigma_dAngBias  = 5*pi/180*dt; % 1 Sigma uncertainty in delta angle bias
Sigma_angErr = 1; % 1 Sigma uncertainty in angular misalignment (rad)
covariance   = single(diag([Sigma_angErr*[1;1;1];Sigma_velNED*[1;1;1];Sigma_dAngBias*[1;1;1]].^2));
%% Main Loop
magIndex = 1;
time = 0;
angErr = 0;
headingAligned = 0;
% delay start by a minimum of 10 IMU samples to allow for initial tilt
% alignment delay
startIndex = max(11,ceil(startDelayTime/dt));
for index = startIndex:indexLimit
    time=time+dt + startIndex*dt;
    % read IMU measurements
    angRate = IMU(index,3:5)';
    % switch in a bias offset to test the filter
    if (time > +inf)
        angRate = angRate + [1;-1;1]*pi/180;
    end
    accel = IMU(index,6:8)';
    % predict states
    [quat, states, Tbn, delAng, delVel]  = PredictStates(quat,states,angRate,accel,dt);
    statesLog(1,index) = time;
    statesLog(2:10,index) = states;
    eulLog(1,index) = time;
    eulLog(2:4,index) = QuatToEul(quat);
    % predict covariance matrix
    covariance  = PredictCovariance(delAng,delVel,quat,states,covariance,dt);
    % fuse velocity measurements - use synthetic measurements
    measVel = [0;0;0];
    [quat,states,angErr,covariance,velInnov,velInnovVar] = FuseVelocity(quat,states,covariance,measVel);
    velInnovLog(1,index) = time;
    velInnovLog(2:4,index) = velInnov;
    velInnovVarLog(1,index) = time;
    velInnovVarLog(2:4,index) = velInnovVar;
    angErrLog(1,index) = time;
    angErrLog(2,index) = angErr;
    % read magnetometer measurements
    while ((MAG(magIndex,1) < IMU(index,1)) && (magIndex < magIndexlimit))
        magIndex = magIndex + 1;
        magBody = 0.001*MAG(magIndex,3:5)';
        if (time >= 1.0 && headingAligned==0 && angErr < 1e-3)
            quat = AlignHeading(quat,magBody,measDec);
            headingAligned = 1;
        end
        % fuse magnetometer measurements if new data available and when tilt has settled
        if (headingAligned == 1)
            [quat,states,covariance,decInnov,decInnovVar] = FuseMagnetometer(quat,states,covariance,magBody,measDec,Tbn);
            decInnovLog(1,magIndex) = time;
            decInnovLog(2,magIndex) = decInnov;
            decInnovVarLog(1,magIndex) = time;
            decInnovVarLog(2,magIndex) = decInnovVar;
        end
    end
end

%% Generate plots
PlotData;