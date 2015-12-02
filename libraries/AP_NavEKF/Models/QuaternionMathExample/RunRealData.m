%% Set initial conditions
clear all;
load('fltTest.mat');
startDelayTime = 100; % number of seconds to delay filter start (used to simulate in-flight restart)
dt = 1/50;
startTime = 0.001*(IMU(1,2));
stopTime = 0.001*(IMU(length(IMU),2));
indexLimit = length(IMU);
magIndexlimit = length(MAG);
statesLog = zeros(11,indexLimit);
eulLog   = zeros(4,indexLimit);
velInnovLog = zeros(4,indexLimit);
angErrLog = velInnovLog;
decInnovLog = zeros(2,magIndexlimit);
velInnovVarLog = velInnovLog;
decInnovVarLog = decInnovLog;
% initialise the filter to level
quat = [1;0;0;0];
states = zeros(10,1);
Tbn = Quat2Tbn(quat);
% Set the expected declination
measDec = 0.18;
% define the state covariances with the exception of the quaternion covariances
Sigma_velNED = 0.5; % 1 sigma uncertainty in horizontal velocity components
Sigma_dAngBias  = 5*pi/180*dt; % 1 Sigma uncertainty in delta angle bias
Sigma_quatErr = 1; % 1 Sigma uncertainty in angular misalignment (rad)
covariance   = single(diag([Sigma_quatErr*[1;1;1;1];Sigma_velNED*[1;1;1];Sigma_dAngBias*[1;1;1]].^2));
%% Main Loop
magIndex = 1;
time = 0;
tiltError = 0;
headingAligned = 0;
angErrVec = [0;0;0];
startIndex = max(11,ceil(startDelayTime/dt));
for index = startIndex:indexLimit
    time=time+dt + startIndex*dt;
    % read IMU measurements and correct rates using estimated bias
    angRate = IMU(index,3:5)' - states(7:9)./dt;
    accel = IMU(index,6:8)';
    % predict states
    [quat, states, Tbn, delAng, delVel]  = PredictStates(quat,states,angRate,accel,dt);
    statesLog(1,index) = time;
    statesLog(2:11,index) = states;
    eulLog(1,index) = time;
    eulLog(2:4,index) = QuatToEul(quat);
    % predict covariance matrix
    covariance  = PredictCovariance(delAng,delVel,quat,states,covariance,dt);
    % read magnetometer measurements
    while ((MAG(magIndex,1) < IMU(index,1)) && (magIndex < magIndexlimit))
        magIndex = magIndex + 1;
        % fuse magnetometer measurements if new data available and when tilt has settled
        if ((MAG(magIndex,1) >= IMU(index,1)) && ((angErrVec(1)^2 + angErrVec(2)^2) < 0.05^2) && (index > 50))
            magBody = 0.001*MAG(magIndex,3:5)';
            [states,covariance,decInnov,decInnovVar] = FuseMagnetometer(states,covariance,magBody,measDec,Tbn);
            decInnovLog(1,magIndex) = time;
            decInnovLog(2,magIndex) = decInnov;
            decInnovVarLog(1,magIndex) = time;
            decInnovVarLog(2,magIndex) = decInnovVar;
        end
    end
    % fuse velocity measurements - use synthetic measurements
    measVel = [0;0;0];
    [states,covariance,velInnov,velInnovVar] = FuseVelocity(states,covariance,measVel);
    velInnovLog(1,index) = time;
    velInnovLog(2:4,index) = velInnov;
    velInnovVarLog(1,index) = time;
    velInnovVarLog(2:4,index) = velInnovVar;
end

%% Generate Plots
PlotData;