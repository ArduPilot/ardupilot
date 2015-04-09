%% Set initial conditions
clear all;
dt = 1/100;
duration = 10;
indexLimit = round(duration/dt);
statesLog = zeros(10,indexLimit);
eulLog   = zeros(4,indexLimit);
velInnovLog = zeros(4,indexLimit);
decInnovLog = zeros(2,indexLimit);
velInnovVarLog = velInnovLog;
decInnovVarLog = decInnovLog;
angErrLog = zeros(2,indexLimit);
% Use a random initial orientation
quatTruth = [rand;randn;randn;randn];
quatLength = sqrt(quatTruth(1)^2 + quatTruth(2)^2 + quatTruth(3)^2 + quatTruth(4)^2);
quatTruth = quatTruth / quatLength;
TbnTruth = Quat2Tbn(quatTruth);
% initialise the filter to level
quat = [1;0;0;0];
states = zeros(9,1);
Tbn = Quat2Tbn(quat);
% define the earths truth magnetic field
magEarthTruth = [0.3;0.1;-0.5];
% define the assumed declination using th etruth field plus location
% variation
measDec = atan2(magEarthTruth(2),magEarthTruth(1)) + 2*pi/180*randn;
% define the magnetometer bias errors
magMeasBias = 0.02*[randn;randn;randn];
% define the state covariances with the exception of the quaternion covariances
Sigma_velNED = 0.5; % 1 sigma uncertainty in horizontal velocity components
Sigma_dAngBias  = 1*pi/180*dt; % 1 Sigma uncertainty in delta angle bias
Sigma_angErr = 1; % 1 Sigma uncertainty in angular misalignment (rad)
covariance   = single(diag([Sigma_angErr*[1;1;1];Sigma_velNED*[1;1;1];Sigma_dAngBias*[1;1;1]].^2));
%% Main Loop
headingAligned=0;
time = 0;
for index = 1:indexLimit
    time=time+dt;
    % synthesise IMU measurements
    angRate = 0*[randn;randn;randn];
    accel = 0*[randn;randn;randn] + transpose(TbnTruth)*[0;0;-9.81];
    % predict states
    [quat, states, Tbn, delAng, delVel]  = PredictStates(quat,states,angRate,accel,dt);
    statesLog(1,index) = time;
    statesLog(2:10,index) = states;
    eulLog(1,index) = time;
    eulLog(2:4,index) = QuatToEul(quat);
    % predict covariance matrix
    covariance  = PredictCovariance(delAng,delVel,quat,states,covariance,dt);
    % synthesise velocity measurements
    measVel = [0;0;0];
    % fuse velocity measurements
    [quat,states,angErr,covariance,velInnov,velInnovVar] = FuseVelocity(quat,states,covariance,measVel);
    velInnovLog(1,index) = time;
    velInnovLog(2:4,index) = velInnov;
    velInnovVarLog(1,index) = time;
    velInnovVarLog(2:4,index) = velInnovVar;
    angErrLog(1,index) = time;
    angErrLog(2,index) = angErr;
    % synthesise magnetometer measurements adding sensor bias
    magBody = transpose(TbnTruth)*magEarthTruth + magMeasBias;
    % fuse magnetometer measurements
    if (index > 500 && headingAligned==0 && angErr < 1e-4)
        quat = AlignHeading(quat,magBody,measDec);
        headingAligned = 1;
    end
    if (headingAligned == 1)
        [quat,states,covariance,decInnov,decInnovVar] = FuseMagnetometer(quat,states,covariance,magBody,measDec,Tbn);
            decInnovLog(1,index) = time;
            decInnovLog(2,index) = decInnov;
            decInnovVarLog(1,index) = time;
            decInnovVarLog(2,index) = decInnovVar;
     end
    
end

%% Generate Plots
PlotData;