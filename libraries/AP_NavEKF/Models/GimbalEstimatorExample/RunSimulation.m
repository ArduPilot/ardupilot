%% Set initial conditions
clear all;
dtSlow = 1/50;
dtFast = 1/1000;
rateMult = round(dtSlow/dtFast);
duration = 60;
indexLimitSlow = round(duration/dtSlow);
indexLimitFast = indexLimitSlow*rateMult;

% create data logging variables
gimbal.time = zeros(1,indexLimitFast);
gimbal.euler = zeros(3,indexLimitFast);
gimbal.eulerTruth = zeros(3,indexLimitFast);
gimbal.eulerError = zeros(3,indexLimitFast);

% Use a random initial truth orientation
phiInit = 0.1*randn;
thetaInit = 0.1*randn;
psiInit = 2*pi*rand - pi;
quatTruth = EulToQuat([phiInit,thetaInit,psiInit]);% [1;0.05*randn;0.05*randn;2*(rand-0.5)];
quatLength = sqrt(quatTruth(1)^2 + quatTruth(2)^2 + quatTruth(3)^2 + quatTruth(4)^2);
quatTruth = quatTruth / quatLength;
TsnTruth = Quat2Tbn(quatTruth);

% define the earths truth magnetic field
declTruth = 10*pi/180;
magEarthTruth = [0.25*cos(declTruth);0.25*sin(declTruth);-0.5];

% define the declination parameter assuming 2deg RMS error - this would be
% obtained from the main EKF to take advantage of in-flight learning
declParam = declTruth + 2*pi/180*randn;

% define the magnetometer bias errors
magMeasBias = 0.02*[randn;randn;randn];

% Define IMU bias errors and noise
gyroBias = 1*pi/180*[randn;randn;randn];
accBias = 0.05*[randn;randn;randn];
gyroNoise = 0.01;
accNoise = 0.05;

% define the state covariances with the exception of the quaternion covariances
Sigma_velNED = 0.5; % 1 sigma uncertainty in horizontal velocity components
Sigma_dAngBias  = 1*pi/180*dtSlow; % 1 Sigma uncertainty in delta angle bias
Sigma_angErr = 1; % 1 Sigma uncertainty in angular misalignment (rad)
covariance   = single(diag([Sigma_angErr*[1;1;1];Sigma_velNED*[1;1;1];Sigma_dAngBias*[1;1;1]]).^2);

% Initialise truth trajectory variables
% fly a CCW circle with constant gimbal angles
gPsiInit = 20*pi/180; % gimbal yaw
gThetaInit = 0; % gimbal pitch
gPhiInit = 0; % gimbal roll
psiTruth = psiInit;
radius = 20;
gndSpd = 5;
trackAngTruth = -pi;
centripAccelMag = gndSpd/radius*gndSpd;
gravAccel = [0;0;-9.81];

%% Main Loop
hdgAlignedEKF=0;
hdgAlignedGimbal=0;
slowIndex = 0;
delAngFast = [0;0;0];
delVelFast = [0;0;0];
delAngSlow = [0;0;0];
delVelSlow = [0;0;0];
prevAngRateMeas = [0;0;0];
prevAccelMeas   = [0;0;0];
quatFast = [1;0;0;0];
quatFastSaved = quatFast;
angRateBiasEKF = [0;0;0];
quatEKF = [1;0;0;0];
for fastIndex = 1:indexLimitFast % 1000 Hz gimbal prediction loop
    time = dtFast*fastIndex;
    % Calculate Truth Data
    % Need to replace this with a full kinematic model or test data
    % calculate truth angular rates - we don't start manoeuvring until
    % heading alignment is complete
    psiRateTruth = gndSpd/radius*hdgAlignedEKF;
    angRateTruth = [0;0;psiRateTruth]; % constant yaw rate
    
    % calculate yaw and track angles
    psiTruth = psiTruth + psiRateTruth*dtFast;
    trackAngTruth = trackAngTruth + psiRateTruth*dtFast;
    
    % Cacluate truth quternion
    quatTruth = EulToQuat([phiInit,thetaInit,psiTruth]);
    
    % Calculate truth rotaton from sensor to NED
    TsnTruth = Quat2Tbn(quatTruth);
    
    % calculate truth accel vector
    centripAccel = centripAccelMag*[-sin(trackAngTruth);cos(trackAngTruth);0];
    accelTruth = transpose(TsnTruth)*(gravAccel + centripAccel);
    
    % calculate truth velocity vector
    truthVel = gndSpd*[cos(trackAngTruth);sin(trackAngTruth);0];
    
    % synthesise sensor measurements
    % Synthesise IMU measurements, adding bias and noise
    angRateMeas = angRateTruth + gyroBias + gyroNoise*[randn;randn;randn];
    accelMeas   = accelTruth + accBias + accNoise*[randn;randn;randn];
    
    % synthesise velocity measurements
    measVel = truthVel;
    
    % synthesise gimbal angles
    gPhi = 0;
    gTheta = 0;
    gPsi = gPsiInit;
    
    % Define rotation from magnetometer to sensor using a 312 rotation sequence
    TmsTruth = calcTms(gPhi,gPsi,gTheta);
    
    % calculate rotation from NED to magnetometer axes Tnm = Tsm * Tns
    TnmTruth = transpose(TmsTruth) * transpose(TsnTruth);
    
    % synthesise magnetometer measurements adding sensor bias
    magMeas = TnmTruth*magEarthTruth + magMeasBias;
    
    % integrate the IMU measurements to produce delta angles and velocities
    % using a trapezoidal integrator
    if isempty(prevAngRateMeas)
        prevAngRateMeas = angRateMeas;
    end
    if isempty(prevAccelMeas)
        prevAccelMeas = accelMeas;
    end
    delAngFast = delAngFast + 0.5*(angRateMeas + prevAngRateMeas)*dtFast;
    delVelFast = delVelFast + 0.5*(accelMeas + prevAccelMeas)*dtFast;
    prevAngRateMeas = angRateMeas;
    prevAccelMeas   = accelMeas;
    
    % Run an attitude prediction calculation at 1000Hz
    % Convert the rotation vector to its equivalent quaternion
    % using a first order approximation after applying the correction for
    % gyro bias using bias estimates from the EKF
    deltaQuat = [1;0.5*(angRateMeas - angRateBiasEKF)*dtFast];
    % Update the quaternions by rotating from the previous attitude through
    % the delta angle rotation quaternion
    quatFast = QuatMult(quatFast,deltaQuat);
    % Normalise the quaternions
    quatFast = NormQuat(quatFast);
    % log the high rate data
    eulLogFast(:,fastIndex) = QuatToEul(quatFast);
    
    % every 20msec we send them to the EKF computer and reset
    % the total
    % we also save a copy of the quaternion from our high rate prediction
    if (rem(fastIndex,rateMult) == 0)
        delAngSlow = delAngFast;
        delVelSlow = delVelFast;
        delAngFast = [0;0;0];
        delVelFast = [0;0;0];
        quatFastSaved = quatFast;
    end
    
    % run the 50Hz EKF loop but do so 5 msec behind the
    % data transmission to simulate the effect of transmission and
    % computational delays
    if (rem(fastIndex,rateMult) == 5)
        slowIndex = slowIndex + 1;
        [quatEKF,angRateBiasEKF,EKFlogs,hdgAlignedEKF] = calcEKF(delAngSlow,delVelSlow,measVel,gPhi,gPsi,gTheta,magMeas,declParam,time,dtSlow,slowIndex,indexLimitSlow);
    end
    
    % Correct Gimbal attitude usng EKF data
    % Assume the gimbal controller receive the EKF solution 10 msec after
    % it sent the sensor data
    if (rem(fastIndex,rateMult) == 10)
        % calculate the quaternion from the EKF corrected attitude to the
        % attitude calculated using the local fast prediction algorithm
        deltaQuatFast = QuatDivide(quatEKF,quatFastSaved);
        % apply this correction to the fast solution at the current time
        % step (this can be applied across several steps to smooth the
        % output if required)
        quatFast = QuatMult(quatFast,deltaQuatFast);
        % normalise the resultant quaternion
        quatFast = NormQuat(quatFast);
        % flag when the gimbals own heading is aligned
        hdgAlignedGimbal = hdgAlignedEKF;
    end
    
    % Log gimbal data
    gimbal.time(fastIndex) = time;
    gimbal.euler(:,fastIndex) = QuatToEul(quatFast);
    gimbal.eulerTruth(:,fastIndex) = QuatToEul(quatTruth);
    if (hdgAlignedGimbal)
        gimbal.eulerError(:,fastIndex) = gimbal.euler(:,fastIndex) - gimbal.eulerTruth(:,fastIndex);
        if (gimbal.eulerError(3,fastIndex) > pi)
            gimbal.eulerError(3,fastIndex) = gimbal.eulerError(3,fastIndex) - 2*pi;
        elseif (gimbal.eulerError(3,fastIndex) < -pi)
            gimbal.eulerError(3,fastIndex) = gimbal.eulerError(3,fastIndex) + 2*pi;
        end
    else
        gimbal.eulerError(:,fastIndex) = [NaN;NaN;NaN];
    end
    
end

%% Generate Plots
close all;
PlotData;