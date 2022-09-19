%% Define log index and start + stop time
log_idx = 1;
tStart = 0;
tEnd = inf;

%% Setting simMode
% 0 = Model validation - Identified models are used and full control loop
% is active
% 1 = Rate Controller validation - No models used, measured input signals
% of rate controller are used
% 2 = Attitude Controller validation - No models used, measured input
% signals of attitude controller are used
% 3 = Altitude Controller validation - No models used, measured input
% signals of attitude controller are used
% 4 = Optimization of controller - Not implemented yet
simMode = 3;

%% Check for existance of data

% Check if sid data is existing
if exist('sidLogs', 'var') % in Workspace
    disp('.mat file already loaded. Reading parameter and signals...');
elseif exist('sid.mat', 'file') % in a .mat file with all other axis tests
    load('sid.mat');
    disp('Loaded data from sid.mat');
elseif ~exist(loadedDataConfig)
    error('Could not find configuration struct. Aborting script.')
else
    error('Could not find data. Aborting script.')
end

%% Config and variable declaration

% Sample rate of simulation
Fs = 400;
dt = 0.0025;
sim.param.dt = 0.0025;

% Load data
sid = sidLogs(log_idx).data;
% Check if relevant messages are defined
if (numel(sid) == 0)
    error('Ardupilog object sid is empty. Aborting.');
end

% Get mode of current test run
mode = loadedDataConfig.modes(log_idx);
% Get manual throttle flag for current test run
thr_man = loadedDataConfig.modesConfig(log_idx).thr_man;
% Get filter messages of current test run
msgs = loadedDataConfig.modesConfig(log_idx).filter_msgs;
% Set axis
axis = loadedDataConfig.axisList(log_idx);
% Check if config parameters are complete
if (isempty(mode) || isempty(thr_man) || isempty(msgs))
   error('Config variables (mode, thr_man, msgs) not complete. Aborting.'); 
end

% Define validation and optimization flag of simulation
sim.param.mdlVal = simMode == 0;
sim.param.rateCtrlVal = simMode == 1;
sim.param.attCtrlVal = simMode == 2 || simMode == 3;
sim.param.altCtrlVal = simMode == 3 || mode == 2;
sim.param.optCtrl = simMode == 4;

%% Optimation phase parameters
% Test signal step:
OPT.stepMag = 25; % Deg/s
OPT.stepTime = 1.4; % s

% Simulation time for optimization
OPT.tEndOpt = 2.3375;

% Reference signal
OPT.tRef = (0:OPT.tEndOpt/dt)*dt;
OPT.yRef = 1-exp(14*(-OPT.tRef/2));
% plot(tRef, yRef);
% xlim([0 tEndOpt]);

%% General Copter settings
sim.param.LOOP_RATE = getParamVal(sid, 'SCHED_LOOP_RATE');
sim.param.dt = round(double(1 / sim.param.LOOP_RATE), 4); % Sample time resulting from loop rate. Round to four decimal digits
sim.param.FRAME_CLASS = getParamVal(sid, 'FRAME_CLASS');
sim.param.FRAME_TYPE = getParamVal(sid, 'FRAME_TYPE');
[sim.param.NUM_MOTORS, sim.param.AXIS_FAC_MOTORS] = getMotorMixerFactors(sim.param.FRAME_CLASS, sim.param.FRAME_TYPE);
sim.param.EKF_TYPE = getParamVal(sid, 'AHRS_EKF_TYPE');
sim.param.GRAVITY_MSS = single(9.80665);
sim.param.dt_THR_LOOP = 1 / 50; % Sample time of throttle loop in Copter.cpp. Defined in Copter.cpp, line 94
sim.param.ANGLE_MAX = getParamVal(sid, 'ANGLE_MAX'); % Maximum lean angle in all flight modes
sim.param.ACRO_Y_EXPO = getParamVal(sid, 'ACRO_Y_EXPO'); % Acro yaw expo to allow faster rotation when stick at edges
sim.param.ACRO_YAW_P = getParamVal(sid, 'ACRO_YAW_P');

% Abort script if frame configuration is not defined yet
if (isempty(sim.param.AXIS_FAC_MOTORS) || sim.param.NUM_MOTORS == 0)
    error("Body configuration of copter is not defined yet!");
end

%% Simulation duration
% Define end time of simulation
if sim.param.mdlVal || sim.param.rateCtrlVal || sim.param.attCtrlVal || sim.param.altCtrlVal
    if isinf(tEnd) % Run full data
        sim.param.timeEnd = sid.RATE.TimeS(end);
    else % Only run part of data
        duration = tEnd-tStart;
        iEnd = int16(duration/dt);
        sim.param.timeEnd = sid.RATE.TimeS(iEnd);
        clear duration iEnd
    end
elseif sim.param.optCtrl
    sim.param.timeEnd = OPT.tEndOpt;
end

%% Inertial Sensor
sim.param.INS.GYRO_FILTER = getParamVal(sid, 'INS_GYRO_FILTER'); % Filter cutoff frequency for gyro lowpass filter
sim.param.INS.GYRO_RATE = 400; % Gyro sampling rate. Actually define by INS_GYRO_RATE, but simulation is run with 400 Hz

%% AHRS
trim_x = getParamVal(sid, 'AHRS_TRIM_X');
trim_y = getParamVal(sid, 'AHRS_TRIM_Y');
sim.param.AHRS.FC2BF = single(dcmFromEuler(trim_x, trim_y, 0)); % Rotation matrix from FC to Body Frame based on AP_AHRS constructor
sim.param.AHRS.BF2FC = sim.param.AHRS.FC2BF';

clear trim_x trim_y
%% RCIN - Radio Input and parameters for Attitude Control
rollCh = getParamVal(sid, 'RCMAP_ROLL');
pitchCh = getParamVal(sid, 'RCMAP_PITCH');
yawCh = getParamVal(sid, 'RCMAP_YAW');
thrCh = getParamVal(sid, 'RCMAP_THROTTLE');

sim.param.RCIN.DZ_RLL = getParamVal(sid, ['RC' num2str(rollCh) '_DZ']);
sim.param.RCIN.MIN_RLL = getParamVal(sid, ['RC' num2str(rollCh) '_MIN']);
sim.param.RCIN.MAX_RLL = getParamVal(sid, ['RC' num2str(rollCh) '_MAX']);
sim.param.RCIN.TRIM_RLL = getParamVal(sid, ['RC' num2str(rollCh) '_TRIM']);
sim.param.RCIN.REVERSED_RLL = getParamVal(sid, ['RC' num2str(rollCh) '_REVERSED']);
sim.param.RCIN.DZ_PIT = getParamVal(sid, ['RC' num2str(pitchCh) '_DZ']);
sim.param.RCIN.MIN_PIT = getParamVal(sid, ['RC' num2str(pitchCh) '_MIN']);
sim.param.RCIN.MAX_PIT = getParamVal(sid, ['RC' num2str(pitchCh) '_MAX']);
sim.param.RCIN.TRIM_PIT = getParamVal(sid, ['RC' num2str(pitchCh) '_TRIM']);
sim.param.RCIN.REVERSED_PIT = getParamVal(sid, ['RC' num2str(pitchCh) '_REVERSED']);
sim.param.RCIN.DZ_YAW = getParamVal(sid, ['RC' num2str(yawCh) '_DZ']);
sim.param.RCIN.MIN_YAW = getParamVal(sid, ['RC' num2str(yawCh) '_MIN']);
sim.param.RCIN.MAX_YAW = getParamVal(sid, ['RC' num2str(yawCh) '_MAX']);
sim.param.RCIN.TRIM_YAW = getParamVal(sid, ['RC' num2str(yawCh) '_TRIM']);
sim.param.RCIN.REVERSED_YAW = getParamVal(sid, ['RC' num2str(yawCh) '_REVERSED']);
sim.param.RCIN.DZ_THR = getParamVal(sid, ['RC' num2str(thrCh) '_DZ']);
sim.param.RCIN.MIN_THR = getParamVal(sid, ['RC' num2str(thrCh) '_MIN']);
sim.param.RCIN.MAX_THR = getParamVal(sid, ['RC' num2str(thrCh) '_MAX']);
sim.param.RCIN.TRIM_THR = getParamVal(sid, ['RC' num2str(thrCh) '_TRIM']);
sim.param.RCIN.REVERSED_THR = getParamVal(sid, ['RC' num2str(thrCh) '_REVERSED']);

sim.param.RCIN.ROLL_PITCH_YAW_INPUT_MAX = 4500; % Yaw, roll, pitch input range, defined in config.h

% Check if RCIN log is available
if sim.param.mdlVal || sim.param.attCtrlVal || sim.param.altCtrlVal
    if isfield(sid, 'RCIN')
        iVec = (sid.RCIN.TimeS >= tStart & sid.RCIN.TimeS <= tEnd);
        sim.signals.RCIN.Time = single(sid.RCIN.TimeS(iVec)-tStart);
        sim.signals.RCIN.RollInput = eval(['single(sid.RCIN.C' num2str(rollCh) '(iVec))']);
        sim.signals.RCIN.PitchInput = eval(['single(sid.RCIN.C' num2str(pitchCh) '(iVec))']);
        sim.signals.RCIN.YawInput = eval(['single(sid.RCIN.C' num2str(yawCh) '(iVec))']);
        sim.signals.RCIN.ThrottleInput = eval(['single(sid.RCIN.C' num2str(thrCh) '(iVec))']);
    else
        if ~isfield(sid, 'RCIN') && axis == 13 
            warning(['RCIN message cannot be found in log ' num2str(log_idx) '! Validating vertical motion model.']);
        else
            warning(['RCIN message cannot be found in log ' num2str(log_idx) '! Ommiting attitude controller and models in simulation. Only rate controller is validated']);
            sim.param.rateCtrlVal = 1;
            sim.param.mdlVal = 0;
        end
        % Set RCIN signals to zero
        sim.signals.RCIN.Time = single(0:0.1:sim.param.timeEnd);
        sim.signals.RCIN.RollInput = single(zeros(length(sim.signals.RCIN.Time),1));
        sim.signals.RCIN.PitchInput = single(zeros(length(sim.signals.RCIN.Time),1));
        sim.signals.RCIN.YawInput = single(zeros(length(sim.signals.RCIN.Time),1));
        sim.signals.RCIN.ThrottleInput = single(zeros(length(sim.signals.RCIN.Time),1));
    end
end

%% Attitude
% Yaw Angle signals are a modulo of 360°, so this operation has to be
% inverted
iVec = (sid.ATT.TimeS >= tStart & sid.ATT.TimeS <= tEnd);
sim.signals.ATT.Time = single(sid.ATT.TimeS(iVec)-tStart);
sim.signals.ATT.Roll = single(sid.ATT.Roll(iVec));
sim.signals.ATT.Pitch = single(sid.ATT.Pitch(iVec));
sim.signals.ATT.Yaw = single(demodYawAngle(sid.ATT.Yaw(iVec)));
sim.signals.ATT.DesRoll = single(sid.ATT.DesRoll(iVec));
sim.signals.ATT.DesPitch = single(sid.ATT.DesPitch(iVec));
sim.signals.ATT.DesYaw = single(demodYawAngle(sid.ATT.DesYaw(iVec)));

% The actual yaw angle is calculated from the DCM with
% Matrix3<T>::to_euler() with the four-quadrant arcus tangens atan2(), for 
% example in AP_AHRS_View::update(), so the controller can only handle 
% values in the range [-pi, pi]. 
sim.signals.ATT.Yaw = sim.signals.ATT.Yaw - 360 * (sim.signals.ATT.Yaw > 180);
sim.signals.ATT.DesYaw = sim.signals.ATT.DesYaw - 360 * (sim.signals.ATT.DesYaw > 180);

% Initial DCM matrix based on initial Euler angles
% Calculation based on Matrix3<T>::from_euler()
for i=1:length(iVec)
    if iVec(i) == 1
        iInit = i;
        break;
    end 
end
rollAngInit = single(sid.ATT.Roll(iInit))*pi/180;
pitchAngInit = single(sid.ATT.Pitch(iInit))*pi/180;
yawAngInit = single(sim.signals.ATT.Yaw(1))*pi/180;
sim.init.ATT.Roll = rollAngInit;
sim.init.ATT.Pitch = pitchAngInit;
sim.init.ATT.Yaw = yawAngInit;
sim.init.ATT.DCM = dcmFromEuler(rollAngInit, pitchAngInit, yawAngInit);

% Initialize attitudeTarget according to
% AC_AttitudeControl::reset_yaw_target_and_rate(), which is called in
% ModeStabilize::run() when copter is landed
attitudeTargetUpdate = fromAxisAngle([0;0;yawAngInit]);
sim.init.ATT.attitudeTarget = quatMult(attitudeTargetUpdate, [1;0;0;0]);
sim.init.ATT.eulerRateTar = [0;0;0];

% Test signals in SITL
% % Attitude Controller Targets
% sim.signals.ATT.attTarMeasW = single(sid.ATAR.attW);
% sim.signals.ATT.attTarMeasX = single(sid.ATAR.attX);
% sim.signals.ATT.attTarMeasY = single(sid.ATAR.attY);
% sim.signals.ATT.attTarMeasZ = single(sid.ATAR.attZ);
% sim.signals.ATT.eulerAngTarMeasX = single(sid.ATAR.angX);
% sim.signals.ATT.eulerAngTarMeasY = single(sid.ATAR.angY);
% sim.signals.ATT.eulerAngTarMeasZ = single(sid.ATAR.angZ);
% sim.signals.ATT.eulerRatTarMeasX = single(sid.ATAR.ratX);
% sim.signals.ATT.eulerRatTarMeasY = single(sid.ATAR.ratY);
% sim.signals.ATT.eulerRatTarMeasZ = single(sid.ATAR.ratZ);
% 
% sim.signals.ATT.CtrlRollIn = single(sid.CTIN.rllCtrl);
% sim.signals.ATT.CtrlPitchIn = single(sid.CTIN.pitCtrl);
% sim.signals.ATT.CtrlYawIn = single(sid.CTIN.yawCtrl);
% sim.signals.ATT.yawRateDesMeas = single(sid.ATIN.yawDes);

clear rollAngInit pitchAngInit yawAngInit attitudeTargetUpdate iInit

%% Attitude Controller General
sim.param.ATC.RATE_FF_ENAB = getParamVal(sid, 'ATC_RATE_FF_ENAB');
sim.param.ATC.INPUT_TC = getParamVal(sid, 'ATC_INPUT_TC'); % Attitude control input time constant
% Roll Angle Controller
sim.param.ATC.ANG_RLL_P = getParamVal(sid, 'ATC_ANG_RLL_P');
sim.param.ATC.ACCEL_R_MAX = getParamVal(sid, 'ATC_ACCEL_R_MAX');
sim.param.ATC.RATE_R_MAX = getParamVal(sid, 'ATC_RATE_R_MAX');
sim.param.ATC.ACCEL_RP_CONTROLLER_MIN_RADSS = 40*pi/180; % Maximum body-frame acceleration limit for the stability controller, defined in AC_AttitudeControl.h
sim.param.ATC.ACCEL_RP_CONTROLLER_MAX_RADSS = 720*pi/180;
% Pitch Angle Controller
sim.param.ATC.ANG_PIT_P = getParamVal(sid, 'ATC_ANG_PIT_P');
sim.param.ATC.ACCEL_P_MAX = getParamVal(sid, 'ATC_ACCEL_P_MAX');
sim.param.ATC.RATE_P_MAX = getParamVal(sid, 'ATC_RATE_P_MAX');
% Yaw Angle Controller
sim.param.ATC.ANG_YAW_P = getParamVal(sid, 'ATC_ANG_YAW_P');
sim.param.ATC.ACCEL_Y_MAX = getParamVal(sid, 'ATC_ACCEL_Y_MAX');
sim.param.ATC.RATE_Y_MAX = getParamVal(sid, 'ATC_RATE_Y_MAX');
sim.param.ATC.ACCEL_Y_CONTROLLER_MAX_RADSS = single(120*pi/180); % Maximum body-frame acceleration limit for the stability controller, defined in AC_AttitudeControl.h
sim.param.ATC.ACCEL_Y_CONTROLLER_MIN_RADSS = single(10*pi/180);
sim.param.ATC.THRUST_ERROR_ANGLE = single(30*pi/180); % Thrust angle error above which yaw corrections are limited. Defined in AC_AttitudeControl.h

% Thrust and throttle calculation parameters
sim.param.MODE.THR_MAN_FLG = thr_man;    % Flag for manual throttle, which depends on the current mode. 
if thr_man
    sim.param.ATC.THR_FILT_CUTOFF = getParamVal(sid, 'PILOT_THR_FILT'); % Cutoff frequency of throttle filter for modes with manual throttle. parameter is given to function call of AC_AttitudeControl_Multi::set_throttle_out().
else
    sim.param.ATC.THR_FILT_CUTOFF = 2.0; % Cutoff frequency of throttle filter for modes with z position control (Defined in AC_PosControl.h, line 33)
end
sim.param.ATC.ANGLE_BOOST = getParamVal(sid, 'ATC_ANGLE_BOOST');   % Enabling of angle boost to increase output throttle. Used in AC_AttitudeConrtol_Multi::get_throttle_boosted()
sim.param.ATC.THR_MIX_MAN = getParamVal(sid, 'ATC_THR_MIX_MAN');   % Throttle vs. attitude priorisation during manual flight
sim.param.ATC.THR_MIX_MIN = getParamVal(sid, 'ATC_THR_MIX_MIN');   % Throttle vs. attitude priorisation used when landing
sim.param.ATC.THR_MIX_MAX = getParamVal(sid, 'ATC_THR_MIX_MAX');   % Throttle vs. attitude priorisation during active flights
sim.param.ATC.THR_MIX_DFLT = single(0.5); % Default value for the priorization of throttle use between attitude control and throttle demand by pilot (or autopilot). Defined in AC_AttitudeControl.h, line 44 
if thr_man
    sim.init.ATC.THR_MIX = sim.param.ATC.THR_MIX_MAN;
else
    sim.init.ATC.THR_MIX = sim.param.ATC.THR_MIX_DFLT;
end
sim.param.MOT.HOVER_LEARN = getParamVal(sid, 'MOT_HOVER_LEARN');     % Enable/Disable automatic learning of hover throttle (0=Disabled, 1=Learn, 2=Learn and Save
sim.param.MOT.THST_HOVER = getParamVal(sid, 'MOT_THST_HOVER');    % Motor thrust needed to hover. Default value of _throttle_hover.
sim.param.MOT.THST_HOVER = sim.param.MOT.THST_HOVER(1); % Assign first value of array to default value which is the correct value in the case, that the parameter is defined twice in the param file.
sim.param.MOT.THST_HOVER_TC = single(10.0);                                         % Time constant used to update estimated hover throttle, found as AP_MOTORS_THST_HOVER_TC in AP_MotorsMulticopter.h
sim.param.MOT.THST_HOVER_MIN = single(0.125);                                       % Minimum possible hover throttle, found as AP_MOTORS_THST_HOVER_MIN in AP_MotorsMulticopter.h
sim.param.MOT.THST_HOVER_MAX = single(0.6875);                                      % Maximum possible hover throttle, found as AP_MOTORS_THST_HOVER_MAX in AP_MotorsMulticopter.h
sim.param.MOT.BAT_VOLT_MAX = getParamVal(sid, 'MOT_BAT_VOLT_MAX');   % Maximum voltage above which no additional scaling on thrust is performed
sim.param.MOT.BAT_VOLT_MIN = getParamVal(sid, 'MOT_BAT_VOLT_MIN');   % Minimum voltage below which no additional scaling on thrust is performed
sim.param.MOT.BAT_CURR_MAX = getParamVal(sid, 'MOT_BAT_CURR_MAX');   % Maximum current over which maximum throttle is limited (and no further scaling is performed)
sim.param.MOT.THST_EXPO = getParamVal(sid, 'MOT_THST_EXPO');             % Motor thrust curve exponent (0.0 for linear to 1.0 for second order curve)
sim.param.MOT.BAT_CURR_TC = getParamVal(sid, 'MOT_BAT_CURR_TC');               % Time constant used to limit the maximum current
sim.param.MOT.YAW_HEADROOM = getParamVal(sid, 'MOT_YAW_HEADROOM');             % Yaw control is goven at least this PWM in microseconds range

% Throttle inputs
iVec = (sid.CTUN.TimeS >= tStart & sid.CTUN.TimeS <= tEnd);
sim.signals.CTUN.Time = single(sid.CTUN.TimeS(iVec)--tStart);
sim.signals.CTUN.ThrIn = single(sid.CTUN.ThI(iVec)); % Throttle In, pilots input to Attitude Control (attitude_control->get_throttle_in())
sim.signals.CTUN.ThrOut = single(sid.CTUN.ThO(iVec)); % Throttle Out, throttle given to motor mixer after filtering (motors->get_throttle())
sim.signals.CTUN.AngBst = single(sid.CTUN.ABst(iVec)); % Extra amount of throttle, added to throttle_in in AC_AttitudeControl_Multi::get_throttle_boosted()
sim.signals.CTUN.ThrHov = single(sid.CTUN.ThH(iVec)); % Estimated throttle required to hover throttle in range 0-1 (AP_MotorsMulticopter::get_throttle_hover())

% Throttle inital values
if sim.param.rateCtrlVal || sim.param.attCtrlVal || sim.param.mdlVal || sim.param.altCtrlVal
    sim.init.CTUN.ThrOut = single(sid.CTUN.ThO(1));
    sim.init.CTUN.ThrHov = single(sid.CTUN.ThH(1));
else
    sim.init.CTUN.ThrOut = single(0);
    sim.init.CTUN.ThrHov = single(sim.param.MOT.THST_HOVER);
end

% Altitude signals
sim.signals.CTUN.DSAlt = single(sid.CTUN.DSAlt(iVec)); % Desired rangefinder altitude
sim.signals.CTUN.DAlt = single(sid.CTUN.DAlt(iVec)); % Desired altitude
sim.signals.CTUN.Alt = single(sid.CTUN.Alt(iVec)); % Achieved altitude (inav)
sim.signals.CTUN.SAlt = single(sid.CTUN.SAlt(iVec)); % Achieved rangefinder altitude
sim.signals.CTUN.TAlt = single(sid.CTUN.TAlt(iVec)); % Terrain altitude
sim.signals.CTUN.BAlt = single(sid.CTUN.BAlt(iVec)); % Barometric altitude
sim.signals.CTUN.CRt = single(sid.CTUN.CRt(iVec)); % Climb rate inav
sim.signals.CTUN.DCRt = single(sid.CTUN.DCRt(iVec)); % Desired climb rate 

% Altitude inital values
sim.init.CTUN.Alt = single(sid.CTUN.Alt(1));
sim.init.CTUN.CRt = single(sid.CTUN.CRt(1)*0.01);

% Intermediate throttle calculation quantities - only read if message
% exists
if isfield(sid, 'MOTQ')
    iVec = (sid.MOTQ.TimeS >= tStart & sid.MOTQ.TimeS <= tEnd);
    sim.signals.MOTQ.Time = single(sid.MOTQ.TimeS(iVec)-tStart);
    sim.signals.MOTQ.ThrAvgMax = single(sid.MOTQ.ThAvgMax(iVec));
    sim.signals.MOTQ.ThrThstMaxMeas = single(sid.MOTQ.ThThstMax(iVec));
    sim.signals.MOTQ.CompGain = single(sid.MOTQ.CompGain(iVec));
    sim.signals.MOTQ.AirDensityRatio = single(sid.MOTQ.AirDensRat(iVec));
    sim.signals.MOTQ.ThrMixOut = single(sid.MOTQ.ThrOut(iVec));
else
    iVec = (sid.CTUN.TimeS >= tStart & sid.CTUN.TimeS <= tEnd);
    sim.signals.MOTQ.Time = single(sid.CTUN.TimeS(iVec)-tStart);
    sim.signals.MOTQ.ThrAvgMax = single(zeros(length(sim.signals.MOTQ.Time),1));
    sim.signals.MOTQ.ThrThstMaxMeas = single(zeros(length(sim.signals.MOTQ.Time),1));
    sim.signals.MOTQ.CompGain = single(zeros(length(sim.signals.MOTQ.Time),1));
    sim.signals.MOTQ.AirDensityRatio = single(zeros(length(sim.signals.MOTQ.Time),1));
    sim.signals.MOTQ.ThrMixOut = single(zeros(length(sim.signals.MOTQ.Time),1));
end


% Battery inputs
iVec = (sid.BAT.TimeS >= tStart & sid.BAT.TimeS <= tEnd);
sim.signals.BAT.Time = single(sid.BAT.TimeS(iVec)-tStart);
sim.signals.BAT.RestVoltEst = single(sid.BAT.VoltR(iVec)); % Estimated resting voltage of the battery
sim.signals.BAT.Curr = single(sid.BAT.Curr(iVec)); % Measured battery current
sim.signals.BAT.Res = single(sid.BAT.Res(iVec)); % Estimated battery resistance
sim.signals.BAT.Volt = single(sid.BAT.Volt(iVec)); % Measured voltage resistance

% Battery inital values
if sim.param.rateCtrlVal || sim.param.attCtrlVal || sim.param.mdlVal || sim.param.altCtrlVal
    sim.init.BAT.RestVoltEst = single(sid.BAT.VoltR(1));
else
    sim.init.BAT.RestVoltEst = single(0);
end

% Baro inputs
iVec = (sid.BARO.TimeS >= tStart & sid.BARO.TimeS <= tEnd);
sim.signals.BARO.Time = single(sid.BARO.TimeS(iVec)-tStart);
sim.signals.BARO.Alt = single(sid.BARO.Alt(iVec));     % Calculated altitude by the barometer
sim.signals.BARO.AirPress = single(sid.BARO.Press(iVec));  % Measured atmospheric pressure by the barometer
sim.signals.BARO.GndTemp = single(sid.BARO.GndTemp(iVec)); % Temperature on ground, specified by parameter or measured while on ground

% Attitude controller outputs
iVec = (sid.MOTB.TimeS >= tStart & sid.MOTB.TimeS <= tEnd);
sim.signals.MOTB.Time = single(sid.MOTB.TimeS(iVec)-tStart);
sim.signals.MOTB.LiftMax = single(sid.MOTB.LiftMax(iVec)); % Maximum motor compensation gain, calculated in AP_MotorsMulticopter::update_lift_max_from_batt_voltage()
sim.signals.MOTB.ThrLimit = single(sid.MOTB.ThLimit(iVec)); % Throttle limit set due to battery current limitations, calculated in AP_MotorsMulticopter::get_current_limit_max_throttle()

% SID Inputs
% Create array for sid signals, each element containing the signal struct
if (mode == 25)
    iVec = (sid.SIDD.TimeS >= tStart & sid.SIDD.TimeS <= tEnd);
    for i=1:13
        sim.signals.SIDD.Time{i} = single(sid.SIDD.TimeS(iVec)-tStart);
        if (i == axis)
            sim.signals.SIDD.Chirp{i} = single(sid.SIDD.Targ(iVec)); % SID test signal
        else
            sim.signals.SIDD.Chirp{i} = single(zeros(length(sim.signals.SIDD.Time{i}),1));
        end
    end
    sim.signals.SIDD.AccX = single(sid.SIDD.Ax(iVec));
    sim.signals.SIDD.AccY = single(sid.SIDD.Ay(iVec));
    sim.signals.SIDD.AccZ = single(sid.SIDD.Az(iVec));
    sim.signals.SIDD.GyrX = single(sid.SIDD.Gx(iVec));
    sim.signals.SIDD.GyrY = single(sid.SIDD.Gy(iVec));
    sim.signals.SIDD.GyrZ = single(sid.SIDD.Gz(iVec));
else
    iVec = (sid.RATE.TimeS >= tStart & sid.RATE.TimeS <= tEnd);
    for i=1:13
        sim.signals.SIDD.Time{i} = single(sid.RATE.TimeS(iVec)-tStart);
        sim.signals.SIDD.Chirp{i} = single(zeros(length(sim.signals.SIDD.Time{i}),1));
    end
    sim.signals.SIDD.AccX = single(zeros(length(sim.signals.SIDD.Time{1}),1));
    sim.signals.SIDD.AccY = single(zeros(length(sim.signals.SIDD.Time{1}),1));
    sim.signals.SIDD.AccZ = single(zeros(length(sim.signals.SIDD.Time{1}),1));
    sim.signals.SIDD.GyrX = single(zeros(length(sim.signals.SIDD.Time{1}),1));
    sim.signals.SIDD.GyrY = single(zeros(length(sim.signals.SIDD.Time{1}),1));
    sim.signals.SIDD.GyrZ = single(zeros(length(sim.signals.SIDD.Time{1}),1));
end

%% Roll Rate Controller
iVec = (sid.PIDR.TimeS >= tStart & sid.PIDR.TimeS <= tEnd);
sim.signals.PIDR.Time = single(sid.PIDR.TimeS(iVec)-tStart);
% Inputs
sim.signals.PIDR.Tar = single(sid.PIDR.Tar(iVec)); % target values filtered
sim.signals.PIDR.Act = single(sid.PIDR.Act(iVec)); % actual values
sim.signals.PIDR.Err = single(sid.PIDR.Err(iVec)); % error values
if sim.param.rateCtrlVal || sim.param.attCtrlVal || sim.param.mdlVal
    % Clock of Slew Limiter in ms
    % Use tracked logging time of PID message in validation modes
    sim.signals.PIDR.ClockDMod = single(1000*sim.signals.PIDR.Time); 
else
    sim.signals.PIDR.ClockDMod = single(1000 * sim.param.dt * (0:length(sim.signals.PIDR.Time)-1)');
end
    
% Outputs
sim.signals.PIDR.FF = single(sid.PIDR.FF(iVec));
sim.signals.PIDR.P = single(sid.PIDR.P(iVec));
sim.signals.PIDR.I = single(sid.PIDR.I(iVec));
sim.signals.PIDR.D = single(sid.PIDR.D(iVec));
sim.signals.PIDR.DMod = single(sid.PIDR.Dmod(iVec));
sim.signals.PIDR.ILimit = single(sid.PIDR.Limit(iVec));
sim.signals.PIDR.SRate = single(sid.PIDR.SRate(iVec)); % Output slew rate of the slew limiter, stored in _output_slew_rate in AC_PID.cpp, line 161

% parameters - Read from PARM.Value cell array with logical indexing
sim.param.PIDR.TC = getParamVal(sid, 'ATC_INPUT_TC');
sim.param.PIDR.FLTT_f = getParamVal(sid, 'ATC_RAT_RLL_FLTT');
sim.param.PIDR.FLTE_f = getParamVal(sid, 'ATC_RAT_RLL_FLTE');
sim.param.PIDR.FLTD_f = getParamVal(sid, 'ATC_RAT_RLL_FLTD');
sim.param.PIDR.P = getParamVal(sid, 'ATC_RAT_RLL_P');
sim.param.PIDR.I = getParamVal(sid, 'ATC_RAT_RLL_I');
sim.param.PIDR.D = getParamVal(sid, 'ATC_RAT_RLL_D');
sim.param.PIDR.IMAX = getParamVal(sid, 'ATC_RAT_RLL_IMAX');
sim.param.PIDR.FF = getParamVal(sid, 'ATC_RAT_RLL_FF'); %0.05;
sim.param.PIDR.SR_MAX = getParamVal(sid, 'ATC_RAT_RLL_SMAX'); %5.0;
sim.param.PIDR.SR_TAU = single(1.0); % Slew Rate Tau - not yet available as a parameter of the copter. Set to 1.0 by default (AC_PID.h, line 24).
sim.param.PIDR.SR_FLT_f = single(25.0);  % Slew Rate lowpass filter cutoff frequency. Defined in SlewLimiter.h, line 14.

% Inital inputs
if sim.param.rateCtrlVal || sim.param.attCtrlVal || sim.param.mdlVal
    sim.init.PIDR.P = single(sid.PIDR.P(1));
    sim.init.PIDR.I = single(sid.PIDR.I(1));
    sim.init.PIDR.D = single(sid.PIDR.D(1));
    sim.init.PIDR.Tar = single(sid.RATE.RDes(1)*pi/180); % Convert to radian due to conversion to degree for logging (AP_AHRS_View::Write_Rate)
    sim.init.PIDR.TarFilt = single(sid.PIDR.Tar(1));
    sim.init.PIDR.ErrFilt = single(sid.PIDR.Err(1));
    sim.init.PIDR.SROut = single(sid.PIDR.SRate(1)); % Initial value of the slew rate determined by the slew limiter. Used for both modifier_slew_rate and output_slew_rate.
    if (sid.PIDR.D(1) ~= 0) % Prevent division by zero
        sim.init.PIDR.DerFilt = single(sid.PIDR.D(1))/sim.param.PIDR.D;
    else
        sim.init.PIDR.DerFilt = single(0);
    end
else % Set to zero if we do not want to validate model with test run
    sim.init.PIDR.P = single(0);
    sim.init.PIDR.I = single(0);
    sim.init.PIDR.D = single(0);
    sim.init.PIDR.Tar = single(0);
    sim.init.PIDR.TarFilt = single(0);
    sim.init.PIDR.ErrFilt = single(0);
    sim.init.PIDR.DerFilt = single(0);
    sim.init.PIDR.SROut = single(0);
end

%% Pitch Rate Controller
iVec = (sid.PIDP.TimeS >= tStart & sid.PIDP.TimeS <= tEnd);
sim.signals.PIDP.Time = single(sid.PIDP.TimeS(iVec)-tStart);
% Inputs
sim.signals.PIDP.Tar = single(sid.PIDP.Tar(iVec)); % target values filtered
sim.signals.PIDP.Act = single(sid.PIDP.Act(iVec)); % actual values
sim.signals.PIDP.Err = single(sid.PIDP.Err(iVec)); % error values                                          
if sim.param.rateCtrlVal || sim.param.attCtrlVal || sim.param.mdlVal
    % Clock of Slew Limiter
    % Use tracked logging time of PID message in validation modes
    sim.signals.PIDP.ClockDMod = single(1000 * sim.signals.PIDP.Time); 
else
    sim.signals.PIDP.ClockDMod = single(1000 * sim.param.dt * (0:length(sim.signals.PIDP.Time)-1)');
end

% Outputs
sim.signals.PIDP.FF = single(sid.PIDP.FF(iVec));
sim.signals.PIDP.P = single(sid.PIDP.P(iVec));
sim.signals.PIDP.I = single(sid.PIDP.I(iVec));
sim.signals.PIDP.D = single(sid.PIDP.D(iVec));
sim.signals.PIDP.DMod = single(sid.PIDP.Dmod(iVec));
sim.signals.PIDP.ILimit = single(sid.PIDP.Limit(iVec));
sim.signals.PIDP.SRate = single(sid.PIDP.SRate(iVec)); % Output slew rate of the slew limiter, stored in _output_slew_rate in AC_PID.cpp, line 161

% Parameters - Read from PARM.Value cell array with logical indexing
sim.param.PIDP.TC = getParamVal(sid, 'ATC_INPUT_TC');
sim.param.PIDP.FLTT_f = getParamVal(sid, 'ATC_RAT_PIT_FLTT');
sim.param.PIDP.FLTE_f = getParamVal(sid, 'ATC_RAT_PIT_FLTE');
sim.param.PIDP.FLTD_f = getParamVal(sid, 'ATC_RAT_PIT_FLTD');
sim.param.PIDP.P = getParamVal(sid, 'ATC_RAT_PIT_P');
sim.param.PIDP.I = getParamVal(sid, 'ATC_RAT_PIT_I');
sim.param.PIDP.D = getParamVal(sid, 'ATC_RAT_PIT_D');
sim.param.PIDP.IMAX = getParamVal(sid, 'ATC_RAT_PIT_IMAX');
sim.param.PIDP.FF = getParamVal(sid, 'ATC_RAT_PIT_FF');
sim.param.PIDP.SR_MAX = getParamVal(sid, 'ATC_RAT_PIT_SMAX');
sim.param.PIDP.SR_TAU = single(1.0); % Slew Rate Tau - not yet available as a parameter of the copter. Set to 1.0 by default (AC_PID.h, line 24).
sim.param.PIDP.SR_FLT_f = single(25.0);  % Slew Rate lowpass filter cutoff frequency. Defined in SlewLimiter.h, line 14.

% Inital inputs
if sim.param.rateCtrlVal || sim.param.attCtrlVal || sim.param.mdlVal
    sim.init.PIDP.P = single(sid.PIDP.P(1));
    sim.init.PIDP.I = single(sid.PIDP.I(1));
    sim.init.PIDP.D = single(sid.PIDP.D(1));
    sim.init.PIDP.Tar = single(sid.RATE.PDes(1)*pi/180); % Convert to radian due to conversion to degree for logging (AP_AHRS_View::Write_Rate)
    sim.init.PIDP.TarFilt = single(sid.PIDP.Tar(1));
    sim.init.PIDP.ErrFilt = single(sid.PIDP.Err(1));
    sim.init.PIDP.SrOut = single(sid.PIDP.SRate(1)); % Initial value of the slew rate determined by the slew limiter. Used for both modifier_slew_rate and output_slew_rate.
    if (sim.param.PIDP.D ~= 0)
        sim.init.PIDP.DerFilt = single(sid.PIDP.D(1))/sim.param.PIDP.D;
    else
        sim.init.PIDP.DerFilt = single(0);
    end
else
    sim.init.PIDP.P = single(0);
    sim.init.PIDP.I = single(0);
    sim.init.PIDP.D = single(0);
    sim.init.PIDP.Tar = single(0);
    sim.init.PIDP.TarFilt = single(0);
    sim.init.PIDP.ErrFilt = single(0);
    sim.init.PIDP.DerFilt = single(0);
    sim.init.PIDP.SrOut = single(0);
end

%% Yaw Rate Controller
iVec = (sid.PIDY.TimeS >= tStart & sid.PIDY.TimeS <= tEnd);
sim.signals.PIDY.Time = single(sid.PIDY.TimeS(iVec)-tStart);
% Inputs
sim.signals.PIDY.Tar = single(sid.PIDY.Tar(iVec)); % target values filtered
sim.signals.PIDY.Act = single(sid.PIDY.Act(iVec)); % actual values
sim.signals.PIDY.Err = single(sid.PIDY.Err(iVec)); % error values                                          
if sim.param.rateCtrlVal || sim.param.attCtrlVal || sim.param.mdlVal
    % Clock of Slew Limiter
    % Use tracked logging time of PID message in validation modes
    sim.signals.PIDY.ClockDMod = single(1000 * sim.signals.PIDY.Time); 
else
    sim.signals.PIDY.ClockDMod = single(1000 * sim.param.dt * (0:length(sim.signals.PIDY.Time)-1)');
end

% Outputs
sim.signals.PIDY.FF = single(sid.PIDY.FF(iVec));
sim.signals.PIDY.P = single(sid.PIDY.P(iVec));
sim.signals.PIDY.I = single(sid.PIDY.I(iVec));
sim.signals.PIDY.D = single(sid.PIDY.D(iVec));
sim.signals.PIDY.DMod = single(sid.PIDY.Dmod(iVec));
sim.signals.PIDY.ILimit = single(sid.PIDY.Limit(iVec));
sim.signals.PIDY.SRate = single(sid.PIDY.SRate(iVec)); % Output slew rate of the slew limiter, stored in _output_slew_rate in AC_PID.cpp, line 161

% Parameters - Read from PARM.Value cell array with logical indexing
sim.param.PIDY.TC = getParamVal(sid, 'ATC_INPUT_TC');
sim.param.PIDY.FLTT_f = getParamVal(sid, 'ATC_RAT_YAW_FLTT');
sim.param.PIDY.FLTE_f = getParamVal(sid, 'ATC_RAT_YAW_FLTE');
sim.param.PIDY.FLTD_f = getParamVal(sid, 'ATC_RAT_YAW_FLTD');
sim.param.PIDY.P = getParamVal(sid, 'ATC_RAT_YAW_P');
sim.param.PIDY.I = getParamVal(sid, 'ATC_RAT_YAW_I');
sim.param.PIDY.D = getParamVal(sid, 'ATC_RAT_YAW_D');
sim.param.PIDY.IMAX = getParamVal(sid, 'ATC_RAT_YAW_IMAX');
sim.param.PIDY.FF = getParamVal(sid, 'ATC_RAT_YAW_FF');
sim.param.PIDY.SR_MAX = getParamVal(sid, 'ATC_RAT_YAW_SMAX');
sim.param.PIDY.SR_TAU = single(1.0); % Slew Rate Tau - not yet available as a parameter of the copter. Set to 1.0 by default (AC_PID.h, line 24).
sim.param.PIDY.SR_FLT_f = single(25.0);  % Slew Rate lowpass filter cutoff frequency. Defined in SlewLimiter.h, line 14.

% Inital inputs
if sim.param.rateCtrlVal || sim.param.attCtrlVal || sim.param.mdlVal
    sim.init.PIDY.P = single(sid.PIDY.P(1));
    sim.init.PIDY.I = single(sid.PIDY.I(1));
    sim.init.PIDY.D = single(sid.PIDY.D(1));
    sim.init.PIDY.Tar = single(sid.RATE.PDes(1)*pi/180); % Convert to radian due to conversion to degree for logging (AP_AHRS_View::Write_Rate)
    sim.init.PIDY.TarFilt = single(sid.PIDY.Tar(1));
    sim.init.PIDY.ErrFilt = single(sid.PIDY.Err(1));
    sim.init.PIDY.SrOut = single(sid.PIDY.SRate(1)); % Initial value of the slew rate determined by the slew limiter. Used for both modifier_slew_rate and output_slew_rate.
    if (sim.param.PIDY.D ~= 0)
        sim.init.PIDY.DerFilt = single(sid.PIDY.D(1))/sim.param.PIDY.D;
    else
        sim.init.PIDY.DerFilt = single(0);
    end
else
    sim.init.PIDY.P = single(0);
    sim.init.PIDY.I = single(0);
    sim.init.PIDY.D = single(0);
    sim.init.PIDY.Tar = single(0);
    sim.init.PIDY.TarFilt = single(0);
    sim.init.PIDY.ErrFilt = single(0);
    sim.init.PIDY.DerFilt = single(0);
    sim.init.PIDY.SrOut = single(0);
end

%% RATE message
iVec = (sid.RATE.TimeS >= tStart & sid.RATE.TimeS <= tEnd);
sim.signals.RATE.Time = single(sid.RATE.TimeS(iVec)-tStart);

% Roll
sim.signals.RATE.ROut = single(sid.RATE.ROut(iVec)); % Rate Controller total output (except FF)
% Rate Controller target (deg/s): Logged signal in val phase, step in opt phase.
if sim.param.rateCtrlVal || sim.param.attCtrlVal || sim.param.mdlVal
    sim.signals.RATE.RDes = single(sid.RATE.RDes(iVec));
elseif sim.param.optCtrl && axis == 10
    sim.signals.RATE.RDes = single((sim.signals.RATE.Time > OPT.stepTime)'*OPT.stepMag); % Step at 5s
else
    sim.signals.RATE.RDes = single(zeros(numel(sim.signals.RATE.Time), 1)); % Set to zero if axis not optimized
end

% Pitch
sim.signals.RATE.POut = single(sid.RATE.POut(iVec));
% Rate Controller target (deg/s): Logged signal in val phase, step in opt phase
if sim.param.rateCtrlVal || sim.param.attCtrlVal || sim.param.mdlVal
    sim.signals.RATE.PDes = single(sid.RATE.PDes(iVec));
elseif sim.param.optCtrl && axis == 11
    sim.signals.RATE.PDes = single((sim.signals.RATE.Time > OPT.stepTime)'*OPT.stepMag); % Step at 5s
else
    sim.signals.RATE.PDes = single(zeros(numel(sim.signals.RATE.Time), 1)); % Set to zero if axis not optimized
end

% Yaw 
sim.signals.RATE.YOut = single(sid.RATE.YOut(iVec));
% Rate Controller target (deg/s): Logged signal in val phase, step in opt phase
if sim.param.rateCtrlVal || sim.param.attCtrlVal || sim.param.mdlVal
    sim.signals.RATE.YDes = single(sid.RATE.YDes(iVec));
elseif sim.param.optCtrl && axis == 12
    sim.signals.RATE.YDes = single((sim.signals.RATE.Time > OPT.stepTime)'*OPT.stepMag); % Step at 5s
else
    sim.signals.RATE.YDes = single(zeros(numel(sim.signals.RATE.Time), 1)); % Set to zero if axis not optimized
end

% Throttle
sim.signals.RATE.AOut = single(sid.RATE.AOut(iVec)); % Throttle command to motors
sim.signals.RATE.A = single(sid.RATE.A(iVec)); % Vertical acceleration in earth frame

%% Position Controller
% Default definitions
sim.param.PSCD.THR_DZ = getParamVal(sid, 'THR_DZ'); % Deadzone above and below mid throttle in PWM microseconds. Used in Althold, Loiter, PosHold. Defined in Parameters.cpp
sim.param.PSCD.VEL_MAX_DOWN_DFLT = single(-150); % Default descent rate in cm/s, defined in AC_PosControl.h, line 27
sim.param.PSCD.VEL_MAX_UP_DFLT = single(250); % Default climb rate in cm/s, defined in AC_PosControl.h, line 28
sim.param.PSCD.ACC_MAX_Z_DFLT = single(250); % Default vertical acceleration cm/s/s, defined in AC_PosControl.h, line 30
sim.param.PSCD.JERK_MAX_Z_DFLT = single(500); % Default vertical jerk, defined as m/s/s/s in AC_PosControl.h, line 31. Converted to cm/s/s/s in AC_PosControl.cpp, line 318.

sim.param.PSCD.VEL_MAX_DN = getParamVal(sid, 'PILOT_SPEED_DN'); % Maximum vertical descending velocity in cm/s, defined in parameters.cpp, line 906
sim.param.PSCD.VEL_MAX_UP = getParamVal(sid, 'PILOT_SPEED_UP'); % Maximum vertical ascending velocity in cm/s, defined in parameters.cpp, line 223
sim.param.PSCD.ACC_MAX_Z = getParamVal(sid, 'PILOT_ACCEL_Z'); % Maximum vertical acceleration used when pilot is controlling the altitude, parameters.cpp, line 232
sim.param.PSCD.JERK_MAX_Z = getParamVal(sid, 'PSC_JERK_Z'); % Jerk limit of vertical kinematic path generation in m/s^3. Determines how quickly aircraft changes acceleration target
sim.param.PSCD.OVERSPEED_GAIN_Z = single(2); % gain controlling rate at which z-axis speed is brought back within SPEED_UP and SPEED_DOWN range, defined in AC_PosControl.h 
%% Position Controller z

% Parameters z position controller
sim.param.PSCD.P_POS = getParamVal(sid, 'PSC_POSZ_P'); % P gain of the vertical position controller

% Parameters z velocity controller
sim.param.PSCD.P_VEL = getParamVal(sid, 'PSC_VELZ_P'); % P gain of the vertical velocity controller
sim.param.PSCD.I_VEL = getParamVal(sid, 'PSC_VELZ_I'); % I gain of the vertical velocity controller
sim.param.PSCD.D_VEL = getParamVal(sid, 'PSC_VELZ_D'); % D gain of the vertical velocity controller
sim.param.PSCD.IMAX_VEL = getParamVal(sid, 'PSC_VELZ_IMAX'); % I gain maximu vertical velocity controller
sim.param.PSCD.FF_VEL = getParamVal(sid, 'PSC_VELZ_FF'); % Feed Forward gain of the vertical velocity controller
sim.param.PSCD.FLTE_VEL = getParamVal(sid, 'PSC_VELZ_FLTE'); % Cutoff frequency of the error filter (in Hz)
sim.param.PSCD.FLTD_VEL = getParamVal(sid, 'PSC_VELZ_FLTD'); % Cutoff frequency of the D term filter (in Hz)
sim.param.PSCD.CTRL_RATIO_INIT = single(2.0); % Initial value of _vel_z_control_ratio, defined in PosControl.h, line 456

% Parameters z acceleration controller
sim.param.PIDA.P = getParamVal(sid, 'PSC_ACCZ_P'); % P gain of the vertical acceleration controller
sim.param.PIDA.I = getParamVal(sid, 'PSC_ACCZ_I'); % I gain of the vertical acceleration controller
sim.param.PIDA.D = getParamVal(sid, 'PSC_ACCZ_D'); % D gain of the vertical acceleration controller
sim.param.PIDA.IMAX = getParamVal(sid, 'PSC_ACCZ_IMAX'); % I gain maximu vertical acceleration controller
sim.param.PIDA.FF = getParamVal(sid, 'PSC_ACCZ_FF'); % Feed Forward gain of the vertical acceleration controller
sim.param.PIDA.FLTE_f = getParamVal(sid, 'PSC_ACCZ_FLTE'); % Cutoff frequency of the error filter (in Hz)
sim.param.PIDA.FLTD_f = getParamVal(sid, 'PSC_ACCZ_FLTD'); % Cutoff frequency of the D term filter (in Hz)
sim.param.PIDA.FLTT_f = getParamVal(sid, 'PSC_ACCZ_FLTT'); % Cutoff frequency of the target filter (in Hz)
sim.param.PIDA.SR_MAX = getParamVal(sid, 'PSC_ACCZ_SMAX'); % Upper limit of the slew rate produced by combined P and D gains
sim.param.PIDA.SR_TAU = single(1.0); % Slew Rate Tau - not yet available as a parameter of the copter. Set to 1.0 by default (AC_PID.h, line 24).
sim.param.PIDA.SR_FLT_f = single(25.0);  % Slew Rate lowpass filter cutoff frequency. Defined in SlewLimiter.h, line 14.

% Read signals for z position controller if PSCD message was logged
if isfield(sid, 'PSCD')
    % Signals of PSCD message
    iVec = (sid.PSCD.TimeS >= tStart & sid.PSCD.TimeS <= tEnd);
    sim.signals.PSCD.Time = single(sid.PSCD.TimeS(iVec)-tStart);
    % Inputs - Multiply by 100 to get cm as used in controller
    % Inversion of signals necessary due to inverted logging
    sim.signals.PSCD.zPosTar = single(-sid.PSCD.TPD(iVec)*100); % Target z-Position, calculated in set_pos_target_z_from_climb_rate_cm()
    sim.signals.PSCD.zPosAct = single(-sid.PSCD.PD(iVec)*100); % Actual z-Position, obtained from INAV
    sim.signals.PSCD.zVelDes = single(-sid.PSCD.DVD(iVec)*100); % Desired z-Velocity, calculated in set_pos_target_z_from_climb_rate_cm()
    sim.signals.PSCD.zVelAct = single(-sid.PSCD.VD(iVec)*100); % Actual z-Velocity, obtained from INAV
    sim.signals.PSCD.zVelTar = single(-sid.PSCD.TVD(iVec)*100); % Target z-Velocity, calculated in update_z_controller()
    sim.signals.PSCD.zAccDes = single(-sid.PSCD.DAD(iVec)*100); % Desired z-Acceleration, calculated in set_pos_target_z_from_climb_rate_cm()
    sim.signals.PSCD.zAccAct = single(-sid.PSCD.AD(iVec)*100); % Actual z-Acceleration, obtained from AHRS  
    sim.signals.PSCD.zAccTar = single(-sid.PSCD.TAD(iVec)*100); % Target z-Acceleration, calculated in update_z_controller()  
    
    % Signals of PIDA message
    iVec = (sid.PIDA.TimeS >= tStart & sid.PIDA.TimeS <= tEnd);
    sim.signals.PIDA.Time = single(sid.PIDA.TimeS(iVec)-tStart);
    % Inputs PID z acceleration
    sim.signals.PIDA.Err = single(sid.PIDA.Err(iVec)); % Error between target and actual z-acceleration
    sim.signals.PIDA.Tar = single(sid.PIDA.Tar(iVec));
    sim.signals.PIDA.Act = single(sid.PIDA.Act(iVec));
    if sim.param.rateCtrlVal || sim.param.attCtrlVal || sim.param.mdlVal || sim.param.altCtrlVal
        % Clock of Slew Limiter
        % Use tracked logging time of PID message in validation modes
        sim.signals.PIDA.ClockDMod = single(1000 * sim.signals.PIDA.Time);
    else
        sim.signals.PIDA.ClockDMod = single(1000 * sim.param.dt * (0:length(sim.signals.PIDA.Time)-1)');
    end
    % Outputs PID z acceleration
    sim.signals.PIDA.FF = single(sid.PIDA.FF(iVec));
    sim.signals.PIDA.P = single(sid.PIDA.P(iVec));
    sim.signals.PIDA.I = single(sid.PIDA.I(iVec));
    sim.signals.PIDA.D = single(sid.PIDA.D(iVec));
    sim.signals.PIDA.DMod = single(sid.PIDA.Dmod(iVec));
    sim.signals.PIDA.ILimit = single(sid.PIDA.Limit(iVec));
    sim.signals.PIDA.SRate = single(sid.PIDA.SRate(iVec));    
    
    % Inital inputs
    if sim.param.rateCtrlVal || sim.param.attCtrlVal || sim.param.mdlVal || sim.param.altCtrlVal
        % Initial actual values - in meter
        sim.init.PSCD.posAct = single(-sid.PSCD.PD(1));
        sim.init.PSCD.velAct = single(-sid.PSCD.VD(1));
        sim.init.PSCD.accAct = single(-sid.PSCD.AD(1));
        
        % Init the position controller to the current position, velocity
        % and acceleration (from AC_PosControl::init_z()) in cm
        sim.init.PIDA.posTar = single(-sid.PSCD.TPD(1)*100);
        sim.init.PIDA.velDes = single(-sid.PSCD.DVD(1)*100);
        sim.init.PIDA.accDes = single(min(max(-sid.PSCD.DAD(1)*100, -sim.param.PSCD.ACC_MAX_Z_DFLT) ,sim.param.PSCD.ACC_MAX_Z_DFLT));
        
        sim.init.PIDA.P = single(sid.PIDA.P(1));
        sim.init.PIDA.I = single(sim.signals.CTUN.ThrIn(1) - sim.signals.CTUN.ThrHov(1)) * 1000; % Integrator init. according to AC_PosControl::init_z_controller()
        sim.init.PIDA.D = single(sid.PIDA.D(1));
        sim.init.PIDA.TarFilt = single(sid.PIDA.Tar(1));
        sim.init.PIDA.ErrFilt = single(sid.PIDA.Err(1));
        sim.init.PIDA.SROut = single(sid.PIDA.SRate(1)); % Initial value of the slew rate determined by the slew limiter. Used for both modifier_slew_rate and output_slew_rate.
        if (sim.param.PIDA.D ~= 0)
            sim.init.PIDA.DerFilt = single(sid.PIDA.D(1))/PSC_ACC_Z.param.D; % Divide through D gain to obtain inital D input
        else
            sim.init.PIDA.DerFilt = single(0);
        end
    else
        % Initial actual values
        sim.init.PSCD.posAct = single(0);
        sim.init.PSCD.velAct = single(0);
        sim.init.PSCD.accAct = single(0);
        
        sim.init.PIDA.posTar = single(0);
        sim.init.PIDA.velDes = single(0);
        sim.init.PIDA.accDes = single(0);
        
        sim.init.PIDA.P = single(0);
        sim.init.PIDA.I = single(0);
        sim.init.PIDA.D = single(0);
        sim.init.PIDA.TarFilt = single(0);
        sim.init.PIDA.ErrFilt = single(0);
        sim.init.PIDA.DerFilt = single(0);
        sim.init.PIDA.SROut = single(0);
    end
else
    % Set all signals to zero, if PSCD message was not logged and z
    % controller has been deactivated
    iVec = (sid.RATE.TimeS >= tStart & sid.RATE.TimeS <= tEnd);
    sim.signals.PSCD.Time = single(sid.RATE.TimeS(iVec)-tStart);
    sim.signals.PSCD.zPosTar = single(zeros(length(sim.signals.PSCD.Time),1)); % Target z-Position, calculated in set_pos_target_z_from_climb_rate_cm()
    sim.signals.PSCD.zPosAct = single(zeros(length(sim.signals.PSCD.Time),1)); % Actual z-Position, obtained from INAV
    sim.signals.PSCD.zVelDes = single(zeros(length(sim.signals.PSCD.Time),1)); % Desired z-Velocity, calculated in set_pos_target_z_from_climb_rate_cm()
    sim.signals.PSCD.zVelAct = single(zeros(length(sim.signals.PSCD.Time),1)); % Actual z-Velocity, obtained from INAV
    sim.signals.PSCD.zVelTar = single(zeros(length(sim.signals.PSCD.Time),1)); % Target z-Velocity, calculated in update_z_controller()
    sim.signals.PSCD.zAccDes = single(zeros(length(sim.signals.PSCD.Time),1)); % Desired z-Acceleration, calculated in set_pos_target_z_from_climb_rate_cm()
    sim.signals.PSCD.zAccAct = single(zeros(length(sim.signals.PSCD.Time),1)); % Actual z-Acceleration, obtained from AHRS  
    sim.signals.PSCD.zAccTar = single(zeros(length(sim.signals.PSCD.Time),1)); % Target z-Acceleration, calculated in update_z_controller()
    
    % Signals of PIDA message
    sim.signals.PIDA.Time = single(sim.signals.RATE.Time)-tStart;
    % Inputs PID z acceleration
    sim.signals.PIDA.Err = single(zeros(length(sim.signals.PIDA.Time),1)); % Error between target and actual z-acceleration
    sim.signals.PIDA.ClockDMod = single(zeros(length(sim.signals.PIDA.Time),1)); % Clock of Slew Limiter

    % Outputs PID z acceleration
    sim.signals.PIDA.FF = single(zeros(length(sim.signals.PIDA.Time),1));
    sim.signals.PIDA.P = single(zeros(length(sim.signals.PIDA.Time),1));
    sim.signals.PIDA.I = single(zeros(length(sim.signals.PIDA.Time),1));
    sim.signals.PIDA.D = single(zeros(length(sim.signals.PIDA.Time),1));
    sim.signals.PIDA.DMod = single(zeros(length(sim.signals.PIDA.Time),1));
    sim.signals.PIDA.ILimit = single(zeros(length(sim.signals.PIDA.Time),1));
    sim.signals.PIDA.SRate = single(zeros(length(sim.signals.PIDA.Time),1)); 
    sim.signals.PIDA.Act = single(zeros(length(sim.signals.PIDA.Time),1));
    sim.signals.PIDA.Tar = single(zeros(length(sim.signals.PIDA.Time),1));
    
    % Initial inputs
    % Initial actual values
    sim.init.PSCD.posAct = single(0);
    sim.init.PSCD.velAct = single(0);
    sim.init.PSCD.accAct = single(0);
    
    sim.init.PIDA.posTar = single(0);
    sim.init.PIDA.velDes = single(0);
    sim.init.PIDA.accDes = single(0);
    
    sim.init.PIDA.P = single(0);
    sim.init.PIDA.I = single(0);
    sim.init.PIDA.D = single(0);
    sim.init.PIDA.TarFilt = single(0);
    sim.init.PIDA.ErrFilt = single(0);
    sim.init.PIDA.DerFilt = single(0);
    sim.init.PIDA.SROut = single(0);
end


% Real Flight Signals of PID z velocity - only load if log messages
% exists
if isfield(sid, 'PCVZ')
    iVec = (sid.PCVZ.TimeS >= tStart & sid.PCVZ.TimeS <= tEnd);
    sim.signals.PCVZ.Time = single(sid.PCVZ.TimeS(iVec)-tStart);
    sim.signals.PCVZ.Err = single(sid.PCVZ.Err(iVec));
    sim.signals.PCVZ.P = single(sid.PCVZ.P(iVec));
    sim.signals.PCVZ.I = single(sid.PCVZ.I(iVec));
    sim.signals.PCVZ.D = single(sid.PCVZ.D(iVec));
    sim.signals.PCVZ.FF = single(sid.PCVZ.FF(iVec));
else % Set to zero otherwise
    iVec = (sid.RATE.TimeS >= tStart & sid.RATE.TimeS <= tEnd);
    sim.signals.PCVZ.Time = single(sid.RATE.TimeS(iVec)-tStart);
    sim.signals.PCVZ.Err = single(zeros(length(sim.signals.PCVZ.Time),1));
    sim.signals.PCVZ.P = single(zeros(length(sim.signals.PCVZ.Time),1));
    sim.signals.PCVZ.I = single(zeros(length(sim.signals.PCVZ.Time),1));
    sim.signals.PCVZ.D = single(zeros(length(sim.signals.PCVZ.Time),1));
    sim.signals.PCVZ.FF = single(zeros(length(sim.signals.PCVZ.Time),1));
end

%% Load identified plant models data
if sim.param.optCtrl || sim.param.mdlVal && exist('idModel', 'var')
    % Roll
    if isempty(findobj(idModel, 'axis', 'RLL'))
        sim.models.RollAxis = idpoly(tf(1, [1 1], sim.param.dt));
    else
        model = findobj(idModel, 'axis', 'RLL');
        % If more than one model is found, let user choose
        if length(model) > 1 
            mdlIdx = input('More than one model found for the roll axis. Input index of desired model: ');
        else
            mdlIdx = 1;
        end
        sim.models.RollAxis = idss(c2d(model(mdlIdx).tfModel, sim.param.dt));
        % Decrease delay to account for the additional feedback delay
        sim.models.RollAxis.InputDelay = sim.models.RollAxis.InputDelay-1;
    end
    % Pitch
    if isempty(findobj(idModel, 'axis', 'PIT'))
        sim.models.PitchAxis = idpoly(tf(1, [1 1], sim.param.dt));
    else
        model = findobj(idModel, 'axis', 'PIT');
        % If more than one model is found, let user choose
        if length(model) > 1 
            mdlIdx = input('More than one model found for the pitch axis. Input index of desired model: ');
        else
            mdlIdx = 1;
        end
        sim.models.PitchAxis = idss(c2d(model(mdlIdx).tfModel, sim.param.dt));
        sim.models.PitchAxis.InputDelay = sim.models.PitchAxis.InputDelay-1;
    end
    % Yaw
    if isempty(findobj(idModel, 'axis', 'YAW'))
        sim.models.YawAxis = idpoly(tf(1, [1 1], sim.param.dt));
    else
        model = findobj(idModel, 'axis', 'YAW');
        % If more than one model is found, let user choose
        if length(model) > 1 
            mdlIdx = input('More than one model found for the yaw axis. Input index of desired model: ');
        else
            mdlIdx = 1;
        end
        sim.models.YawAxis = idss(c2d(model(mdlIdx).tfModel, sim.param.dt));
        sim.models.YawAxis.InputDelay = sim.models.YawAxis.InputDelay-1;
    end
    % Vertical Motion - Throttle
    if isempty(findobj(idModel, 'axis', 'THR'))
        sim.models.RollAxis = idpoly(tf(1, [1 1], sim.param.dt));
    else
        model = findobj(idModel, 'axis', 'THR');
        % If more than one model is found, let user choose
        if length(model) > 1
            mdlIdx = input('More than one model found for the vertical axis. Input index of desired model: ');
        else
            mdlIdx = 1;
        end
        sim.models.VerticalAxis = idss(c2d(model(mdlIdx).tfModel, sim.param.dt));
        % Decrease delay to account for the additional feedback delay
        sim.models.VerticalAxis.InputDelay = sim.models.VerticalAxis.InputDelay-1;
    end
else % Create dummy models of optimization is not active
    sim.models.RollAxis = idpoly(tf(1, [1 1], sim.param.dt));
    sim.models.PitchAxis = idpoly(tf(1, [1 1], sim.param.dt));
    sim.models.YawAxis = idpoly(tf(1, [1 1], sim.param.dt));
    sim.models.VerticalAxis = idpoly(tf(1, [1 1], sim.param.dt));
end

% Initial values
if sim.param.mdlVal && mode == 25
    sim.models.RollInit = single(sid.SIDD.Gx(1)*pi/180);
    sim.models.PitchInit = single(sid.SIDD.Gy(1)*pi/180);
    sim.models.YawInit = single(sid.SIDD.Gz(1)*pi/180);
    sim.models.VerticalOutInit = single(sid.SIDD.Az(1));
    sim.models.VerticalInInit = single(sid.RATE.AOut(1));
else
    sim.models.RollInit = single(0);
    sim.models.PitchInit = single(0);
    sim.models.YawInit = single(0);
    sim.models.VerticalOutInit = single(-9.81);
    sim.models.VerticalInInit = single(sid.RATE.AOut(1));
end

%% Delete all irrelevant data
clear i dist start thr_man rel_msg mode param_names msgs tEnd tStart simMode
clear phase_des runOutsideLoop mdlIdx pitchCh rollCh yawCh iVec Fs axis dt
