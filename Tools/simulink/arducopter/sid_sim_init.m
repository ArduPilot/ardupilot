% Initialization of the Simulink model. The script loads all important
% signals and parameter to the workspace.
%
% Fabian Bredemeier - IAV GmbH
% License: GPL v3
%% Define log index and start + stop time
if ~exist('log_idx', 'var')
    log_idx = 2;
end
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
if ~exist('simMode', 'var')
    simMode = 0;
end

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

% Sample rate of simInulation
Fs = 400;
dt = 0.0025;
simIn.param.dt = 0.0025;

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

% Define validation and optimization flag of simInulation
simIn.param.mdlVal = simMode == 0;
simIn.param.rateCtrlVal = simMode == 1;
simIn.param.attCtrlVal = simMode == 2 || simMode == 3;
simIn.param.altCtrlVal = simMode == 3 || mode == 2;
simIn.param.optCtrl = simMode == 4;

%% General Copter settings
simIn.param.LOOP_RATE = getParamVal(sid, 'SCHED_LOOP_RATE');
simIn.param.dt = round(double(1 / simIn.param.LOOP_RATE), 4); % Sample time resulting from loop rate. Round to four decimal digits
simIn.param.FRAME_CLASS = getParamVal(sid, 'FRAME_CLASS');
simIn.param.FRAME_TYPE = getParamVal(sid, 'FRAME_TYPE');
[simIn.param.NUM_MOTORS, simIn.param.AXIS_FAC_MOTORS] = getMotorMixerFactors(simIn.param.FRAME_CLASS, simIn.param.FRAME_TYPE);
simIn.param.EKF_TYPE = getParamVal(sid, 'AHRS_EKF_TYPE');
simIn.param.GRAVITY_MSS = single(9.80665);
simIn.param.dt_THR_LOOP = 1 / 50; % Sample time of throttle loop in Copter.cpp. Defined in Copter.cpp, line 94
simIn.param.ANGLE_MAX = getParamVal(sid, 'ANGLE_MAX'); % Maximum lean angle in all flight modes
simIn.param.PILOT_Y_EXPO = getParamVal(sid, 'PILOT_Y_EXPO'); % Acro yaw expo to allow faster rotation when stick at edges
simIn.param.PILOT_Y_RATE = getParamVal(sid, 'PILOT_Y_RATE');
simIn.param.PILOT_Y_RATE_TC = getParamVal(sid, 'PILOT_Y_RATE_TC'); % Pilot yaw rate control input time constant
simIn.param.MODE.ModeNr = mode; % Flight mode

% Abort script if frame configuration is not defined yet
if (isempty(simIn.param.AXIS_FAC_MOTORS) || simIn.param.NUM_MOTORS == 0)
    error("Body configuration of copter is not defined yet!");
end

%% Simulation duration
% Define end time of simInulation
if simIn.param.mdlVal || simIn.param.rateCtrlVal || simIn.param.attCtrlVal || simIn.param.altCtrlVal
    if isinf(tEnd) % Run full data
        simIn.param.timeEnd = sid.RATE.TimeS(end);
    else % Only run part of data
        duration = tEnd-tStart;
        iEnd = int16(duration/dt);
        simIn.param.timeEnd = sid.RATE.TimeS(iEnd);
        clear duration iEnd
    end
elseif simIn.param.optCtrl
    simIn.param.timeEnd = OPT.tEndOpt;
end

%% Inertial Sensor
simIn.param.INS.GYRO_FILTER = getParamVal(sid, 'INS_GYRO_FILTER'); % Filter cutoff frequency for gyro lowpass filter
simIn.param.INS.GYRO_RATE = 400; % Gyro sampling rate. Actually define by INS_GYRO_RATE, but simInulation is run with 400 Hz

%% AHRS
trim_x = getParamVal(sid, 'AHRS_TRIM_X');
trim_y = getParamVal(sid, 'AHRS_TRIM_Y');
simIn.param.AHRS.FC2BF = single(dcmFromEuler(trim_x, trim_y, 0)); % Rotation matrix from FC to Body Frame based on AP_AHRS constructor
simIn.param.AHRS.BF2FC = simIn.param.AHRS.FC2BF';

clear trim_x trim_y
%% RCIN - Radio Input and parameters for Attitude Control
rollCh = getParamVal(sid, 'RCMAP_ROLL');
pitchCh = getParamVal(sid, 'RCMAP_PITCH');
yawCh = getParamVal(sid, 'RCMAP_YAW');
thrCh = getParamVal(sid, 'RCMAP_THROTTLE');

simIn.param.RCIN.DZ_RLL = getParamVal(sid, ['RC' num2str(rollCh) '_DZ']);
simIn.param.RCIN.MIN_RLL = getParamVal(sid, ['RC' num2str(rollCh) '_MIN']);
simIn.param.RCIN.MAX_RLL = getParamVal(sid, ['RC' num2str(rollCh) '_MAX']);
simIn.param.RCIN.TRIM_RLL = getParamVal(sid, ['RC' num2str(rollCh) '_TRIM']);
simIn.param.RCIN.REVERSED_RLL = getParamVal(sid, ['RC' num2str(rollCh) '_REVERSED']);
simIn.param.RCIN.DZ_PIT = getParamVal(sid, ['RC' num2str(pitchCh) '_DZ']);
simIn.param.RCIN.MIN_PIT = getParamVal(sid, ['RC' num2str(pitchCh) '_MIN']);
simIn.param.RCIN.MAX_PIT = getParamVal(sid, ['RC' num2str(pitchCh) '_MAX']);
simIn.param.RCIN.TRIM_PIT = getParamVal(sid, ['RC' num2str(pitchCh) '_TRIM']);
simIn.param.RCIN.REVERSED_PIT = getParamVal(sid, ['RC' num2str(pitchCh) '_REVERSED']);
simIn.param.RCIN.DZ_YAW = getParamVal(sid, ['RC' num2str(yawCh) '_DZ']);
simIn.param.RCIN.MIN_YAW = getParamVal(sid, ['RC' num2str(yawCh) '_MIN']);
simIn.param.RCIN.MAX_YAW = getParamVal(sid, ['RC' num2str(yawCh) '_MAX']);
simIn.param.RCIN.TRIM_YAW = getParamVal(sid, ['RC' num2str(yawCh) '_TRIM']);
simIn.param.RCIN.REVERSED_YAW = getParamVal(sid, ['RC' num2str(yawCh) '_REVERSED']);
simIn.param.RCIN.DZ_THR = getParamVal(sid, ['RC' num2str(thrCh) '_DZ']);
simIn.param.RCIN.MIN_THR = getParamVal(sid, ['RC' num2str(thrCh) '_MIN']);
simIn.param.RCIN.MAX_THR = getParamVal(sid, ['RC' num2str(thrCh) '_MAX']);
simIn.param.RCIN.TRIM_THR = getParamVal(sid, ['RC' num2str(thrCh) '_TRIM']);
simIn.param.RCIN.REVERSED_THR = getParamVal(sid, ['RC' num2str(thrCh) '_REVERSED']);

simIn.param.RCIN.ROLL_PITCH_YAW_INPUT_MAX = 4500; % Yaw, roll, pitch input range, defined in config.h

% Check if RCIN log is available
if simIn.param.mdlVal || simIn.param.attCtrlVal || simIn.param.altCtrlVal
    if isfield(sid, 'RCIN')
        iVec = (sid.RCIN.TimeS >= tStart & sid.RCIN.TimeS <= tEnd);
        simIn.signals.RCIN.Time = single(sid.RCIN.TimeS(iVec)-tStart);
        simIn.signals.RCIN.RollInput = eval(['single(sid.RCIN.C' num2str(rollCh) '(iVec))']);
        simIn.signals.RCIN.PitchInput = eval(['single(sid.RCIN.C' num2str(pitchCh) '(iVec))']);
        simIn.signals.RCIN.YawInput = eval(['single(sid.RCIN.C' num2str(yawCh) '(iVec))']);
        simIn.signals.RCIN.ThrottleInput = eval(['single(sid.RCIN.C' num2str(thrCh) '(iVec))']);
    else
        if ~isfield(sid, 'RCIN') && axis == 13 
            warning(['RCIN message cannot be found in log ' num2str(log_idx) '! Validating vertical motion model.']);
        else
            warning(['RCIN message cannot be found in log ' num2str(log_idx) '! Ommiting attitude controller and models in simInulation. Only rate controller is validated']);
            simIn.param.rateCtrlVal = 1;
            simIn.param.mdlVal = 0;
        end
        % Set RCIN signals to zero
        simIn.signals.RCIN.Time = single(0:0.1:simIn.param.timeEnd);
        simIn.signals.RCIN.RollInput = single(zeros(length(simIn.signals.RCIN.Time),1));
        simIn.signals.RCIN.PitchInput = single(zeros(length(simIn.signals.RCIN.Time),1));
        simIn.signals.RCIN.YawInput = single(zeros(length(simIn.signals.RCIN.Time),1));
        simIn.signals.RCIN.ThrottleInput = single(zeros(length(simIn.signals.RCIN.Time),1));
    end
end

%% Attitude
% Yaw Angle signals are a modulo of 360°, so this operation has to be
% inverted
iVec = (sid.ATT.TimeS >= tStart & sid.ATT.TimeS <= tEnd);
simIn.signals.ATT.Time = single(sid.ATT.TimeS(iVec)-tStart);
simIn.signals.ATT.Roll = single(sid.ATT.Roll(iVec));
simIn.signals.ATT.Pitch = single(sid.ATT.Pitch(iVec));
simIn.signals.ATT.Yaw = single(demodYawAngle(sid.ATT.Yaw(iVec)));
simIn.signals.ATT.DesRoll = single(sid.ATT.DesRoll(iVec));
simIn.signals.ATT.DesPitch = single(sid.ATT.DesPitch(iVec));
simIn.signals.ATT.DesYaw = single(demodYawAngle(sid.ATT.DesYaw(iVec)));

% The actual yaw angle is calculated from the DCM with
% Matrix3<T>::to_euler() with the four-quadrant arcus tangens atan2(), for 
% example in AP_AHRS_View::update(), so the controller can only handle 
% values in the range [-pi, pi]. 
simIn.signals.ATT.Yaw = simIn.signals.ATT.Yaw - 360 * (simIn.signals.ATT.Yaw > 180);
simIn.signals.ATT.DesYaw = simIn.signals.ATT.DesYaw - 360 * (simIn.signals.ATT.DesYaw > 180);

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
yawAngInit = single(simIn.signals.ATT.Yaw(1))*pi/180;
simIn.init.ATT.Roll = rollAngInit;
simIn.init.ATT.Pitch = pitchAngInit;
simIn.init.ATT.Yaw = yawAngInit;
simIn.init.ATT.DCM = dcmFromEuler(rollAngInit, pitchAngInit, yawAngInit);

% Initialize attitudeTarget according to
% AC_AttitudeControl::reset_yaw_target_and_rate(), which is called in
% ModeStabilize::run() when copter is landed
attitudeTargetUpdate = fromAxisAngle([0;0;yawAngInit]);
simIn.init.ATT.attitudeTarget = quatMult(attitudeTargetUpdate, [1;0;0;0]);
simIn.init.ATT.eulerRateTar = [0;0;0];

% Test signals in SITL
% Attitude Controller Targets
% simIn.signals.ATT.attTarMeasW = single(sid.ATAR.attW);
% simIn.signals.ATT.attTarMeasX = single(sid.ATAR.attX);
% simIn.signals.ATT.attTarMeasY = single(sid.ATAR.attY);
% simIn.signals.ATT.attTarMeasZ = single(sid.ATAR.attZ);
% simIn.signals.ATT.eulerAngTarMeasX = single(sid.ATAR.angX);
% simIn.signals.ATT.eulerAngTarMeasY = single(sid.ATAR.angY);
% simIn.signals.ATT.eulerAngTarMeasZ = single(sid.ATAR.angZ);
% simIn.signals.ATT.eulerRatTarMeasX = single(sid.ATAR.ratX);
% simIn.signals.ATT.eulerRatTarMeasY = single(sid.ATAR.ratY);
% simIn.signals.ATT.eulerRatTarMeasZ = single(sid.ATAR.ratZ);
% 
% simIn.signals.ATT.CtrlRollIn = single(sid.CTIN.rllCtrl);
% simIn.signals.ATT.CtrlPitchIn = single(sid.CTIN.pitCtrl);
% simIn.signals.ATT.CtrlYawIn = single(sid.CTIN.yawCtrl);
% simIn.signals.ATT.yawRateDesMeas = single(sid.ATIN.yawDes);

clear rollAngInit pitchAngInit yawAngInit attitudeTargetUpdate iInit

%% Attitude Controller General
simIn.param.ATC.RATE_FF_ENAB = getParamVal(sid, 'ATC_RATE_FF_ENAB');
simIn.param.ATC.INPUT_TC = getParamVal(sid, 'ATC_INPUT_TC'); % Attitude control input time constant
simIn.param.ATC.ANGLE_LIMIT_MIN = single(10); % Min lean angle so that vehicle can maintain limited control, defined in AC_AttitudeControl.cpp as AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MIN
simIn.param.ATC.ANGLE_LIMIT_THROTTLE_MAX = single(0.8); % Max throttle used to limit lean angle so that vehicle does not lose altitude, defined in AC_AttitudeControl.h as AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX
simIn.param.ATC.ANG_LIM_TC = getParamVal(sid, 'ATC_ANG_LIM_TC'); % Angle Limit (to maintain altitude) Time Constant
% Roll Angle Controller
simIn.param.ATC.ANG_RLL_P = getParamVal(sid, 'ATC_ANG_RLL_P');
simIn.param.ATC.ACCEL_R_MAX = getParamVal(sid, 'ATC_ACCEL_R_MAX');
simIn.param.ATC.RATE_R_MAX = getParamVal(sid, 'ATC_RATE_R_MAX');
simIn.param.ATC.ACCEL_RP_CONTROLLER_MIN_RADSS = 40*pi/180; % Maximum body-frame acceleration limit for the stability controller, defined in AC_AttitudeControl.h
simIn.param.ATC.ACCEL_RP_CONTROLLER_MAX_RADSS = 720*pi/180;
% Pitch Angle Controller
simIn.param.ATC.ANG_PIT_P = getParamVal(sid, 'ATC_ANG_PIT_P');
simIn.param.ATC.ACCEL_P_MAX = getParamVal(sid, 'ATC_ACCEL_P_MAX');
simIn.param.ATC.RATE_P_MAX = getParamVal(sid, 'ATC_RATE_P_MAX');
% Yaw Angle Controller
simIn.param.ATC.ANG_YAW_P = getParamVal(sid, 'ATC_ANG_YAW_P');
simIn.param.ATC.ACCEL_Y_MAX = getParamVal(sid, 'ATC_ACCEL_Y_MAX');
simIn.param.ATC.RATE_Y_MAX = getParamVal(sid, 'ATC_RATE_Y_MAX');
simIn.param.ATC.ACCEL_Y_CONTROLLER_MAX_RADSS = single(120*pi/180); % Maximum body-frame acceleration limit for the stability controller, defined in AC_AttitudeControl.h
simIn.param.ATC.ACCEL_Y_CONTROLLER_MIN_RADSS = single(10*pi/180);
simIn.param.ATC.THRUST_ERROR_ANGLE = single(30*pi/180); % Thrust angle error above which yaw corrections are limited. Defined in AC_AttitudeControl.h

% Thrust and throttle calculation parameters
simIn.param.MODE.THR_MAN_FLG = thr_man;    % Flag for manual throttle, which depends on the current mode. 
if thr_man
    simIn.param.ATC.THR_FILT_CUTOFF = getParamVal(sid, 'PILOT_THR_FILT'); % Cutoff frequency of throttle filter for modes with manual throttle. parameter is given to function call of AC_AttitudeControl_Multi::set_throttle_out().
else
    simIn.param.ATC.THR_FILT_CUTOFF = 2.0; % Cutoff frequency of throttle filter for modes with z position control (Defined in AC_PosControl.h, line 33)
end
simIn.param.ATC.ANGLE_BOOST = getParamVal(sid, 'ATC_ANGLE_BOOST');   % Enabling of angle boost to increase output throttle. Used in AC_AttitudeConrtol_Multi::get_throttle_boosted()
simIn.param.ATC.THR_MIX_MAN = getParamVal(sid, 'ATC_THR_MIX_MAN');   % Throttle vs. attitude priorisation during manual flight
simIn.param.ATC.THR_MIX_MIN = getParamVal(sid, 'ATC_THR_MIX_MIN');   % Throttle vs. attitude priorisation used when landing
simIn.param.ATC.THR_MIX_MAX = getParamVal(sid, 'ATC_THR_MIX_MAX');   % Throttle vs. attitude priorisation during active flights
simIn.param.ATC.THR_MIX_DFLT = single(0.5); % Default value for the priorization of throttle use between attitude control and throttle demand by pilot (or autopilot). Defined in AC_AttitudeControl.h, line 44 
if thr_man
    simIn.init.ATC.THR_MIX = simIn.param.ATC.THR_MIX_MAN;
else
    simIn.init.ATC.THR_MIX = simIn.param.ATC.THR_MIX_DFLT;
end
simIn.param.MOT.HOVER_LEARN = getParamVal(sid, 'MOT_HOVER_LEARN');     % Enable/Disable automatic learning of hover throttle (0=Disabled, 1=Learn, 2=Learn and Save
simIn.param.MOT.THST_HOVER = getParamVal(sid, 'MOT_THST_HOVER');    % Motor thrust needed to hover. Default value of _throttle_hover.
simIn.param.MOT.THST_HOVER = simIn.param.MOT.THST_HOVER(1); % Assign first value of array to default value which is the correct value in the case, that the parameter is defined twice in the param file.
simIn.param.MOT.THST_HOVER_TC = single(10.0);                                         % Time constant used to update estimated hover throttle, found as AP_MOTORS_THST_HOVER_TC in AP_MotorsMulticopter.h
simIn.param.MOT.THST_HOVER_MIN = single(0.125);                                       % Minimum possible hover throttle, found as AP_MOTORS_THST_HOVER_MIN in AP_MotorsMulticopter.h
simIn.param.MOT.THST_HOVER_MAX = single(0.6875);                                      % Maximum possible hover throttle, found as AP_MOTORS_THST_HOVER_MAX in AP_MotorsMulticopter.h
simIn.param.MOT.BAT_VOLT_MAX = getParamVal(sid, 'MOT_BAT_VOLT_MAX');   % Maximum voltage above which no additional scaling on thrust is performed
simIn.param.MOT.BAT_VOLT_MIN = getParamVal(sid, 'MOT_BAT_VOLT_MIN');   % Minimum voltage below which no additional scaling on thrust is performed
simIn.param.MOT.BAT_CURR_MAX = getParamVal(sid, 'MOT_BAT_CURR_MAX');   % Maximum current over which maximum throttle is limited (and no further scaling is performed)
simIn.param.MOT.THST_EXPO = getParamVal(sid, 'MOT_THST_EXPO');             % Motor thrust curve exponent (0.0 for linear to 1.0 for second order curve)
simIn.param.MOT.BAT_CURR_TC = getParamVal(sid, 'MOT_BAT_CURR_TC');               % Time constant used to limit the maximum current
simIn.param.MOT.YAW_HEADROOM = getParamVal(sid, 'MOT_YAW_HEADROOM');             % Yaw control is goven at least this PWM in microseconds range

% Throttle inputs
iVec = (sid.CTUN.TimeS >= tStart & sid.CTUN.TimeS <= tEnd);
simIn.signals.CTUN.Time = single(sid.CTUN.TimeS(iVec)--tStart);
simIn.signals.CTUN.ThrIn = single(sid.CTUN.ThI(iVec)); % Throttle In, pilots input to Attitude Control (attitude_control->get_throttle_in())
simIn.signals.CTUN.ThrOut = single(sid.CTUN.ThO(iVec)); % Throttle Out, throttle given to motor mixer after filtering (motors->get_throttle())
simIn.signals.CTUN.AngBst = single(sid.CTUN.ABst(iVec)); % Extra amount of throttle, added to throttle_in in AC_AttitudeControl_Multi::get_throttle_boosted()
simIn.signals.CTUN.ThrHov = single(sid.CTUN.ThH(iVec)); % Estimated throttle required to hover throttle in range 0-1 (AP_MotorsMulticopter::get_throttle_hover())

% Throttle inital values
if simIn.param.rateCtrlVal || simIn.param.attCtrlVal || simIn.param.mdlVal || simIn.param.altCtrlVal
    simIn.init.CTUN.ThrOut = single(sid.CTUN.ThO(1));
    simIn.init.CTUN.ThrHov = single(sid.CTUN.ThH(1));
else
    simIn.init.CTUN.ThrOut = single(0);
    simIn.init.CTUN.ThrHov = single(simIn.param.MOT.THST_HOVER);
end

% Altitude signals
simIn.signals.CTUN.DSAlt = single(sid.CTUN.DSAlt(iVec)); % Desired rangefinder altitude
simIn.signals.CTUN.DAlt = single(sid.CTUN.DAlt(iVec)); % Desired altitude
simIn.signals.CTUN.Alt = single(sid.CTUN.Alt(iVec)); % Achieved altitude (inav)
simIn.signals.CTUN.SAlt = single(sid.CTUN.SAlt(iVec)); % Achieved rangefinder altitude
simIn.signals.CTUN.TAlt = single(sid.CTUN.TAlt(iVec)); % Terrain altitude
simIn.signals.CTUN.BAlt = single(sid.CTUN.BAlt(iVec)); % Barometric altitude
simIn.signals.CTUN.CRt = single(sid.CTUN.CRt(iVec)); % Climb rate inav
simIn.signals.CTUN.DCRt = single(sid.CTUN.DCRt(iVec)); % Desired climb rate 

% Altitude inital values
simIn.init.CTUN.Alt = single(sid.CTUN.Alt(1));
simIn.init.CTUN.CRt = single(sid.CTUN.CRt(1)*0.01);

% Intermediate throttle calculation quantities - only read if message
% exists
if isfield(sid, 'MOTQ')
    iVec = (sid.MOTQ.TimeS >= tStart & sid.MOTQ.TimeS <= tEnd);
    simIn.signals.MOTQ.Time = single(sid.MOTQ.TimeS(iVec)-tStart);
    simIn.signals.MOTQ.ThrAvgMax = single(sid.MOTQ.ThAvgMax(iVec));
    simIn.signals.MOTQ.ThrThstMaxMeas = single(sid.MOTQ.ThThstMax(iVec));
    simIn.signals.MOTQ.CompGain = single(sid.MOTQ.CompGain(iVec));
    simIn.signals.MOTQ.AirDensityRatio = single(sid.MOTQ.AirDensRat(iVec));
    simIn.signals.MOTQ.ThrMixOut = single(sid.MOTQ.ThrOut(iVec));
else
    iVec = (sid.CTUN.TimeS >= tStart & sid.CTUN.TimeS <= tEnd);
    simIn.signals.MOTQ.Time = single(sid.CTUN.TimeS(iVec)-tStart);
    simIn.signals.MOTQ.ThrAvgMax = single(zeros(length(simIn.signals.MOTQ.Time),1));
    simIn.signals.MOTQ.ThrThstMaxMeas = single(zeros(length(simIn.signals.MOTQ.Time),1));
    simIn.signals.MOTQ.CompGain = single(zeros(length(simIn.signals.MOTQ.Time),1));
    simIn.signals.MOTQ.AirDensityRatio = single(zeros(length(simIn.signals.MOTQ.Time),1));
    simIn.signals.MOTQ.ThrMixOut = single(zeros(length(simIn.signals.MOTQ.Time),1));
end


% Battery inputs
iVec = (sid.BAT.TimeS >= tStart & sid.BAT.TimeS <= tEnd);
simIn.signals.BAT.Time = single(sid.BAT.TimeS(iVec)-tStart);
simIn.signals.BAT.RestVoltEst = single(sid.BAT.VoltR(iVec)); % Estimated resting voltage of the battery
simIn.signals.BAT.Curr = single(sid.BAT.Curr(iVec)); % Measured battery current
simIn.signals.BAT.Res = single(sid.BAT.Res(iVec)); % Estimated battery resistance
simIn.signals.BAT.Volt = single(sid.BAT.Volt(iVec)); % Measured voltage resistance

% Battery inital values
if simIn.param.rateCtrlVal || simIn.param.attCtrlVal || simIn.param.mdlVal || simIn.param.altCtrlVal
    simIn.init.BAT.RestVoltEst = single(sid.BAT.VoltR(1));
else
    simIn.init.BAT.RestVoltEst = single(0);
end

% Baro inputs
iVec = (sid.BARO.TimeS >= tStart & sid.BARO.TimeS <= tEnd);
simIn.signals.BARO.Time = single(sid.BARO.TimeS(iVec)-tStart);
simIn.signals.BARO.Alt = single(sid.BARO.Alt(iVec));     % Calculated altitude by the barometer
simIn.signals.BARO.AirPress = single(sid.BARO.Press(iVec));  % Measured atmospheric pressure by the barometer
simIn.signals.BARO.GndTemp = single(sid.BARO.GndTemp(iVec)); % Temperature on ground, specified by parameter or measured while on ground

% Attitude controller outputs
iVec = (sid.MOTB.TimeS >= tStart & sid.MOTB.TimeS <= tEnd);
simIn.signals.MOTB.Time = single(sid.MOTB.TimeS(iVec)-tStart);
simIn.signals.MOTB.LiftMax = single(sid.MOTB.LiftMax(iVec)); % Maximum motor compensation gain, calculated in AP_MotorsMulticopter::update_lift_max_from_batt_voltage()
simIn.signals.MOTB.ThrLimit = single(sid.MOTB.ThLimit(iVec)); % Throttle limit set due to battery current limitations, calculated in AP_MotorsMulticopter::get_current_limit_max_throttle()
simIn.signals.MOTB.ThrAvMx = single(sid.MOTB.ThrAvMx(iVec)); % Throttle average max

% SID Inputs
% Create array for sid signals, each element containing the signal struct
if (mode == 25)
    iVec = (sid.SIDD.TimeS >= tStart & sid.SIDD.TimeS <= tEnd);
    for i=1:13
        simIn.signals.SIDD.Time{i} = single(sid.SIDD.TimeS(iVec)-tStart);
        if (i == axis)
            simIn.signals.SIDD.Chirp{i} = single(sid.SIDD.Targ(iVec)); % SID test signal
        else
            simIn.signals.SIDD.Chirp{i} = single(zeros(length(simIn.signals.SIDD.Time{i}),1));
        end
    end
    simIn.signals.SIDD.AccX = single(sid.SIDD.Ax(iVec));
    simIn.signals.SIDD.AccY = single(sid.SIDD.Ay(iVec));
    simIn.signals.SIDD.AccZ = single(sid.SIDD.Az(iVec));
    simIn.signals.SIDD.GyrX = single(sid.SIDD.Gx(iVec));
    simIn.signals.SIDD.GyrY = single(sid.SIDD.Gy(iVec));
    simIn.signals.SIDD.GyrZ = single(sid.SIDD.Gz(iVec));
else
    iVec = (sid.RATE.TimeS >= tStart & sid.RATE.TimeS <= tEnd);
    for i=1:13
        simIn.signals.SIDD.Time{i} = single(sid.RATE.TimeS(iVec)-tStart);
        simIn.signals.SIDD.Chirp{i} = single(zeros(length(simIn.signals.SIDD.Time{i}),1));
    end
    simIn.signals.SIDD.AccX = single(zeros(length(simIn.signals.SIDD.Time{1}),1));
    simIn.signals.SIDD.AccY = single(zeros(length(simIn.signals.SIDD.Time{1}),1));
    simIn.signals.SIDD.AccZ = single(zeros(length(simIn.signals.SIDD.Time{1}),1));
    simIn.signals.SIDD.GyrX = single(zeros(length(simIn.signals.SIDD.Time{1}),1));
    simIn.signals.SIDD.GyrY = single(zeros(length(simIn.signals.SIDD.Time{1}),1));
    simIn.signals.SIDD.GyrZ = single(zeros(length(simIn.signals.SIDD.Time{1}),1));
end

%% Roll Rate Controller
iVec = (sid.PIDR.TimeS >= tStart & sid.PIDR.TimeS <= tEnd);
simIn.signals.PIDR.Time = single(sid.PIDR.TimeS(iVec)-tStart);
% Inputs
simIn.signals.PIDR.Tar = single(sid.PIDR.Tar(iVec)); % target values filtered
simIn.signals.PIDR.Act = single(sid.PIDR.Act(iVec)); % actual values
simIn.signals.PIDR.Err = single(sid.PIDR.Err(iVec)); % error values
if simIn.param.rateCtrlVal || simIn.param.attCtrlVal || simIn.param.mdlVal
    % Clock of Slew Limiter in ms
    % Use tracked logging time of PID message in validation modes
    simIn.signals.PIDR.ClockDMod = single(1000*simIn.signals.PIDR.Time); 
else
    simIn.signals.PIDR.ClockDMod = single(1000 * simIn.param.dt * (0:length(simIn.signals.PIDR.Time)-1)');
end
    
% Outputs
simIn.signals.PIDR.FF = single(sid.PIDR.FF(iVec));
simIn.signals.PIDR.P = single(sid.PIDR.P(iVec));
simIn.signals.PIDR.I = single(sid.PIDR.I(iVec));
simIn.signals.PIDR.D = single(sid.PIDR.D(iVec));
simIn.signals.PIDR.DMod = single(sid.PIDR.Dmod(iVec));
simIn.signals.PIDR.ILimit = single(sid.PIDR.Limit(iVec));
simIn.signals.PIDR.SRate = single(sid.PIDR.SRate(iVec)); % Output slew rate of the slew limiter, stored in _output_slew_rate in AC_PID.cpp, line 161

% parameters - Read from PARM.Value cell array with logical indexing
simIn.param.PIDR.TC = getParamVal(sid, 'ATC_INPUT_TC');
simIn.param.PIDR.FLTT_f = getParamVal(sid, 'ATC_RAT_RLL_FLTT');
simIn.param.PIDR.FLTE_f = getParamVal(sid, 'ATC_RAT_RLL_FLTE');
simIn.param.PIDR.FLTD_f = getParamVal(sid, 'ATC_RAT_RLL_FLTD');
simIn.param.PIDR.P = getParamVal(sid, 'ATC_RAT_RLL_P');
simIn.param.PIDR.I = getParamVal(sid, 'ATC_RAT_RLL_I');
simIn.param.PIDR.D = getParamVal(sid, 'ATC_RAT_RLL_D');
simIn.param.PIDR.IMAX = getParamVal(sid, 'ATC_RAT_RLL_IMAX');
simIn.param.PIDR.FF = getParamVal(sid, 'ATC_RAT_RLL_FF'); %0.05;
simIn.param.PIDR.SR_MAX = getParamVal(sid, 'ATC_RAT_RLL_SMAX'); %5.0;
simIn.param.PIDR.SR_TAU = single(1.0); % Slew Rate Tau - not yet available as a parameter of the copter. Set to 1.0 by default (AC_PID.h, line 24).
simIn.param.PIDR.SR_FLT_f = single(25.0);  % Slew Rate lowpass filter cutoff frequency. Defined in SlewLimiter.h, line 14.

% Inital inputs
if simIn.param.rateCtrlVal || simIn.param.attCtrlVal || simIn.param.mdlVal
    simIn.init.PIDR.P = single(sid.PIDR.P(1));
    simIn.init.PIDR.I = single(sid.PIDR.I(1));
    simIn.init.PIDR.D = single(sid.PIDR.D(1));
    simIn.init.PIDR.Tar = single(sid.RATE.RDes(1)*pi/180); % Convert to radian due to conversion to degree for logging (AP_AHRS_View::Write_Rate)
    simIn.init.PIDR.TarFilt = single(sid.PIDR.Tar(1));
    simIn.init.PIDR.ErrFilt = single(sid.PIDR.Err(1));
    simIn.init.PIDR.SROut = single(sid.PIDR.SRate(1)); % Initial value of the slew rate determined by the slew limiter. Used for both modifier_slew_rate and output_slew_rate.
    if (sid.PIDR.D(1) ~= 0) % Prevent division by zero
        simIn.init.PIDR.DerFilt = single(sid.PIDR.D(1))/simIn.param.PIDR.D;
    else
        simIn.init.PIDR.DerFilt = single(0);
    end
else % Set to zero if we do not want to validate model with test run
    simIn.init.PIDR.P = single(0);
    simIn.init.PIDR.I = single(0);
    simIn.init.PIDR.D = single(0);
    simIn.init.PIDR.Tar = single(0);
    simIn.init.PIDR.TarFilt = single(0);
    simIn.init.PIDR.ErrFilt = single(0);
    simIn.init.PIDR.DerFilt = single(0);
    simIn.init.PIDR.SROut = single(0);
end

%% Pitch Rate Controller
iVec = (sid.PIDP.TimeS >= tStart & sid.PIDP.TimeS <= tEnd);
simIn.signals.PIDP.Time = single(sid.PIDP.TimeS(iVec)-tStart);
% Inputs
simIn.signals.PIDP.Tar = single(sid.PIDP.Tar(iVec)); % target values filtered
simIn.signals.PIDP.Act = single(sid.PIDP.Act(iVec)); % actual values
simIn.signals.PIDP.Err = single(sid.PIDP.Err(iVec)); % error values                                          
if simIn.param.rateCtrlVal || simIn.param.attCtrlVal || simIn.param.mdlVal
    % Clock of Slew Limiter
    % Use tracked logging time of PID message in validation modes
    simIn.signals.PIDP.ClockDMod = single(1000 * simIn.signals.PIDP.Time); 
else
    simIn.signals.PIDP.ClockDMod = single(1000 * simIn.param.dt * (0:length(simIn.signals.PIDP.Time)-1)');
end

% Outputs
simIn.signals.PIDP.FF = single(sid.PIDP.FF(iVec));
simIn.signals.PIDP.P = single(sid.PIDP.P(iVec));
simIn.signals.PIDP.I = single(sid.PIDP.I(iVec));
simIn.signals.PIDP.D = single(sid.PIDP.D(iVec));
simIn.signals.PIDP.DMod = single(sid.PIDP.Dmod(iVec));
simIn.signals.PIDP.ILimit = single(sid.PIDP.Limit(iVec));
simIn.signals.PIDP.SRate = single(sid.PIDP.SRate(iVec)); % Output slew rate of the slew limiter, stored in _output_slew_rate in AC_PID.cpp, line 161

% Parameters - Read from PARM.Value cell array with logical indexing
simIn.param.PIDP.TC = getParamVal(sid, 'ATC_INPUT_TC');
simIn.param.PIDP.FLTT_f = getParamVal(sid, 'ATC_RAT_PIT_FLTT');
simIn.param.PIDP.FLTE_f = getParamVal(sid, 'ATC_RAT_PIT_FLTE');
simIn.param.PIDP.FLTD_f = getParamVal(sid, 'ATC_RAT_PIT_FLTD');
simIn.param.PIDP.P = getParamVal(sid, 'ATC_RAT_PIT_P');
simIn.param.PIDP.I = getParamVal(sid, 'ATC_RAT_PIT_I');
simIn.param.PIDP.D = getParamVal(sid, 'ATC_RAT_PIT_D');
simIn.param.PIDP.IMAX = getParamVal(sid, 'ATC_RAT_PIT_IMAX');
simIn.param.PIDP.FF = getParamVal(sid, 'ATC_RAT_PIT_FF');
simIn.param.PIDP.SR_MAX = getParamVal(sid, 'ATC_RAT_PIT_SMAX');
simIn.param.PIDP.SR_TAU = single(1.0); % Slew Rate Tau - not yet available as a parameter of the copter. Set to 1.0 by default (AC_PID.h, line 24).
simIn.param.PIDP.SR_FLT_f = single(25.0);  % Slew Rate lowpass filter cutoff frequency. Defined in SlewLimiter.h, line 14.

% Inital inputs
if simIn.param.rateCtrlVal || simIn.param.attCtrlVal || simIn.param.mdlVal
    simIn.init.PIDP.P = single(sid.PIDP.P(1));
    simIn.init.PIDP.I = single(sid.PIDP.I(1));
    simIn.init.PIDP.D = single(sid.PIDP.D(1));
    simIn.init.PIDP.Tar = single(sid.RATE.PDes(1)*pi/180); % Convert to radian due to conversion to degree for logging (AP_AHRS_View::Write_Rate)
    simIn.init.PIDP.TarFilt = single(sid.PIDP.Tar(1));
    simIn.init.PIDP.ErrFilt = single(sid.PIDP.Err(1));
    simIn.init.PIDP.SrOut = single(sid.PIDP.SRate(1)); % Initial value of the slew rate determined by the slew limiter. Used for both modifier_slew_rate and output_slew_rate.
    if (simIn.param.PIDP.D ~= 0)
        simIn.init.PIDP.DerFilt = single(sid.PIDP.D(1))/simIn.param.PIDP.D;
    else
        simIn.init.PIDP.DerFilt = single(0);
    end
else
    simIn.init.PIDP.P = single(0);
    simIn.init.PIDP.I = single(0);
    simIn.init.PIDP.D = single(0);
    simIn.init.PIDP.Tar = single(0);
    simIn.init.PIDP.TarFilt = single(0);
    simIn.init.PIDP.ErrFilt = single(0);
    simIn.init.PIDP.DerFilt = single(0);
    simIn.init.PIDP.SrOut = single(0);
end

%% Yaw Rate Controller
iVec = (sid.PIDY.TimeS >= tStart & sid.PIDY.TimeS <= tEnd);
simIn.signals.PIDY.Time = single(sid.PIDY.TimeS(iVec)-tStart);
% Inputs
simIn.signals.PIDY.Tar = single(sid.PIDY.Tar(iVec)); % target values filtered
simIn.signals.PIDY.Act = single(sid.PIDY.Act(iVec)); % actual values
simIn.signals.PIDY.Err = single(sid.PIDY.Err(iVec)); % error values                                          
if simIn.param.rateCtrlVal || simIn.param.attCtrlVal || simIn.param.mdlVal
    % Clock of Slew Limiter
    % Use tracked logging time of PID message in validation modes
    simIn.signals.PIDY.ClockDMod = single(1000 * simIn.signals.PIDY.Time); 
else
    simIn.signals.PIDY.ClockDMod = single(1000 * simIn.param.dt * (0:length(simIn.signals.PIDY.Time)-1)');
end

% Outputs
simIn.signals.PIDY.FF = single(sid.PIDY.FF(iVec));
simIn.signals.PIDY.P = single(sid.PIDY.P(iVec));
simIn.signals.PIDY.I = single(sid.PIDY.I(iVec));
simIn.signals.PIDY.D = single(sid.PIDY.D(iVec));
simIn.signals.PIDY.DMod = single(sid.PIDY.Dmod(iVec));
simIn.signals.PIDY.ILimit = single(sid.PIDY.Limit(iVec));
simIn.signals.PIDY.SRate = single(sid.PIDY.SRate(iVec)); % Output slew rate of the slew limiter, stored in _output_slew_rate in AC_PID.cpp, line 161

% Parameters - Read from PARM.Value cell array with logical indexing
simIn.param.PIDY.TC = getParamVal(sid, 'ATC_INPUT_TC');
simIn.param.PIDY.FLTT_f = getParamVal(sid, 'ATC_RAT_YAW_FLTT');
simIn.param.PIDY.FLTE_f = getParamVal(sid, 'ATC_RAT_YAW_FLTE');
simIn.param.PIDY.FLTD_f = getParamVal(sid, 'ATC_RAT_YAW_FLTD');
simIn.param.PIDY.P = getParamVal(sid, 'ATC_RAT_YAW_P');
simIn.param.PIDY.I = getParamVal(sid, 'ATC_RAT_YAW_I');
simIn.param.PIDY.D = getParamVal(sid, 'ATC_RAT_YAW_D');
simIn.param.PIDY.IMAX = getParamVal(sid, 'ATC_RAT_YAW_IMAX');
simIn.param.PIDY.FF = getParamVal(sid, 'ATC_RAT_YAW_FF');
simIn.param.PIDY.SR_MAX = getParamVal(sid, 'ATC_RAT_YAW_SMAX');
simIn.param.PIDY.SR_TAU = single(1.0); % Slew Rate Tau - not yet available as a parameter of the copter. Set to 1.0 by default (AC_PID.h, line 24).
simIn.param.PIDY.SR_FLT_f = single(25.0);  % Slew Rate lowpass filter cutoff frequency. Defined in SlewLimiter.h, line 14.

% Inital inputs
if simIn.param.rateCtrlVal || simIn.param.attCtrlVal || simIn.param.mdlVal
    simIn.init.PIDY.P = single(sid.PIDY.P(1));
    simIn.init.PIDY.I = single(sid.PIDY.I(1));
    simIn.init.PIDY.D = single(sid.PIDY.D(1));
    simIn.init.PIDY.Tar = single(sid.RATE.PDes(1)*pi/180); % Convert to radian due to conversion to degree for logging (AP_AHRS_View::Write_Rate)
    simIn.init.PIDY.TarFilt = single(sid.PIDY.Tar(1));
    simIn.init.PIDY.ErrFilt = single(sid.PIDY.Err(1));
    simIn.init.PIDY.SrOut = single(sid.PIDY.SRate(1)); % Initial value of the slew rate determined by the slew limiter. Used for both modifier_slew_rate and output_slew_rate.
    if (simIn.param.PIDY.D ~= 0)
        simIn.init.PIDY.DerFilt = single(sid.PIDY.D(1))/simIn.param.PIDY.D;
    else
        simIn.init.PIDY.DerFilt = single(0);
    end
else
    simIn.init.PIDY.P = single(0);
    simIn.init.PIDY.I = single(0);
    simIn.init.PIDY.D = single(0);
    simIn.init.PIDY.Tar = single(0);
    simIn.init.PIDY.TarFilt = single(0);
    simIn.init.PIDY.ErrFilt = single(0);
    simIn.init.PIDY.DerFilt = single(0);
    simIn.init.PIDY.SrOut = single(0);
end

%% RATE message
iVec = (sid.RATE.TimeS >= tStart & sid.RATE.TimeS <= tEnd);
simIn.signals.RATE.Time = single(sid.RATE.TimeS(iVec)-tStart);

% Roll
simIn.signals.RATE.ROut = single(sid.RATE.ROut(iVec)); % Rate Controller total output (except FF)
% Rate Controller target (deg/s): Logged signal in val phase, step in opt phase.
if simIn.param.rateCtrlVal || simIn.param.attCtrlVal || simIn.param.mdlVal
    simIn.signals.RATE.RDes = single(sid.RATE.RDes(iVec));
elseif simIn.param.optCtrl && axis == 10
    simIn.signals.RATE.RDes = single((simIn.signals.RATE.Time > OPT.stepTime)'*OPT.stepMag); % Step at 5s
else
    simIn.signals.RATE.RDes = single(zeros(numel(simIn.signals.RATE.Time), 1)); % Set to zero if axis not optimized
end

% Pitch
simIn.signals.RATE.POut = single(sid.RATE.POut(iVec));
% Rate Controller target (deg/s): Logged signal in val phase, step in opt phase
if simIn.param.rateCtrlVal || simIn.param.attCtrlVal || simIn.param.mdlVal
    simIn.signals.RATE.PDes = single(sid.RATE.PDes(iVec));
elseif simIn.param.optCtrl && axis == 11
    simIn.signals.RATE.PDes = single((simIn.signals.RATE.Time > OPT.stepTime)'*OPT.stepMag); % Step at 5s
else
    simIn.signals.RATE.PDes = single(zeros(numel(simIn.signals.RATE.Time), 1)); % Set to zero if axis not optimized
end

% Yaw 
simIn.signals.RATE.YOut = single(sid.RATE.YOut(iVec));
% Rate Controller target (deg/s): Logged signal in val phase, step in opt phase
if simIn.param.rateCtrlVal || simIn.param.attCtrlVal || simIn.param.mdlVal
    simIn.signals.RATE.YDes = single(sid.RATE.YDes(iVec));
elseif simIn.param.optCtrl && axis == 12
    simIn.signals.RATE.YDes = single((simIn.signals.RATE.Time > OPT.stepTime)'*OPT.stepMag); % Step at 5s
else
    simIn.signals.RATE.YDes = single(zeros(numel(simIn.signals.RATE.Time), 1)); % Set to zero if axis not optimized
end

% Throttle
simIn.signals.RATE.AOut = single(sid.RATE.AOut(iVec)); % Throttle command to motors
simIn.signals.RATE.A = single(sid.RATE.A(iVec)); % Vertical acceleration in earth frame

%% Position Controller
% Default definitions
simIn.param.PSCD.THR_DZ = getParamVal(sid, 'THR_DZ'); % Deadzone above and below mid throttle in PWM microseconds. Used in Althold, Loiter, PosHold. Defined in Parameters.cpp
simIn.param.PSCD.VEL_MAX_DOWN_DFLT = single(-150); % Default descent rate in cm/s, defined in AC_PosControl.h, line 27
simIn.param.PSCD.VEL_MAX_UP_DFLT = single(250); % Default climb rate in cm/s, defined in AC_PosControl.h, line 28
simIn.param.PSCD.ACC_MAX_Z_DFLT = single(250); % Default vertical acceleration cm/s/s, defined in AC_PosControl.h, line 30
simIn.param.PSCD.JERK_MAX_Z_DFLT = single(500); % Default vertical jerk, defined as m/s/s/s in AC_PosControl.h, line 31. Converted to cm/s/s/s in AC_PosControl.cpp, line 318.

simIn.param.PSCD.VEL_MAX_DN = getParamVal(sid, 'PILOT_SPEED_DN'); % Maximum vertical descending velocity in cm/s, defined in parameters.cpp, line 906
simIn.param.PSCD.VEL_MAX_UP = getParamVal(sid, 'PILOT_SPEED_UP'); % Maximum vertical ascending velocity in cm/s, defined in parameters.cpp, line 223
simIn.param.PSCD.ACC_MAX_Z = getParamVal(sid, 'PILOT_ACCEL_Z'); % Maximum vertical acceleration used when pilot is controlling the altitude, parameters.cpp, line 232
simIn.param.PSCD.JERK_MAX_Z = getParamVal(sid, 'PSC_JERK_Z'); % Jerk limit of vertical kinematic path generation in m/s^3. Determines how quickly aircraft changes acceleration target
simIn.param.PSCD.OVERSPEED_GAIN_Z = single(2); % gain controlling rate at which z-axis speed is brought back within SPEED_UP and SPEED_DOWN range, defined in AC_PosControl.h 
%% Position Controller z

% Parameters z position controller
simIn.param.PSCD.P_POS = getParamVal(sid, 'PSC_POSZ_P'); % P gain of the vertical position controller

% Parameters z velocity controller
simIn.param.PSCD.P_VEL = getParamVal(sid, 'PSC_VELZ_P'); % P gain of the vertical velocity controller
simIn.param.PSCD.I_VEL = getParamVal(sid, 'PSC_VELZ_I'); % I gain of the vertical velocity controller
simIn.param.PSCD.D_VEL = getParamVal(sid, 'PSC_VELZ_D'); % D gain of the vertical velocity controller
simIn.param.PSCD.IMAX_VEL = getParamVal(sid, 'PSC_VELZ_IMAX'); % I gain maximu vertical velocity controller
simIn.param.PSCD.FF_VEL = getParamVal(sid, 'PSC_VELZ_FF'); % Feed Forward gain of the vertical velocity controller
simIn.param.PSCD.FLTE_VEL = getParamVal(sid, 'PSC_VELZ_FLTE'); % Cutoff frequency of the error filter (in Hz)
simIn.param.PSCD.FLTD_VEL = getParamVal(sid, 'PSC_VELZ_FLTD'); % Cutoff frequency of the D term filter (in Hz)
simIn.param.PSCD.CTRL_RATIO_INIT = single(2.0); % Initial value of _vel_z_control_ratio, defined in PosControl.h, line 456

% Parameters z acceleration controller
simIn.param.PIDA.P = getParamVal(sid, 'PSC_ACCZ_P'); % P gain of the vertical acceleration controller
simIn.param.PIDA.I = getParamVal(sid, 'PSC_ACCZ_I'); % I gain of the vertical acceleration controller
simIn.param.PIDA.D = getParamVal(sid, 'PSC_ACCZ_D'); % D gain of the vertical acceleration controller
simIn.param.PIDA.IMAX = getParamVal(sid, 'PSC_ACCZ_IMAX'); % I gain maximu vertical acceleration controller
simIn.param.PIDA.FF = getParamVal(sid, 'PSC_ACCZ_FF'); % Feed Forward gain of the vertical acceleration controller
simIn.param.PIDA.FLTE_f = getParamVal(sid, 'PSC_ACCZ_FLTE'); % Cutoff frequency of the error filter (in Hz)
simIn.param.PIDA.FLTD_f = getParamVal(sid, 'PSC_ACCZ_FLTD'); % Cutoff frequency of the D term filter (in Hz)
simIn.param.PIDA.FLTT_f = getParamVal(sid, 'PSC_ACCZ_FLTT'); % Cutoff frequency of the target filter (in Hz)
simIn.param.PIDA.SR_MAX = getParamVal(sid, 'PSC_ACCZ_SMAX'); % Upper limit of the slew rate produced by combined P and D gains
simIn.param.PIDA.SR_TAU = single(1.0); % Slew Rate Tau - not yet available as a parameter of the copter. Set to 1.0 by default (AC_PID.h, line 24).
simIn.param.PIDA.SR_FLT_f = single(25.0);  % Slew Rate lowpass filter cutoff frequency. Defined in SlewLimiter.h, line 14.

% Read signals for z position controller if PSCD message was logged
if isfield(sid, 'PSCD')
    % Signals of PSCD message
    iVec = (sid.PSCD.TimeS >= tStart & sid.PSCD.TimeS <= tEnd);
    simIn.signals.PSCD.Time = single(sid.PSCD.TimeS(iVec)-tStart);
    % Inputs - Multiply by 100 to get cm as used in controller
    % Inversion of signals necessary due to inverted logging
    simIn.signals.PSCD.zPosTar = single(-sid.PSCD.TPD(iVec)*100); % Target z-Position, calculated in set_pos_target_z_from_climb_rate_cm()
    simIn.signals.PSCD.zPosAct = single(-sid.PSCD.PD(iVec)*100); % Actual z-Position, obtained from INAV
    simIn.signals.PSCD.zVelDes = single(-sid.PSCD.DVD(iVec)*100); % Desired z-Velocity, calculated in set_pos_target_z_from_climb_rate_cm()
    simIn.signals.PSCD.zVelAct = single(-sid.PSCD.VD(iVec)*100); % Actual z-Velocity, obtained from INAV
    simIn.signals.PSCD.zVelTar = single(-sid.PSCD.TVD(iVec)*100); % Target z-Velocity, calculated in update_z_controller()
    simIn.signals.PSCD.zAccDes = single(-sid.PSCD.DAD(iVec)*100); % Desired z-Acceleration, calculated in set_pos_target_z_from_climb_rate_cm()
    simIn.signals.PSCD.zAccAct = single(-sid.PSCD.AD(iVec)*100); % Actual z-Acceleration, obtained from AHRS  
    simIn.signals.PSCD.zAccTar = single(-sid.PSCD.TAD(iVec)*100); % Target z-Acceleration, calculated in update_z_controller()  
    
    % Signals of PIDA message
    iVec = (sid.PIDA.TimeS >= tStart & sid.PIDA.TimeS <= tEnd);
    simIn.signals.PIDA.Time = single(sid.PIDA.TimeS(iVec)-tStart);
    % Inputs PID z acceleration
    simIn.signals.PIDA.Err = single(sid.PIDA.Err(iVec)); % Error between target and actual z-acceleration
    simIn.signals.PIDA.Tar = single(sid.PIDA.Tar(iVec));
    simIn.signals.PIDA.Act = single(sid.PIDA.Act(iVec));
    if simIn.param.rateCtrlVal || simIn.param.attCtrlVal || simIn.param.mdlVal || simIn.param.altCtrlVal
        % Clock of Slew Limiter
        % Use tracked logging time of PID message in validation modes
        simIn.signals.PIDA.ClockDMod = single(1000 * simIn.signals.PIDA.Time);
    else
        simIn.signals.PIDA.ClockDMod = single(1000 * simIn.param.dt * (0:length(simIn.signals.PIDA.Time)-1)');
    end
    % Outputs PID z acceleration
    simIn.signals.PIDA.FF = single(sid.PIDA.FF(iVec));
    simIn.signals.PIDA.P = single(sid.PIDA.P(iVec));
    simIn.signals.PIDA.I = single(sid.PIDA.I(iVec));
    simIn.signals.PIDA.D = single(sid.PIDA.D(iVec));
    simIn.signals.PIDA.DMod = single(sid.PIDA.Dmod(iVec));
    simIn.signals.PIDA.ILimit = single(sid.PIDA.Limit(iVec));
    simIn.signals.PIDA.SRate = single(sid.PIDA.SRate(iVec));    
    
    % Inital inputs
    if simIn.param.rateCtrlVal || simIn.param.attCtrlVal || simIn.param.mdlVal || simIn.param.altCtrlVal
        % Initial actual values - in meter
        simIn.init.PSCD.posAct = single(-sid.PSCD.PD(1));
        simIn.init.PSCD.velAct = single(-sid.PSCD.VD(1));
        simIn.init.PSCD.accAct = single(-sid.PSCD.AD(1));
        
        % Init the position controller to the current position, velocity
        % and acceleration (from AC_PosControl::init_z()) in cm
        simIn.init.PIDA.posTar = single(-sid.PSCD.TPD(1)*100);
        simIn.init.PIDA.velDes = single(-sid.PSCD.DVD(1)*100);
        simIn.init.PIDA.accDes = single(min(max(-sid.PSCD.DAD(1)*100, -simIn.param.PSCD.ACC_MAX_Z_DFLT) ,simIn.param.PSCD.ACC_MAX_Z_DFLT));
        
        simIn.init.PIDA.P = single(sid.PIDA.P(1));
        simIn.init.PIDA.I = single(simIn.signals.CTUN.ThrIn(1) - simIn.signals.CTUN.ThrHov(1)) * 1000 ...
            - simIn.param.PIDA.P * (-sid.PSCD.TAD(1)*100 - (-sid.PSCD.AD(1))*100) ...
            - simIn.param.PIDA.FF * sid.PIDA.Tar(1); % Integrator init. according to AC_PosControl::init_z_controller()
        simIn.init.PIDA.D = single(sid.PIDA.D(1));
        simIn.init.PIDA.TarFilt = single(sid.PIDA.Tar(1));
        simIn.init.PIDA.ErrFilt = single(sid.PIDA.Err(1));
        simIn.init.PIDA.SROut = single(sid.PIDA.SRate(1)); % Initial value of the slew rate determined by the slew limiter. Used for both modifier_slew_rate and output_slew_rate.
        if (simIn.param.PIDA.D ~= 0)
            simIn.init.PIDA.DerFilt = single(sid.PIDA.D(1))/PSC_ACC_Z.param.D; % Divide through D gain to obtain inital D input
        else
            simIn.init.PIDA.DerFilt = single(0);
        end
    else
        % Initial actual values
        simIn.init.PSCD.posAct = single(0);
        simIn.init.PSCD.velAct = single(0);
        simIn.init.PSCD.accAct = single(0);
        
        simIn.init.PIDA.posTar = single(0);
        simIn.init.PIDA.velDes = single(0);
        simIn.init.PIDA.accDes = single(0);
        
        simIn.init.PIDA.P = single(0);
        simIn.init.PIDA.I = single(0);
        simIn.init.PIDA.D = single(0);
        simIn.init.PIDA.TarFilt = single(0);
        simIn.init.PIDA.ErrFilt = single(0);
        simIn.init.PIDA.DerFilt = single(0);
        simIn.init.PIDA.SROut = single(0);
    end
else
    % Set all signals to zero, if PSCD message was not logged and z
    % controller has been deactivated
    iVec = (sid.RATE.TimeS >= tStart & sid.RATE.TimeS <= tEnd);
    simIn.signals.PSCD.Time = single(sid.RATE.TimeS(iVec)-tStart);
    simIn.signals.PSCD.zPosTar = single(zeros(length(simIn.signals.PSCD.Time),1)); % Target z-Position, calculated in set_pos_target_z_from_climb_rate_cm()
    simIn.signals.PSCD.zPosAct = single(zeros(length(simIn.signals.PSCD.Time),1)); % Actual z-Position, obtained from INAV
    simIn.signals.PSCD.zVelDes = single(zeros(length(simIn.signals.PSCD.Time),1)); % Desired z-Velocity, calculated in set_pos_target_z_from_climb_rate_cm()
    simIn.signals.PSCD.zVelAct = single(zeros(length(simIn.signals.PSCD.Time),1)); % Actual z-Velocity, obtained from INAV
    simIn.signals.PSCD.zVelTar = single(zeros(length(simIn.signals.PSCD.Time),1)); % Target z-Velocity, calculated in update_z_controller()
    simIn.signals.PSCD.zAccDes = single(zeros(length(simIn.signals.PSCD.Time),1)); % Desired z-Acceleration, calculated in set_pos_target_z_from_climb_rate_cm()
    simIn.signals.PSCD.zAccAct = single(zeros(length(simIn.signals.PSCD.Time),1)); % Actual z-Acceleration, obtained from AHRS  
    simIn.signals.PSCD.zAccTar = single(zeros(length(simIn.signals.PSCD.Time),1)); % Target z-Acceleration, calculated in update_z_controller()
    
    % Signals of PIDA message
    simIn.signals.PIDA.Time = single(simIn.signals.RATE.Time)-tStart;
    % Inputs PID z acceleration
    simIn.signals.PIDA.Err = single(zeros(length(simIn.signals.PIDA.Time),1)); % Error between target and actual z-acceleration
    simIn.signals.PIDA.ClockDMod = single(zeros(length(simIn.signals.PIDA.Time),1)); % Clock of Slew Limiter

    % Outputs PID z acceleration
    simIn.signals.PIDA.FF = single(zeros(length(simIn.signals.PIDA.Time),1));
    simIn.signals.PIDA.P = single(zeros(length(simIn.signals.PIDA.Time),1));
    simIn.signals.PIDA.I = single(zeros(length(simIn.signals.PIDA.Time),1));
    simIn.signals.PIDA.D = single(zeros(length(simIn.signals.PIDA.Time),1));
    simIn.signals.PIDA.DMod = single(zeros(length(simIn.signals.PIDA.Time),1));
    simIn.signals.PIDA.ILimit = single(zeros(length(simIn.signals.PIDA.Time),1));
    simIn.signals.PIDA.SRate = single(zeros(length(simIn.signals.PIDA.Time),1)); 
    simIn.signals.PIDA.Act = single(zeros(length(simIn.signals.PIDA.Time),1));
    simIn.signals.PIDA.Tar = single(zeros(length(simIn.signals.PIDA.Time),1));
    
    % Initial inputs
    % Initial actual values
    simIn.init.PSCD.posAct = single(0);
    simIn.init.PSCD.velAct = single(0);
    simIn.init.PSCD.accAct = single(0);
    
    simIn.init.PIDA.posTar = single(0);
    simIn.init.PIDA.velDes = single(0);
    simIn.init.PIDA.accDes = single(0);
    
    simIn.init.PIDA.P = single(0);
    simIn.init.PIDA.I = single(0);
    simIn.init.PIDA.D = single(0);
    simIn.init.PIDA.TarFilt = single(0);
    simIn.init.PIDA.ErrFilt = single(0);
    simIn.init.PIDA.DerFilt = single(0);
    simIn.init.PIDA.SROut = single(0);
end


% Real Flight Signals of PID z velocity - only load if log messages
% exists
if isfield(sid, 'PCVZ')
    iVec = (sid.PCVZ.TimeS >= tStart & sid.PCVZ.TimeS <= tEnd);
    simIn.signals.PCVZ.Time = single(sid.PCVZ.TimeS(iVec)-tStart);
    simIn.signals.PCVZ.Err = single(sid.PCVZ.Err(iVec));
    simIn.signals.PCVZ.P = single(sid.PCVZ.P(iVec));
    simIn.signals.PCVZ.I = single(sid.PCVZ.I(iVec));
    simIn.signals.PCVZ.D = single(sid.PCVZ.D(iVec));
    simIn.signals.PCVZ.FF = single(sid.PCVZ.FF(iVec));
else % Set to zero otherwise
    iVec = (sid.RATE.TimeS >= tStart & sid.RATE.TimeS <= tEnd);
    simIn.signals.PCVZ.Time = single(sid.RATE.TimeS(iVec)-tStart);
    simIn.signals.PCVZ.Err = single(zeros(length(simIn.signals.PCVZ.Time),1));
    simIn.signals.PCVZ.P = single(zeros(length(simIn.signals.PCVZ.Time),1));
    simIn.signals.PCVZ.I = single(zeros(length(simIn.signals.PCVZ.Time),1));
    simIn.signals.PCVZ.D = single(zeros(length(simIn.signals.PCVZ.Time),1));
    simIn.signals.PCVZ.FF = single(zeros(length(simIn.signals.PCVZ.Time),1));
end

%% Load identified plant models data
if simIn.param.optCtrl || simIn.param.mdlVal && exist('idModel', 'var')
    % Roll
    if isempty(findobj(idModel, 'axis', 'RLL'))
        simIn.models.RollAxis = idpoly(tf(1, [1 1], simIn.param.dt));
    else
        model = findobj(idModel, 'axis', 'RLL');
        % If more than one model is found, let user choose
        if length(model) > 1 
            mdlIdx = input('More than one model found for the roll axis. Input index of desired model: ');
        else
            mdlIdx = 1;
        end
        simIn.models.RollAxis = idss(c2d(model(mdlIdx).tfModel, simIn.param.dt));
        % Decrease delay to account for the additional feedback delay
        simIn.models.RollAxis.InputDelay = simIn.models.RollAxis.InputDelay-1;
    end
    % Pitch
    if isempty(findobj(idModel, 'axis', 'PIT'))
        simIn.models.PitchAxis = idpoly(tf(1, [1 1], simIn.param.dt));
    else
        model = findobj(idModel, 'axis', 'PIT');
        % If more than one model is found, let user choose
        if length(model) > 1 
            mdlIdx = input('More than one model found for the pitch axis. Input index of desired model: ');
        else
            mdlIdx = 1;
        end
        simIn.models.PitchAxis = idss(c2d(model(mdlIdx).tfModel, simIn.param.dt));
        simIn.models.PitchAxis.InputDelay = simIn.models.PitchAxis.InputDelay-1;
    end
    % Yaw
    if isempty(findobj(idModel, 'axis', 'YAW'))
        simIn.models.YawAxis = idpoly(tf(1, [1 1], simIn.param.dt));
    else
        model = findobj(idModel, 'axis', 'YAW');
        % If more than one model is found, let user choose
        if length(model) > 1 
            mdlIdx = input('More than one model found for the yaw axis. Input index of desired model: ');
        else
            mdlIdx = 1;
        end
        simIn.models.YawAxis = idss(c2d(model(mdlIdx).tfModel, simIn.param.dt));
        simIn.models.YawAxis.InputDelay = simIn.models.YawAxis.InputDelay-1;
    end
    % Vertical Motion - Throttle
    if isempty(findobj(idModel, 'axis', 'THR'))
        simIn.models.RollAxis = idpoly(tf(1, [1 1], simIn.param.dt));
    else
        model = findobj(idModel, 'axis', 'THR');
        % If more than one model is found, let user choose
        if length(model) > 1
            mdlIdx = input('More than one model found for the vertical axis. Input index of desired model: ');
        else
            mdlIdx = 1;
        end
        simIn.models.VerticalAxis = idss(c2d(model(mdlIdx).tfModel, simIn.param.dt));
        % Decrease delay to account for the additional feedback delay
        simIn.models.VerticalAxis.InputDelay = simIn.models.VerticalAxis.InputDelay-1;
    end
else % Create dummy models of optimization is not active
    simIn.models.RollAxis = idpoly(tf(1, [1 1], simIn.param.dt));
    simIn.models.PitchAxis = idpoly(tf(1, [1 1], simIn.param.dt));
    simIn.models.YawAxis = idpoly(tf(1, [1 1], simIn.param.dt));
    simIn.models.VerticalAxis = idpoly(tf(1, [1 1], simIn.param.dt));
end

% Initial values
if simIn.param.mdlVal && mode == 25
    simIn.models.RollInit = single(sid.SIDD.Gx(1)*pi/180);
    simIn.models.PitchInit = single(sid.SIDD.Gy(1)*pi/180);
    simIn.models.YawInit = single(sid.SIDD.Gz(1)*pi/180);
    simIn.models.VerticalOutInit = single(sid.SIDD.Az(1));
    simIn.models.VerticalInInit = single(sid.RATE.AOut(1));
else
    simIn.models.RollInit = single(0);
    simIn.models.PitchInit = single(0);
    simIn.models.YawInit = single(0);
    simIn.models.VerticalOutInit = single(-9.81);
    simIn.models.VerticalInInit = single(sid.RATE.AOut(1));
end

%% Delete all irrelevant data
clear i dist start thr_man rel_msg mode param_names msgs tEnd tStart simMode
clear phase_des runOutsideLoop mdlIdx pitchCh rollCh yawCh iVec Fs axis dt
clear simMode thrCh 
