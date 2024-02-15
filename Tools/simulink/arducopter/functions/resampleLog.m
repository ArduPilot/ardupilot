function [logResampled] = resampleLog(log,resampling,method)
% Resamples log if resample is set to true, so that all included messages 
% have the same first time stamp and the same amount of elements 
% (if they have the same sample time)
% If resample is set to false, the Time vector of all messages is just
% adapted to the first time stamp of the log
%
% Fabian Bredemeier - IAV GmbH
% License: GPL v3

headerProps2Copy = {'fileName', 'filePathName', 'platform', 'version', ...
    'commit', 'bootTimeUTC', 'msgsContained', 'totalLogMsgs', 'msgFilter', ...
    'numMsgs'};

% Define messages that are to be excluded from resampling, but still copied
msgs2Exclude = {'FMT', 'FMTU', 'MODE', 'MULT', 'UNIT', 'PARM', 'SIDS'};

% Messages with multiple instances
msgWithMultInstances = {'ARSP', 'BARO', 'BAT', 'BCL', 'BCL2', 'ESC', ...
    'GPA', 'GPS', 'IMU', 'MAG', 'PRX', 'RFND'};

% Define message header props that need to be copied to new log
msgProps2Copy = {'typeNumID', 'fieldUnits', 'fieldMultipliers', 'name', ...
    'MsgInstance'};

% Define message props that need to be excluded in new log
msgProps2Exclude = {'TimeS', 'LineNo', 'DatenumUTC', 'TimeUS'};

% Define sample times that are used for logging
TsVec = [0.00025 0.001 0.0025 0.01 0.02 0.04 0.1 0.2 1];

% Allowed jitter percentage of sample time
TsJitPerc = 0.5;

% Get first timestamp of log
timeZero = getTimeZero(log, log.msgsContained, msgs2Exclude);

% Convert LogMsgGroup object into structs
log = log.getStruct();

for prop = string(fieldnames(log))'
    propObj = log.(prop);
    
    % Just copy property if contained in cell array
    if any(strcmp(prop, headerProps2Copy))
        logResampled.(prop) = propObj;
        continue;
    end
    
    % Skip message if it shall be excluded
    if any(strcmp(prop, msgs2Exclude))
        logResampled.(prop) = propObj;
        continue;
    end
    
    % Skip message if it is a deleted handle
    if isempty(propObj)
        continue;
    end
    
    % Handle messages with multiple instances (like BAT)
    if any(strcmp(msgWithMultInstances, prop))
        % Create idx vector for certain instance
        if isfield(propObj, 'Instance') % Check message name of instance
            idxVec = propObj.Instance == 0;
        elseif isfield(propObj, 'I')
            idxVec = propObj.I == 0;
        elseif isfield(propObj, 'C') % XKF messages
            idxVec = propObj.C == 0;
        end
    else % Message without multiple instances
        idxVec = logical(ones(length(propObj.TimeS),1));
    end
    
    % Get plausibilized time vector
    timePlaus = plausibilizeTime(log.(prop).TimeS(idxVec));
    
    % Determine sample time of message
    TsRaw = mean(diff(timePlaus));
    Ts = 0;
    for TsElem = TsVec
        if TsRaw > TsElem*(1-TsJitPerc) && TsRaw < TsElem*(1+TsJitPerc)
            Ts = TsElem;
            break;
        end
    end
    % Throw error if Ts could not be determined
    if Ts == 0
        error(['Sample time of message ' char(prop) ' could not be determined. Mean: ' num2str(TsRaw) 's']);
    end
    
    % Copy properties to new object. Resample signals
    msgProps = string(fieldnames(log.(prop)))';
    for msgProp = msgProps(1:end)
                
        % Copy property if included in msgProps2Copy
        if any(strcmp(msgProp, msgProps2Copy))
            logResampled.(prop).(msgProp) = propObj.(msgProp);
            continue;
        end
        
        % Skip property if included in msgProps2Exclude
        if any(strcmp(msgProp, msgProps2Exclude))
            continue;
        end
        
        % Get signal values for the correct indices
        signal = propObj.(msgProp)(idxVec);
        
        % Define time starting from first time stamp
        time = propObj.TimeS(idxVec) - timeZero;
        
        % Plausibilize time and signal vector
        [time, signal] = plausibilizeTime(time, signal);
        
        % If any time sample is still larger than the final time,
        % plausibilize the time vector again
        while(any(time > time(end)))
            [time, signal] = plausibilizeTime(time, signal);
        end
        
        % Define old time vector. Insert zero at beginning for the signals
        % to start at 0s. Insert first signal value in value vector.
        if time(1) > 1e-3
            timeOld = zeros(length(time)+1,1);
            signalOld = zeros(length(signal)+1,1);
            timeOld(2:length(time)+1, 1) = time;
            timeOld(1) = 0;
            signalOld(2:length(signal)+1, 1) = signal;
            signalOld(1) = signal(1);
        else
            timeOld = time;
            signalOld = signal;
        end
        
        % Catch nan in signal
        if any(isnan(signalOld))
            signalOld = zeros(length(signalOld),1);
            warning(['Detected nan in ' char(prop) '.' char(msgProp)])
        end
        
        if resampling
            % Resample signal
            if any(strcmp(method, 'makima')) || any(strcmp(method, 'nearest'))
                len = round(timeOld(end) / Ts, 0); % Get amount of time vec elements
                timeNew = Ts * (0:len)';
                signalNew = interp1(timeOld, signalOld, timeNew, method, 'extrap');
            else
                [signalNew, timeNew] = resample(signalOld, timeOld, 1/Ts, method);
            end
        else
            signalNew = signal;
            timeNew = Ts*(0:length(signal)-1)';
        end
                
        % Insert property
        logResampled.(prop).(msgProp) = signalNew;
        % Add new property sampling time Ts
        logResampled.(prop).Ts = Ts;
    end
    
    % Add Time Property
    logResampled.(prop).TimeS = timeNew;
end
end

