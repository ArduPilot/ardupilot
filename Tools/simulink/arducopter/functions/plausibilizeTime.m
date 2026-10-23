function [timePlaus, signalPlaus] = plausibilizeTime(time,signal)
% Checks if time vector has a major tracking error and delete respective
% element from time and signal vector
%
% Fabian Bredemeier - IAV GmbH
% License: GPL v3

% Get indices for time values that are too high
validIdx = ~(time > 1e6);
if time(1) > time(2) % First time stamp is invalid if greater than second
    validIdx(1) = 0;
end

% Calculate average sample time
Ts_avg = sum(diff(time(validIdx)))/length(time);

% Only plausibilize time vector
if nargin == 1
    % Set first element
    if time(1) > 1e6 && time(2) < 1e6 || time(1) > time(2)
        timePlaus = time(2)-Ts_avg;
    else
        timePlaus = time(1);
    end 
    idxDiff = 0; % Tracking of removed elements
    skipNext = 0; % Flag to skip next element
    for i = 2:length(time)-1
        if skipNext
            skipNext = 0;
            continue;
        end
        if(time(i+1)<time(i))
            % Check if current or next element is incorrect
            if abs((time(i)-time(i-1))-Ts_avg) > abs((time(i+1)-time(i-1))-Ts_avg) && time(i+1)-time(i-1) > 0
                % Current element is wrong since deviation to average
                % sample time is larger - skip current and do nothing
                
            else
                % Next element is wrong, so add current to corrected vector
                timePlaus(i-idxDiff,1) = time(i);
                
                % Make sure next one is skipped
                skipNext = 1;
            end
            idxDiff = idxDiff + 1;
        else
            timePlaus(i-idxDiff,1) = time(i);
        end
    end
    % Add last element
    len = length(time);
    timePlaus(len-idxDiff,1) = time(end);
    return;

% Also plausibilize signal vector
elseif nargin == 2
    % Set first element
    if time(1) > 1e6 && time(2) < 1e6 || time(1) > time(2)
        timePlaus = time(2)-Ts_avg;
        signalPlaus = signal(2);
    else
        timePlaus = time(1);
        signalPlaus = signal(1);
    end
    idxDiff = 0; % Tracking of removed elements
    skipNext = 0; % Flag to skip next element
    for i = 2:length(time)-1
        if skipNext
            skipNext = 0;
            continue;
        end
        if(time(i+1)<time(i))
            % Check if current or next element is incorrect
            if abs((time(i)-time(i-1))-Ts_avg) > abs((time(i+1)-time(i-1))-Ts_avg) && time(i+1)-time(i-1) > 0
                % Current element is wrong since deviation to average
                % sample time is larger - skip current and do nothing
                
            else
                % Next element is wrong, so add current to corrected vector
                timePlaus(i-idxDiff,1) = time(i);
                signalPlaus(i-idxDiff,1) = signal(i);
                
                % Make sure next one is skipped
                skipNext = 1;
            end
            idxDiff = idxDiff + 1;
        else
            timePlaus(i-idxDiff,1) = time(i);
            signalPlaus(i-idxDiff,1) = signal(i);
        end
    end
    % Add last element
    len = length(time);
    timePlaus(len-idxDiff,1) = time(end);
    signalPlaus(len-idxDiff,1) = signal(end);
end
end

