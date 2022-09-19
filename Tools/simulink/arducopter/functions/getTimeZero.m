function timeZero = getTimeZero(log, msgs, msgs2Ex)
% Get the earliest time stamp in the log

% Create dummy zero timestamp
if isprop(log, 'RATE')
    timeZero = log.RATE.TimeS(end);
else
    error('Message RATE not contained in log.');
end

% Define messages that should not be included in search for time stamp
if nargin < 3
    msgs2Exclude = {'FMT', 'FMTU', 'MODE', 'MULT', 'UNIT', 'PARM'};
else
    msgs2Exclude = msgs2Ex;
end

for m = msgs
    msg = m{1};
    % Skip messages to exclude
    if any(strcmp(msgs2Exclude, msg))
        continue;
    end
    
    % Output warning if message is not included in log
    if ~any(strcmp(fieldnames(log), msg))
        warning(['Message ' msg ' not included in log!']);
    end
    
    % Skip message if message is deleted handle
    if ~isvalid(log.(msg))
        continue;
    end
    
    firstTime = log.(msg).TimeS(1);
    if firstTime < timeZero
        timeZero = firstTime;
    end
end
end

