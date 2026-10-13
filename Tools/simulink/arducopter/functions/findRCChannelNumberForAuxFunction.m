function RCChannelNumberForAuxFunction = findRCChannelNumberForAuxFunction(obj,functionNumber)
% Return the number of the RC channel whose RCn_OPTION parameter is set
% to the supplied function number from the AUX_FUNC enumeration in
% RC_Channel.h, e.g. 201 for roll input
%
% Peter Barker, based on getParamVal.m by Fabian Bredemeier
% License: GPL v3

% Get list of parameter names
param_names = cell(length(obj.PARM.Name),1);
for i=1:length(obj.PARM.Name)
   param_names{i,1} = deblank(obj.PARM.Name(i,1:16));
end

% Find the first channel with the function assigned
for chan = 1:16
    paramIdx = strcmp(param_names, ['RC' num2str(chan) '_OPTION']);
    paramVal = obj.PARM.Value(paramIdx);
    if ~isempty(paramVal) && paramVal(1) == functionNumber
        RCChannelNumberForAuxFunction = chan;
        return;
    end
end

% Logs from before the RCn_OPTION change carry the RCMAP_ parameter instead
rcmapNames = {201, 'RCMAP_ROLL'; 202, 'RCMAP_PITCH'; 203, 'RCMAP_THROTTLE'; 204, 'RCMAP_YAW'};
for i = 1:size(rcmapNames, 1)
    if rcmapNames{i, 1} == functionNumber
        paramIdx = strcmp(param_names, rcmapNames{i, 2});
        paramVal = obj.PARM.Value(paramIdx);
        if ~isempty(paramVal)
            RCChannelNumberForAuxFunction = paramVal(1);
            return;
        end
    end
end

error(['No RC channel has option ' num2str(functionNumber) ' assigned.']);

end
