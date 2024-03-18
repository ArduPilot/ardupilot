function RCChannelNumberForAuxFunction = findRCChannelNumberForAuxFunction(obj,functionNumber)
% Return a channel number for the supplied function number from the enumeration in RC_Channel.h
%
% Peter Barker, based on getParamVal.m by Fabian Bredemeier
% License: GPL v3

% Get list of parameter names
param_names = cell(length(obj.PARM.Name),1);
for i=1:length(obj.PARM.Name)
   param_names{i,1} = deblank(obj.PARM.Name(i,1:16));
end

% Translate parameter name to cell array if only one name is provided
if ~iscell(paramName)
    desParamNames{1} = paramName;
else
    desParamNames = paramName;
end

FIXME below this point

% Read parameter(s) with boolean indexing
paramVals = single(zeros(numel(desParamNames),1));
for i = 1:length(desParamNames)
    paramIdx = strcmp(param_names, desParamNames{i});
    paramVal = single(obj.PARM.Value(paramIdx));
    if isempty(paramVal)
        warning('Parameter could not be found!');
        paramVal = [];
        return;
    end
    paramVals(i,1) = paramVal(1); % Prevent nultidimensional parameters
    
    if isempty(paramVals)
        error(['Parameter ' desParam{i} ' could not be found.']);
    end
end

end

