function FixAutoGenCCode(fileName)
%% Initialize variables
delimiter = '';

%% Format string for each line of text:
%   column1: text (%s)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%s%[^\n\r]';

%% Open the text file.
fileID = fopen(strcat(fileName,'.c'),'r');

%% Read columns of data according to format string.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter,  'ReturnOnError', false);

%% Close the text file.
fclose(fileID);

%% Create output variable
SymbolicOutput = [dataArray{1:end-1}];

%% Clear temporary variables
clearvars filename delimiter formatSpec fileID dataArray ans;

%% Convert 1 based indexes in symbolic array variables
for lineIndex = 1:length(SymbolicOutput)
    SymbolicOutput(lineIndex) = regexprep(SymbolicOutput(lineIndex), '_l_', '(');
    SymbolicOutput(lineIndex) = regexprep(SymbolicOutput(lineIndex), '_c_', ',');
    SymbolicOutput(lineIndex) = regexprep(SymbolicOutput(lineIndex), '_r_', ')');
end

% replace 2-D left indexes
for arrayIndex = 1:99
    strIndex = int2str(arrayIndex);
    strRep = sprintf('[%d,',(arrayIndex-1));
    strPat = strcat('\(',strIndex,'\,');
    for lineIndex = 1:length(SymbolicOutput)
        str = char(SymbolicOutput(lineIndex));
        SymbolicOutput(lineIndex) = {regexprep(str, strPat, strRep)};
    end
end

% replace 2-D right indexes
for arrayIndex = 1:99
    strIndex = int2str(arrayIndex);
    strRep = sprintf(',%d]',(arrayIndex-1));
    strPat = strcat('\,',strIndex,'\)');
    for lineIndex = 1:length(SymbolicOutput)
        str = char(SymbolicOutput(lineIndex));
        SymbolicOutput(lineIndex) = {regexprep(str, strPat, strRep)};
    end
end

% replace commas
for lineIndex = 1:length(SymbolicOutput)
    str = char(SymbolicOutput(lineIndex));
    SymbolicOutput(lineIndex) = {regexprep(str, '\,', '][')};
end

%% add float declarations in front of temporary variables
expression = 't(\w*) =';
replace = 'float t$1 =';
for lineIndex = 1:length(SymbolicOutput)
    SymbolicOutput(lineIndex) = regexprep(SymbolicOutput(lineIndex),expression,replace);
end

%% replace (1.0/2.0) with 0.5f
expression = '\(1.0/2.0\)';
replace = '0.5f';
for lineIndex = 1:length(SymbolicOutput)
    SymbolicOutput(lineIndex) = regexprep(SymbolicOutput(lineIndex),expression,replace);
end

%% replace 2.0
expression = '2\.0';
replace = '2.0f';
for lineIndex = 1:length(SymbolicOutput)
    SymbolicOutput(lineIndex) = regexprep(SymbolicOutput(lineIndex),expression,replace);
end

%% Write to file
fileName = strcat(fileName,'.cpp');
fid = fopen(fileName,'wt');
for lineIndex = 1:length(SymbolicOutput)
    fprintf(fid,char(SymbolicOutput(lineIndex)));
    fprintf(fid,'\n');
end
fclose(fid);
clear all;

end
