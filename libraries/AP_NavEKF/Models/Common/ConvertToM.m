function ConvertToM(nStates)
%% Initialize variables
fileName = strcat('SymbolicOutput',int2str(nStates),'.txt');
delimiter = '';

%% Format string for each line of text:
%   column1: text (%s)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%s%[^\n\r]';

%% Open the text file.
fileID = fopen(fileName,'r');

%% Read columns of data according to format string.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter,  'ReturnOnError', false,'Bufsize',65535);

%% Close the text file.
fclose(fileID);

%% Create output variable
SymbolicOutput = [dataArray{1:end-1}];

%% Clear temporary variables
clearvars filename delimiter formatSpec fileID dataArray ans;

%% replace brackets and commas
for lineIndex = 1:length(SymbolicOutput)
    SymbolicOutput(lineIndex) = regexprep(SymbolicOutput(lineIndex), '_l_', '(');
    SymbolicOutput(lineIndex) = regexprep(SymbolicOutput(lineIndex), '_c_', ',');
    SymbolicOutput(lineIndex) = regexprep(SymbolicOutput(lineIndex), '_r_', ')');
end

%% Write to file
fileName = strcat('M_code',int2str(nStates),'.txt');
fid = fopen(fileName,'wt');
for lineIndex = 1:length(SymbolicOutput)
    fprintf(fid,char(SymbolicOutput(lineIndex)));
    fprintf(fid,'\n');
end
fclose(fid);
clear all;

end