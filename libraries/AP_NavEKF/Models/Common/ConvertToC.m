function ConvertToC(fileName)
delimiter = '';

%% Format string for each line of text:
%   column1: text (%s)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%s%[^\n\r]';

%% Open the text file.
fileID = fopen(strcat(fileName,'.m'),'r');

%% Read columns of data according to format string.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter,  'ReturnOnError', false, 'Bufsize', 65535);

%% Close the text file.
fclose(fileID);

%% Create output variable
SymbolicOutput = [dataArray{1:end-1}];

%% Clear temporary variables
clearvars filename delimiter formatSpec fileID dataArray ans;

%% Convert indexing and replace brackets

% replace 1-D indexes
for arrayIndex = 1:99
    strIndex = int2str(arrayIndex);
    strRep = sprintf('[%d]',(arrayIndex-1));
    strPat = strcat('\(',strIndex,'\)');
    for lineIndex = 1:length(SymbolicOutput)
        str = char(SymbolicOutput(lineIndex));
        SymbolicOutput(lineIndex) = {regexprep(str, strPat, strRep)};
    end
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

%% replace . operators
for lineIndex = 1:length(SymbolicOutput)
    strIn = char(SymbolicOutput(lineIndex));
    strIn = regexprep(strIn,'\.\*','\*');
    strIn = regexprep(strIn,'\.\/','\/');
    strIn = regexprep(strIn,'\.\^','\^');
    SymbolicOutput(lineIndex) = cellstr(strIn);
end

%% Replace ^2

% replace where adjacent to ) parenthesis
for lineIndex = 1:length(SymbolicOutput)
    idxsq = regexp(SymbolicOutput(lineIndex),'\)\^2');
    if ~isempty(idxsq{1})
        strIn = SymbolicOutput(lineIndex);
        idxlp = regexp(strIn,'\(');
        idxrp = regexp(strIn,'\)');
        for pwrIndex = 1:length(idxsq{1})
            counter = 1;
            index = idxsq{1}(pwrIndex);
            endIndex(pwrIndex) = index;
            while (counter > 0 && index > 0)
                index = index - 1;
                counter = counter + ~isempty(find(idxrp{1} == index, 1));
                counter = counter - ~isempty(find(idxlp{1} == index, 1));
            end
            startIndex(pwrIndex) = index;
            %            strPat = strcat(strIn{1}(startIndex(pwrIndex):endIndex(pwrIndex)),'^2');
            strRep = strcat('sq',strIn{1}(startIndex(pwrIndex):endIndex(pwrIndex)));
            %            cellStrPat(pwrIndex) = cellstr(strPat);
            cellStrRep(pwrIndex) = cellstr(strRep);
        end
        for pwrIndex = 1:length(idxsq{1})
            strRep = char(cellStrRep(pwrIndex));
            strIn{1}(startIndex(pwrIndex):endIndex(pwrIndex)+2) = strRep;
        end
        SymbolicOutput(lineIndex) = strIn;
    end
end

% replace where adjacent to ] parenthesis
for lineIndex = 1:length(SymbolicOutput)
    strIn = char(SymbolicOutput(lineIndex));
    [match,idxsq1,idxsq2] = regexp(strIn,'\w*\[\w*\]\^2','match','start','end');
    [idxsq3] = regexp(strIn,'\[\w*\]\^2','start');
    if ~isempty(match)
        for pwrIndex = 1:length(match)
            strVar   = strIn(idxsq1(pwrIndex):idxsq3(pwrIndex)-1);
            strIndex = strIn(idxsq3(pwrIndex)+1:idxsq2(pwrIndex)-3);
            strPat = strcat(strVar,'\[',strIndex,'\]\^2');
            strRep = strcat('sq(',strVar,'[',strIndex,']',')');
            strIn = regexprep(strIn,strPat,strRep);
        end
        SymbolicOutput(lineIndex) = cellstr(strIn);
    end
end

% replace where adjacent to alpanumeric characters
for lineIndex = 1:length(SymbolicOutput)
    strIn = char(SymbolicOutput(lineIndex));
    [match,idxsq1,idxsq2] = regexp(strIn,'\w*\^2','match','start','end');
    [idxsq3] = regexp(strIn,'\^2','start');
    if ~isempty(match)
        for pwrIndex = 1:length(match)
            strVar   = strIn(idxsq1(pwrIndex)+2*(pwrIndex-1):idxsq2(pwrIndex)-2+2*(pwrIndex-1));
            strPat = strcat(strVar,'\^2');
            strRep = strcat('sq(',strVar,')');
            strIn = regexprep(strIn,strPat,strRep);
        end
        SymbolicOutput(lineIndex) = cellstr(strIn);
    end
end

%% Replace ^(1/2)

% replace where adjacent to ) parenthesis
for lineIndex = 1:length(SymbolicOutput)
    idxsq = regexp(SymbolicOutput(lineIndex),'\)\^\(1\/2\)');
    if ~isempty(idxsq{1})
        strIn = SymbolicOutput(lineIndex);
        idxlp = regexp(strIn,'\(');
        idxrp = regexp(strIn,'\)');
        for pwrIndex = 1:length(idxsq{1})
            counter = 1;
            index = idxsq{1}(pwrIndex);
            endIndex(pwrIndex) = index;
            while (counter > 0 && index > 0)
                index = index - 1;
                counter = counter + ~isempty(find(idxrp{1} == index, 1));
                counter = counter - ~isempty(find(idxlp{1} == index, 1));
            end
            startIndex(pwrIndex) = index;
            %            strPat = strcat(strIn{1}(startIndex(pwrIndex):endIndex(pwrIndex)),'^2');
            strRep = strcat('(sqrt',strIn{1}(startIndex(pwrIndex):endIndex(pwrIndex)),')');
            %            cellStrPat(pwrIndex) = cellstr(strPat);
            cellStrRep(pwrIndex) = cellstr(strRep);
        end
        for pwrIndex = 1:length(idxsq{1})
            strRep = char(cellStrRep(pwrIndex));
            strIn{1}(startIndex(pwrIndex):endIndex(pwrIndex)+6) = strRep;
        end
        SymbolicOutput(lineIndex) = strIn;
    end
end

%% Replace Divisions
% Compiler looks after this type of optimisation for us
% for lineIndex = 1:length(SymbolicOutput)
%     strIn = char(SymbolicOutput(lineIndex));
%     strIn = regexprep(strIn,'\/2','\*0\.5');
%     strIn = regexprep(strIn,'\/4','\*0\.25');
%     SymbolicOutput(lineIndex) = cellstr(strIn);
% end

%% Convert declarations
for lineIndex = 1:length(SymbolicOutput)
    str = char(SymbolicOutput(lineIndex));
    if ~isempty(regexp(str,'zeros', 'once'))
        index1 = regexp(str,' = zeros[','once')-1;
        index2 = regexp(str,' = zeros[','end','once')+1;
        index3 = regexp(str,'\]\[','once')-1;
        index4 = index3 + 3;
        index5 = max(regexp(str,'\]'))-1;
        str1 = {'float '};
        str2 = str(1:index1);
        str3 = '[';
        str4 = str(index2:index3);
        str4 = num2str(str2num(str4)+1);
        str5 = '][';
        str6 = str(index4:index5);
        str6 = num2str(str2num(str6)+1);
        str7 = '];';
        SymbolicOutput(lineIndex) = strcat(str1,str2,str3,str4,str5,str6,str7);
    end
end

%% Change covariance matrix variable name to P
for lineIndex = 1:length(SymbolicOutput)
    strIn = char(SymbolicOutput(lineIndex));
    strIn = regexprep(strIn,'OP\[','P[');
    SymbolicOutput(lineIndex) = cellstr(strIn);
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