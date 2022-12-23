% Reads ArduPilot's system identification dataflash logs and extracts the SID
% subsections of the fligts into .mat file, discarding all irrelevant data
%
%
% Fabian Bredemeier - IAV GmbH
% License: GPL v3

close all;

%% File selection by user

% ---------------------------------------------- %
%                                                %
% File selection                                 %  
%                                                %   
% ---------------------------------------------- %

% 1) Ask user if next data should be added to existing data set
if exist('sidLogs', 'var')
    % Abort if config data for existing data is missing
    if ~exist('loadedDataConfig', 'var')
        error('No configuration of the existing data could be found. Aborting...');
    end
    addData = input('Add next data to existing data set (0: no, 1: yes)?');
    if addData > 1 && ~isnumeric(addData)
        error('Input not valid. Only 0 or 1 are allowed.')
    end
else
    addData = 0;
end

% Delete old data of new data will not be added. Otherwise get number of
% existing elements
if ~addData
    clear sidLogs idData
    numOldSubflights = 0;
    numOldSysidflights = 0;
else
    numOldSubflights = numel(sidLogs);
    if exist('idData', 'var')
        numOldSysidflights = numel(idData);
    end
end

% 2) User chooses new flight data from explorer
mainPath = pwd;
flightDataSearchPath = [mainPath];
[files,flightDataPath] = uigetfile({'*.*'},...
   'Select One or More Files', ...
   'MultiSelect', 'on', ...
    flightDataSearchPath);
% If only one file is selected, convert char array to cell array
if ~iscell(files)
    files = cellstr(files);
    numFiles = 1;
else
    numFiles = size(files,2);
end

% 3) Get file extensions - .BIN oder .mat
fileExt = cell(numFiles,1);
for i=1:numFiles
    fileExt{i} = files{i}(end-2:end);
end

% 4) Create cell array with file names
fileNames = cell(numFiles,1);
for i=1:numFiles
    fileNames{i} = [flightDataPath files{i}];
end

% 5) Load .mat files - loadedDataConfig is struct, so load fileName field
chooseSubflights = false;
for i=1:numFiles
    if strcmp(fileExt{i}, 'mat')
        load(fileNames{i});
        fileNames = loadedDataConfig.fileNames;
        % Fill chooseSubflights variable with false for amount of
        % subflights
        for j=1:length(loadedDataConfig.subflights)
            chooseSubflights(j) = false;
        end
    else
        % Activate subflight choosing if BIN files were selected
        chooseSubflights(i) = true;
    end
end

%% Load complete log data and show tested modes

% ---------------------------------------------- %
%                                                %
% Loading Dataflash Logs                         %  
%                                                %   
% ---------------------------------------------- %

fprintf('\n-------------------------------------------\nLoading Flash Logs...\n')
fprintf('-------------------------------------------\n');
    
% Variables to store subflight numbers,amount of subflights and modes
subflights = 0;
fileIndices = 0;
numSubflights = 0;
modes = 0;

for fileIdx = 1:length(fileNames)
    
    fprintf('Processing file %d ...\n', fileIdx);
    
    % Throw error if files cannot be found
    if ~isfile(fileNames{fileIdx})
        error(['File ' + string(fileNames{fileIdx}) + ' could not be found.']);
    end
    logs(fileIdx) = Ardupilog(char(fileNames{fileIdx}));
    
    % Plot and Output for sysid mode
    if any(logs(fileIdx).MODE.Mode == 25)
        fig = figure;
        set(fig, 'defaultAxesColorOrder', [[0 0 0]; [0 0 0]]); % Set axes color to black
        
        subplot(2, 1, 1);
        plot(logs(fileIdx).MODE.TimeS, logs(fileIdx).MODE.ModeNum, 'k*');
        ylabel(logs(fileIdx).getLabel('MODE/ModeNum'));
        if ~issorted(logs(fileIdx).MODE.TimeS, 'strictascend')
            disp([char(fileNames{fileIdx}) ' MODE.TimeS is not monotonic!'])
        end
        title(['Sysid Mode' fileNames{fileIdx}], 'Interpreter', 'none');
        
        % Plot tested SID axes
        hold on;
        yyaxis right;
        plot(logs(fileIdx).SIDS.TimeS, logs(fileIdx).SIDS.Ax, 'r*');
        ylabel(logs(fileIdx).getLabel('SIDS/Ax'));
        if ~issorted(logs(fileIdx).SIDS.TimeS, 'strictascend')
            disp([char(fileNames{fileIdx}) ' SIDS.TimeS is not monotonic!'])
        end
        xlim([logs(fileIdx).MODE.TimeS(1) logs(fileIdx).PIDR.TimeS(end)]);
        hold off;
        
        % Plot frequency of SID tests
        subplot(2, 1, 2);
        plot(logs(fileIdx).SIDD.TimeS, logs(fileIdx).SIDD.F, 'b.');
        ylabel(logs(fileIdx).getLabel('SIDD/F'));
        if ~issorted(logs(fileIdx).SIDD.TimeS, 'strictascend')
            disp([char(fileNames{fileIdx}) ' SIDD.TimeS is not monotonic!'])
        end
        hold on;
        
        % Plot subflight numbers
        yyaxis right;
        TimeS = logs(fileIdx).MODE.TimeS;
        subflightNr = 1:length(logs(fileIdx).MODE.Mode);
        plot(TimeS, subflightNr, 'k*');
        ylabel('subflight');
        if ~issorted(logs(fileIdx).SIDS.TimeS, 'strictascend')
            disp([char(fileNames{fileIdx}) ' SIDS.TimeS is not monotonic!'])
        end
        xlim([logs(fileIdx).MODE.TimeS(1) logs(fileIdx).PIDR.TimeS(end)]);
        hold off;
    end
    
    % Plot for z position controller
    if ~isempty(logs(fileIdx).PSCD.TimeS)
        fig = figure;
        set(fig, 'defaultAxesColorOrder', [[0 0 0]; [0 0 0]]); % Set axes color to black
        
        % Plot mode numbers
        subplot(2, 1, 1);
        plot(logs(fileIdx).MODE.TimeS, logs(fileIdx).MODE.ModeNum, 'k*');
        ylabel(logs(fileIdx).getLabel('MODE/ModeNum'));
        if ~issorted(logs(fileIdx).MODE.TimeS, 'strictascend')
            disp([char(fileNames{fileIdx}) ' MODE.TimeS is not monotonic!'])
        end
        xlim([logs(fileIdx).MODE.TimeS(1) logs(fileIdx).PIDR.TimeS(end)]);
        ylim([min(logs(fileIdx).MODE.ModeNum)-1 max(logs(fileIdx).MODE.ModeNum)+1]);
        title(['Other Modes' fileNames{fileIdx}], 'Interpreter', 'none');
        
        % Plot measured z position
        subplot(2,1,2);
        plot(logs(fileIdx).PSCD.TimeS, -1*(logs(fileIdx).PSCD.PD), 'b.'); % -1 because of inverted logging of PSCD.PD
        ylabel('z-Position (m)');
        if ~issorted(logs(fileIdx).PSCD.TimeS, 'strictascend')
            disp([char(fileNames{fileIdx}) ' PSCD.TimeS is not monotonic!'])
        end
        % Plot subflight numbers
        hold on;
        yyaxis right;
        TimeS = logs(fileIdx).MODE.TimeS;
        subflightNr = 1:length(logs(fileIdx).MODE.Mode);
        plot(TimeS, subflightNr, 'k*');
        ylabel('subflight');
        xlim([logs(fileIdx).MODE.TimeS(1) logs(fileIdx).PIDR.TimeS(end)]);
        ylim([0 subflightNr(end)+1]);
        hold off;
    end
    
    % Plot for x-y position controller
    if ~isempty(logs(fileIdx).PSCN.TimeS) && ~isempty(logs(fileIdx).PSCE.TimeS)
        fig = figure;
        set(fig, 'defaultAxesColorOrder', [[0 0 0]; [0 0 0]]); % Set axes color to black
        
        % Plot mode numbers
        subplot(2, 1, 1);
        plot(logs(fileIdx).MODE.TimeS, logs(fileIdx).MODE.ModeNum, 'k*');
        ylabel(logs(fileIdx).getLabel('MODE/ModeNum'));
        if ~issorted(logs(fileIdx).MODE.TimeS, 'strictascend')
            disp([char(fileNames{fileIdx}) ' MODE.TimeS is not monotonic!'])
        end
        xlim([logs(fileIdx).MODE.TimeS(1) logs(fileIdx).PIDR.TimeS(end)]);
        ylim([min(logs(fileIdx).MODE.ModeNum)-1 max(logs(fileIdx).MODE.ModeNum)+1]);
        title(['Other Modes' fileNames{fileIdx}], 'Interpreter', 'none');
        
        % Plot measured x and y position
        subplot(2,1,2);
        plot(logs(fileIdx).PSCN.TimeS, logs(fileIdx).PSCN.PN, 'r.');
        ylabel('Position (m)');
        if ~issorted(logs(fileIdx).PSCN.TimeS, 'strictascend')
            disp([char(fileNames{fileIdx}) ' PSCN.TimeS is not monotonic!'])
        end
        hold on;
        plot(logs(fileIdx).PSCE.TimeS, logs(fileIdx).PSCE.PE', 'b.');
        if ~issorted(logs(fileIdx).PSCE.TimeS, 'strictascend')
            disp([char(fileNames{fileIdx}) ' PSCE.TimeS is not monotonic!'])
        end
        
        % Plot subflight numbers
        yyaxis right;
        TimeS = logs(fileIdx).MODE.TimeS;
        subflightNr = 1:length(logs(fileIdx).MODE.Mode);
        plot(TimeS, subflightNr, 'k*');
        ylabel('subflight');
        xlim([logs(fileIdx).MODE.TimeS(1) logs(fileIdx).PIDR.TimeS(end)]);
        ylim([0 subflightNr(end)+1]);
        hold off;
        legend('x-Position', 'y-Position', 'Subflight') % PSCN is x, PSCE is y
    end
    
    % This aids the users configuring the subflights
    disp('Tested Flight Modes:');
    disp('   flight  filename');
    disp(['     '  num2str(fileIdx) '    ' char(fileNames{fileIdx})]);
    fprintf('Subflight\tMode\tSID_AXIS\n');
    j = 1;
    for i=1:length(logs(fileIdx).MODE.Mode)
        if logs(fileIdx).MODE.Mode(i) == 25
            % If mode is sysid, print tested axis
            
            % Decrement sysid mode index j if the start time of
            % sysid mode was not tracked (and does not equal the mode
            % time stamps as a result). If it was not tracked, the
            % settings are identical to the sysid mode before and were
            % not changed.
            if abs(logs(fileIdx).MODE.TimeS(i)-logs(fileIdx).SIDS.TimeS(j)) > 0.01 && j > 1
                j = j-1;
            end
            
            fprintf('\t%d\t\t%d\t\t%d\n', i, logs(fileIdx).MODE.Mode(i), logs(fileIdx).SIDS.Ax(j));
            
            % Increment sysid mode index j only if the start time was
            % tracked (and equals the mode time stamps as a result).
            % Otherwise, the tracked sysid information belongs to the
            % next sysid flight and is equal to the current, so that
            % the index should not be changed to derive the same infos.
            if abs(logs(fileIdx).MODE.TimeS(i)-logs(fileIdx).SIDS.TimeS(j)) < 0.01
                j = j+1;
            end
        else
            % If not, just print mode
            fprintf('\t%d\t\t%d\t\t-\n', i, logs(fileIdx).MODE.Mode(i));
        end
    end
    
    % Let the user select the subflight if BIN files are selected
    iSubflights = 1;
    while(true)
        if chooseSubflights(fileIdx) 
            nextSubflight = input('Insert number of subflight to be saved (skip with 0): ');
            % Repeat loop if input is empty
            if isempty(nextSubflight)
                continue;
            end
            % Leave loop when nextSubflight is zero
            if nextSubflight == 0
                fprintf(['Subflight selection for flight ' num2str(fileIdx) ' done.\n']);
                break;
            end
            % Give warnings when given number exceeds number of
            % subflights...
            if nextSubflight > length(logs(fileIdx).MODE.Mode)
                warning('Given number exceeds number of subflights!');
                continue
            end
            % ...or when subflight was selected twice for the same flight
            if any(nextSubflight == subflights(numSubflights+1:end))
                warning('Subflight already selected!');
                continue
            end
        
        else
            % Break if number of added subflights equals the total number
            % of loaded subflights for that file (represented by appearence
            % of the respective file index in fileIndices)
            if numel(subflights)-numSubflights >= sum(loadedDataConfig.fileIndices == fileIdx) && subflights ~= 0
                break;
            end
            % mat file was chosen - read subflight from config
            nextSubflight = loadedDataConfig.subflights(numSubflights+iSubflights);
        end
        
        % Add subflight, file index and mode
        subflights(numSubflights+iSubflights) = nextSubflight;
        fileIndices(numSubflights+iSubflights) = fileIdx;
        modes(numSubflights+iSubflights) = logs(fileIdx).MODE.Mode(nextSubflight);
        modesConfig(numSubflights+iSubflights) = ModeConfig(modes(end));
        iSubflights = iSubflights + 1; % Update subflight index
        
        % Break loop if number of selected subflights equals number of
        % modes
        if length(logs(fileIdx).MODE.Mode) < iSubflights
            break;
        end
    end
    % Update amount of subflights only if 0 was not chosen as subflight
    if ~any(subflights == 0)
        numSubflights = length(subflights); 
    end
end
clear file ax TimeS subflightNr fig nextSubflight iSubflights numFiles fileIdx fileExt
clear flightDataSearchPath
%% Slice logs to desired subflights 

% Time increment that is used to extend the slicing time interval to 
% prevent signals (that are only defined at the first time step) to be 
% excluded from the sliced log because of floating point precision
dt_ext = 0.001;

% Create list for axis that were tested (axis=0 if sysid was not used)
axisList = 0;

% ---------------------------------------------- %
%                                                %
% Extract relevant log messages and slice logs   %  
%                                                %   
% ---------------------------------------------- %

% slice the system identification flight subsection(s)
fprintf('\n-------------------------------------------\n')
fprintf('Extracting messages and slicing logs...\n');
fprintf('-------------------------------------------\n');
% Delete sid in case of reloading for new config
clear sid
for i = 1:length(fileIndices)
    fprintf(['Subflight Nr. ' num2str(i) '\n']);
    % Check if current subflight number doesn't exceed actual subflights
    if (subflights(i) > length(logs(fileIndices(i)).MODE.Mode) )
        error(['Defined subflight (index ' num2str(i) ') exceeds amount of actual subflights!']);
    end
    
    % Filter log messages
    logFilterMsgs = {};
    logFilterMsgs = modesConfig(i).filter_msgs;
    % Check if log_filter_msgs is not empty
    if ~iscellstr(logFilterMsgs)
        error('logFilterMsgs variable is not a string cell array');
    end
    % Check if message is included in flight log
    for msg = logFilterMsgs
        % Skip message if it was not defined - delete it from array
        if ~isprop(logs(fileIndices(i)), msg{1})
            warning(['Message ' msg{1} ' not included in subflight ' ...
                num2str(subflights(i)) ' of logfile ' ...
                fileNames{fileIndices(i)} '!'])
        elseif isempty(logs(fileIndices(i)).(msg{1}).LineNo)
            warning(['Message ' msg{1} ' not included in subflight ' ...
                num2str(subflights(i)) ' of logfile ' ...
                fileNames{fileIndices(i)} '!']);
            logFilterMsgs(strcmp(logFilterMsgs, msg{1})) = [];
        end
    end
    log = logs(fileIndices(i)).filterMsgs(logFilterMsgs);
    
    % Slice log to desired time section
    if subflights(i) < length(logs(fileIndices(i)).MODE.Mode)
        % Current subflight is not last subflight of log
        start_time = logs(fileIndices(i)).MODE.TimeUS(subflights(i))/1e6;
        % end time: Subtract 2*dt_ext to prevent first time step of
        % next flight mode to be part of sliced log
        end_time = logs(fileIndices(i)).MODE.TimeUS(subflights(i)+1)/1e6-2*dt_ext;
    else % use end time of TimeUS message in case of the last subflight
        start_time = logs(fileIndices(i)).MODE.TimeUS(subflights(i))/1e6;
        end_time = logs(fileIndices(i)).RATE.TimeUS(end)/1e6;
    end
    log = log.getSlice( [start_time-dt_ext end_time+dt_ext], 'TimeS');
    
    % Check if flight mode of log corresponds to defined mode in config
    if modes(i) ~= log.MODE.Mode
        error_msg = ['Defined Mode (' num2str(modes(i)) ') in ' ...
            'mode_list with index ' num2str(i) ' does not correspond '...
            'to mode of log (' num2str(log.MODE.Mode) ')!'];
        error(error_msg);
    end
    
    % preserve some messages that are outside of the slice time interval
    log.FMT = logs(fileIndices(i)).FMT;
    log.UNIT = logs(fileIndices(i)).UNIT;
    log.FMTU = logs(fileIndices(i)).FMTU;
    log.MULT = logs(fileIndices(i)).MULT;
    log.PARM = logs(fileIndices(i)).PARM;
    % Update the number of actual included messages
    log.numMsgs = 0;
    for msg = log.msgsContained
        msgName = char(msg);
        % Check if message has been sliced off log
        if ~isvalid(log.(msgName))
            warning(['Message ' msg{1} ' in log ' ...
                num2str(i) ' was sliced off message!']);
        else
            log.numMsgs = log.numMsgs + length(log.(msgName).LineNo);
        end
    end
    
    % Get axis from flight. Set to 0 if sysid mode is not active.
    axis = get_axis(log);
    axis = axis(1);
    % Add new axis.
    axisList(i) = axis;
    
    % Resample log and save it to sid array
    sidLogs(numOldSubflights + i).data = resampleLog(log, true, 'linear');
end

% Save identification data in IdData objects
% Index to save IO data of SID runs
j = 1;
for i=1:length(sidLogs)-numOldSubflights
    if sidLogs(numOldSubflights + i).data.MODE.Mode == 25
        % Let user insert minimum and maximum frequencies for spectral
        % analysis
        disp('Insert minimum and maximum frequency for spectral analysis (enter no value to choose sweep values):')
        fminSweep = sidLogs(numOldSubflights + i).data.SIDS.FSt;
        fmaxSweep = sidLogs(numOldSubflights + i).data.SIDS.FSp;
        fmin = input(['Minimum frequency (sweep = ' num2str(fminSweep) '): fmin = ']);
        fmax = input(['Maximum frequency (sweep = ' num2str(fmaxSweep) '): fmax = ']);
        if isempty(fmin) fmin = fminSweep; end
        if isempty(fmax) fmax = fmaxSweep; end
        idData(numOldSysidflights + j) = IdData(sidLogs(numOldSubflights + i).data, subflights(i), i, fmin, fmax);
        
        % Increment IO data index for later saving
        j = j+1;
    end
end

% Print modes and/or tested sid axis in log array
fprintf('\nChosen test flights:\n')
fprintf('Dataset\t|File\t|Subflight\t|Mode\t|Axis\t|Id Quality\n')
for i=1:length(sidLogs)-numOldSubflights
    if (isfield(sidLogs(numOldSubflights + i).data, 'SIDS'))
        % Find correct idData object for data quality
        idDataObj = findobj(idData, '-function', ...
            @(obj) strcmp(obj.fileName, fileNames{fileIndices(i)}) && obj.dataIndex == i);
        % When identical objects are contained in idData vector, use latest
        idDataObj = idDataObj(end);
        fprintf('sid(%d)\t|%d\t\t|%d\t\t\t|%d\t\t|%d\t\t|%.2f\t\n', int16(i+numOldSubflights), ...
            fileIndices(i), subflights(i), sidLogs(numOldSubflights + i).data.MODE.Mode(1), ...
            sidLogs(numOldSubflights + i).data.SIDS.Ax, idDataObj.dataQuality);
    else
        fprintf('sid(%d)\t|%d\t\t|%d\t\t\t|%d\t\t|-\t\t|-\n', int16(i+numOldSubflights), ...
            fileIndices(i), subflights(i), sidLogs(numOldSubflights + i).data.MODE.Mode(1));
    end
end

% Save configuration of loaded data to struct and to repo
if numOldSubflights == 0
    % No configuration in workspace yet
    loadedDataConfig.fileNames = fileNames;
    loadedDataConfig.fileIndices = fileIndices;
    loadedDataConfig.subflights = subflights;
    loadedDataConfig.modes = modes;
    loadedDataConfig.modesConfig = modesConfig;
    loadedDataConfig.axisList = axisList;
else
    % Add new configuration to existing configuration array
    for i=1:length(fileNames)
        loadedDataConfig.fileNames{end+1} = fileNames{i};
    end
    for i=1:length(fileIndices)
        % File indices must be adjusted to index of file in existing config
        loadedDataConfig.fileIndices(numOldSubflights+i) = length(loadedDataConfig.fileNames);
        loadedDataConfig.subflights(numOldSubflights+i) = subflights(i);
        loadedDataConfig.modes(numOldSubflights+i) = modes(i);
        loadedDataConfig.modesConfig(numOldSubflights+i) = modesConfig(i);
        loadedDataConfig.axisList(numOldSubflights+i) = axisList(i);
    end
end
loadedDataConfigFile = [flightDataPath 'dataLoadConfig.mat'];
save(loadedDataConfigFile, 'loadedDataConfig');

% Save new flightdata to repo
newDataFile = 'sid.mat';
newDataFile = [flightDataPath newDataFile];
if exist(newDataFile, 'file')
    [filename, path] = uiputfile({'*.mat','Mat file (*.mat)';'*.*','All files (*.*)'}, 'Save File Name', ...
        newDataFile);
    % Set user-specified file name
    newDataFile = [flightDataPath filename];
end
if newDataFile ~= 0
    if exist('idData','var')
        save(newDataFile, 'sidLogs', 'idData', 'loadedDataConfig');
    else
        save(newDataFile, 'sidLogs', 'loadedDataConfig');
    end
end

clear msg msgName filename path in_dat out_dat len idd delta_T k
clear start_time end_time sid idDataObj minNumElem
clear newDataFile logPathName numOldSubflights numOldSysidflights
clear fmin fminSweep fmax fmaxSweep addData logs

clear i j dt_ext log_filenames axis axisList files chooseSubflights
clear numSubflights storedFile logFilterMsgs fileNames fileIndices subflights
clear dataLoadConfigFile dataStored loadedDataConfigFile modes modesConfig

% Interact with user to continue
uiwait(msgbox(['Data loaded. Press ok to close figures and continue.'],'Data loaded','none','nonmodal'));
close all
%% Functions
% Function to read tested axis from log
function axis = get_axis(obj)
    if isprop(obj, 'SIDS')
        axis = obj.SIDS.Ax;
    else
        % Set to zero if sysid mode is not active
        axis = 0;
    end
end