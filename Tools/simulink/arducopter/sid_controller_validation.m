% Validation of the controller model
%
% The script initializes and runs the Simulink model in controller
% validation mode and validates the controller model. 
%
% The user has to specify the controller that is to be validated (simMode) 
% and the index of the logfile in "sidLogs" (log_idx) that is used for the 
% validation.
%
% Fabian Bredemeier - IAV GmbH
% License: GPL v3

curPath = pwd;

%% Define log file index and controller type to be validated
% Log file index used for validation
log_idx = 2;

% Check validity of log_idx
if log_idx > numel(sidLogs)
    error(['Defined log_idx exceeds number of elements in sidLogs (' num2str(numel(sidLogs)) ')!']);
end

% Show figure with flight paths for user to decide which controller to
% validate
figure % Attitude controller signals
subplot(311)
plot(sidLogs(log_idx).data.RATE.TimeS, sidLogs(log_idx).data.RATE.RDes, 'LineWidth', 1.4);
hold on
plot(sidLogs(log_idx).data.RATE.TimeS, sidLogs(log_idx).data.RATE.R, 'LineWidth', 1.4);
hold off
grid on
legend('Desired', 'Actual');
xlabel('Time (s)');
ylabel('Roll angle (deg)');
xlim([0 max(sidLogs(log_idx).data.RATE.TimeS)]);
subplot(312)
plot(sidLogs(log_idx).data.RATE.TimeS, sidLogs(log_idx).data.RATE.PDes, 'LineWidth', 1.4);
hold on
plot(sidLogs(log_idx).data.RATE.TimeS, sidLogs(log_idx).data.RATE.P, 'LineWidth', 1.4);
hold off
grid on
legend('Desired', 'Actual');
xlabel('Time (s)');
ylabel('Pitch angle (deg)');
xlim([0 max(sidLogs(log_idx).data.RATE.TimeS)]);
subplot(313)
plot(sidLogs(log_idx).data.RATE.TimeS, sidLogs(log_idx).data.RATE.YDes, 'LineWidth', 1.4);
hold on
plot(sidLogs(log_idx).data.RATE.TimeS, sidLogs(log_idx).data.RATE.Y, 'LineWidth', 1.4);
hold off
grid on
legend('Desired', 'Actual');
xlabel('Time (s)');
ylabel('Yaw angle (deg)');
xlim([0 max(sidLogs(log_idx).data.RATE.TimeS)]);

% Plot z Controller signals if Althold flight mode was active
if sidLogs(log_idx).data.MODE.Mode == 2
    figure
    plot(sidLogs(log_idx).data.PSCD.TimeS, -1*(sidLogs(log_idx).data.PSCD.TPD), 'LineWidth', 1.4); % -1 to yield upwards position
    hold on
    plot(sidLogs(log_idx).data.PSCD.TimeS, -1*(sidLogs(log_idx).data.PSCD.PD), 'LineWidth', 1.4);
    hold off
    grid on
    legend('Desired', 'Actual');
    xlabel('Time (s)');
    ylabel('z Position (m)');
    xlim([0 max(sidLogs(log_idx).data.RATE.TimeS)]);
end

% Ask user for controller to validate
% 1 = Rate controller
% 2 = Attitude controller
% 3 = Position controller
fprintf('Flight mode in log file: ');
fprintf('%d', sidLogs(log_idx).data.MODE.Mode);
switch sidLogs(log_idx).data.MODE.Mode
    case 0
        fprintf(' (Stabilize)\n');
    case 2
        fprintf(' (Althold)\n');
end
fprintf('Available controllers that can be validated: \n');
fprintf('1 = Rate controller\n');
fprintf('2 = Attitude controller\n');
fprintf('3 = z Position controller\n');
fprintf('0 = Abort.\n');
simMode = input(['Enter controller to be validated: ']);
switch simMode
    case 0
        fprintf('Aborting...\n');
        close all
        return;
    case 1
        ctrlName = 'rateCtrl';
    case 2
        ctrlName = 'attCtrl';
    case 3
        ctrlName = 'zPosCtrl';
end

% Define title of validation for documentation purposes
valTitle = 'Arducopter-4.3';

%% Configuration of validation - Get signal names for comparison
% Declaration of temporary evaluation structs
val_out_sig_names = {};
val_out_sig_sim = {};
val_out_sig_meas = {};
val_in_sig_names = {};
val_in_sig = {};

% Call the config script to read the necessary signals for the evaluation
% and the names of their counterparts in the Ardupilot code
sid_controller_validation_cfg

%% Run simulation
% Sim init script
sid_sim_init

% Run simulation
simOut = sim("arducopter");

%% Store results

% Create directory for storing the results for the controller validation
% if we are not currently in the result folder...
if isempty(regexp(curPath, 'results'))
    % ...AND if the folder is not already existing on the current path
    if ~exist('results', 'file')
        mkdir results
        cd results
    else 
        cd results
    end
end
if isempty(regexp(curPath, 'ctrlVal'))
    if ~exist('ctrlVal', 'file')
        mkdir ctrlVal
        cd ctrlVal
    else
        cd ctrlVal
    end
end

% Get number of existing subdirectories with today's date and create new 
% subdirectory with the validation title and controller that is validated
date = datestr(now, 'yy_mm_dd');
numFolders = 0;
while true 
    if numFolders < 10
        dir_name = [date '_0' num2str(numFolders) '_' valTitle '_' ctrlName];
        if ~(isfolder(dir_name))
            mkdir(dir_name);
            break;
        end
    else
        dir_name = [date '_' num2str(numFolders) '_' valTitle '_' ctrlName];
        if ~(isfolder(dir_name))
            mkdir(dir_name);
            break;
        end
    end
   numFolders = numFolders + 1; 
end

% Open or create log
log_id = fopen('result_log.txt','a+');
fprintf(log_id, 'Validation %s\n', dir_name); % Print directory to safe figures
fprintf(log_id, 'Flight data path: %s\n', sidLogs(log_idx).data.filePathName); % Print flight data path
fprintf(log_id, 'Flight data file: %s\n', sidLogs(log_idx).data.fileName); % Print flight data file name
fprintf(log_id, 'Subflight: %d\n', loadedDataConfig.subflights(log_idx));
header_mark = '---------------------------------------------------------------\n';
fprintf(log_id, header_mark);
table_header = '%s\t\t\t|%s\n';
fprintf(log_id, table_header, 'Signal', 'Fit between measured and simulated signal (in %)');
fprintf(log_id, header_mark);
fclose(log_id);

% Calculate evaluation struct & save results
for i=1:length(val_out_sig_names)
    % Get SimulationData object
    out_sim = simOut.logsout.get(val_out_sig_sim{i});
    out_meas = simOut.logsout.get(val_out_sig_meas{i});
    out_name = val_out_sig_names{i};
    in_test = simOut.logsout.get(val_in_sig{i});
    
    % Create figure of results
    figure('Name', out_name);
    plot(out_sim.Values.time, out_sim.Values.Data);
    hold on
    plot(out_meas.Values.time, out_meas.Values.Data);
    hold off
    ylabel(out_name)
    
    yyaxis right
    plot(in_test.Values.time, in_test.Values.Data);
    yAxR = gca;
    ylabel(val_in_sig_names{i});
    yAxR.YLim = [min(in_test.Values.Data)-min(in_test.Values.Data)*0.05, max(in_test.Values.Data)+max(in_test.Values.Data)*0.05];
    
    % Figure settings
    xlim([0 out_sim.Values.time(end)]); % Limit time
    grid on % Activate grid of axes
    title([out_name ': Comparison between simulated and measured controller outputs.'], 'Interpreter', 'none');
    legend('Simulated Output', 'Measured Output', 'Input');
    
    % Save figure
    cd(dir_name)
    savefig([out_name '.fig'])
    cd ..
    close all
    
    % Calculate coefficient of determination to determine fit between
    % measured and simulated signal
    sigFit = 1 - sum((out_meas.Values.Data - out_sim.Values.Data).^2)/sum(out_meas.Values.Data.^2);
    
    % Write log
    log_line = '%s\t\t| %6.3f \n';
    log_id = fopen('result_log.txt','a');
    fprintf(log_id, log_line, out_name, sigFit*100);
    fclose(log_id);
end

log_id = fopen('result_log.txt','a');
fprintf(log_id, '\n');
fclose(log_id);

% Return to main directory
cd(curPath);

clear val_in_sig val_in_sig_names val_out_sig_meas val_out_sig_sim val_out_sig_names
clear valTitle sigFit sig_name out_name sig_meas sig_test test_sig test_sig_names
clear table_header f_name num_folders file_struct header header_mark
clear log_id log_line num_folders sig_name_ap sig_sim sig_real signal_log underscore
clear date i dir_name out out_meas out_sim in_test yAxR ctrlName