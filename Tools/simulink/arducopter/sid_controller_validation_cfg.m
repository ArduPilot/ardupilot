% Configuration script for the control model validation
% Determines simulated and measured signals that are compared to validate
% the control model
%
% Fabian Bredemeier - IAV GmbH
% License: GPL v3

switch simMode
    case 1 % Rate Controller
        val_out_sig_names = {'RATE.ROut', 'RATE.POut', 'RATE.YOut'}; % Names of signals in the Ardupilot dataflash logs
        val_out_sig_sim = {'rollRateCtrlOut', 'pitchRateCtrlOut', 'yawRateCtrlOut'}; % Names of the simulated signals
        val_out_sig_meas = {'rollRateTotMeas', 'pitchRateTotMeas', 'yawRateTotMeas'}; % Names of the measured signals
        val_in_sig_names = {'RATE_RDes', 'RATE_PDes', 'RATE_YDes'};
        val_in_sig = {'<rollRateTar>', '<pitchRateTar>', '<yawRateTar>'};
    case 2 % Attitude controller
        val_out_sig_names = {'RATE.RDes','RATE.PDes', 'RATE.YDes'};
        val_out_sig_sim = {'rollRateTar', 'pitchRateTar', 'yawRateTar'};
        val_out_sig_meas = {'rollRateTarMeas','pitchRateTarMeas', 'yawRateTarMeas'};
        val_in_sig_names = {'RCIN.C1', 'RCIN.C2', 'RCIN.C4'};
        val_in_sig = {'rcinRoll', 'rcinPitch', 'rcinYaw'};
    case 3 % z position controller
        val_out_sig_names = {'CTUN.ThI', 'PSCD.TAD', 'PSCD.TVD'};
        val_out_sig_sim = {'thrOut', 'zAccTar', 'zVelTar'};
        val_out_sig_meas = {'thrInMeas', 'zAccTarMeas', 'zVelTarMeas'};
        val_in_sig_names = {'PSCD.TAD', 'PSCD.TVD', 'PSCD.TPD'};
        val_in_sig = {'zAccTar', 'zVelTar', 'zPosTar'};
end