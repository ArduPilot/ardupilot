% This class provides important setting information for the flight modes
% that have been chosen for the SID process.
%
% Fabian Bredemeier - IAV GmbH
% License: GPL v3

classdef ModeConfig 
   properties
       name = '';
       mode_num;
       thr_man;
       filter_msgs = {};
   end
   
   methods 
       % Constructor
       function obj = ModeConfig(mode_num)
           props = ModeConfig.getModeProps(mode_num);
           obj.name = props.name;
           obj.mode_num = mode_num;
           obj.thr_man = props.thr_man;
           obj.filter_msgs = props.filter_msgs;
       end
       
       % Function that returns index of mode properties that corresponds to
       % the mode number
       function modeIndex = getIndexFromModeNum(obj, modeNum)
          for i=1:length(obj.names)
              if (obj.mode_num_list(i) == modeNum)
                 modeIndex = i;
                 break;
              end
          end
       end
       
       function filter_msgs = get_filter_msgs(obj)
          filter_msgs = obj.filter_msgs;
       end
       
       function thr_man = get_thr_man(obj)
          thr_man = obj.thr_man;
       end
       
       function name = get_name(obj)
          name = obj.name;
       end
       
       % Function to compare properties to another ModeConfig object
       function identical = compare_to(obj, obj2)
           % Name
            if strcmp(obj.name, obj2.get_name()) ~= 1
                identical = false;
                return;
            end
            % Throttle Man
            if obj.thr_man ~= obj2.get_thr_man
                identical = false;
                return;
            end
            % Filter message list
            if numel(obj.filter_msgs) ~= numel(obj2.get_filter_msgs())
                identical = false;
                return;
            end
            for i=1:numel(obj.filter_msgs)
                obj2_filter_msgs = obj2.get_filter_msgs();
                if strcmp(obj.filter_msgs{i}, obj2_filter_msgs{i}) ~= 1
                    identical = false;
                    return;
                end
            end
            identical = true;
       end
   end
   
   methods (Static)
       
       % Function that stores all mode properties
       function propsStruct = getModeProps(modeNum)
           % Define basic filter message cell array which is necessary for
           % only the inner loop or the part that is used in SYSID
           % respectively. Used for modes whose relevant msgs are not yet
           % clear.
           filter_msgs_sysid = {'FMT', 'UNIT', 'FMTU', 'MULT', 'PARM', 'MODE', 'SIDD', ...
               'SIDS', 'ATT', 'CTRL', 'RATE', 'PIDR', 'PIDP', 'PIDY', 'CTUN', ...
               'IMU','BAT', 'BARO', 'MOTB'};
           % Filter message cell array which is necessary to run simulation
           % in loiter mode
           filter_msgs_loiter = {'FMT', 'UNIT', 'FMTU', 'MULT', 'PARM', 'MODE', ...
               'ATT', 'CTRL', 'RATE', 'PIDR', 'PIDP', 'PIDY', 'PIDA', 'CTUN', ...
               'IMU', 'BAT', 'BARO', 'MOTB', 'PSCD', 'PSCE', 'PSCN', 'RCIN'};
           % Stabilize
           filter_msgs_stabilize = {'FMT', 'UNIT', 'FMTU', 'MULT', 'PARM', 'MODE', ...
               'ATT', 'CTRL', 'RATE', 'PIDR', 'PIDP', 'PIDY', 'PIDA', 'CTUN', ...
               'IMU', 'BAT', 'BARO', 'MOTB', 'GYR', 'ACC', 'RCIN'};
           % Acro
           filter_msgs_acro = {'FMT', 'UNIT', 'FMTU', 'MULT', 'PARM', 'MODE', ...
               'ATT', 'CTRL', 'RATE', 'PIDR', 'PIDP', 'PIDY', 'PIDA', 'CTUN', ...
               'IMU', 'BAT', 'BARO', 'MOTB', 'GYR', 'ACC'};
           % AltHold
           filter_msgs_althold = {'FMT', 'UNIT', 'FMTU', 'MULT', 'PARM', 'MODE', ...
               'ATT', 'CTRL', 'RATE', 'PIDR', 'PIDP', 'PIDY', 'PIDA', 'CTUN', ...
               'IMU', 'BAT', 'BARO', 'MOTB', 'PSCD', 'GYR', 'ACC', 'RCIN'};
           switch modeNum
               case 0  % Stabilize
                   propsStruct.name = 'Stabilize';
                   propsStruct.thr_man = true;
                   propsStruct.filter_msgs = filter_msgs_stabilize;
               case 1 % Acro
                   propsStruct.name = 'Acro';
                   propsStruct.thr_man = true;
                   propsStruct.filter_msgs = filter_msgs_acro;
               case 2 % Alt_Hold
                   propsStruct.name = 'Alt_Hold';
                   propsStruct.thr_man = false;
                   propsStruct.filter_msgs = filter_msgs_althold;
               case 3 % Auto
                   propsStruct.name = 'Auto';
                   propsStruct.thr_man = false;
                   propsStruct.filter_msgs = filter_msgs_loiter;
               case 4 % Guided
                   propsStruct.name = 'Guided';
                   propsStruct.thr_man = false;
                   propsStruct.filter_msgs = filter_msgs_loiter;
               case 5 % Loiter
                   propsStruct.name = 'Loiter';
                   propsStruct.thr_man = false;
                   propsStruct.filter_msgs = filter_msgs_loiter;
               case 6 % RTL
                   propsStruct.name = 'RTL';
                   propsStruct.thr_man = false;
                   propsStruct.filter_msgs = filter_msgs_sysid;
               case 7 % Circle
                   propsStruct.name = 'Circle';
                   propsStruct.thr_man = false;
                   propsStruct.filter_msgs = filter_msgs_sysid;
               case 9 % Land
                   propsStruct.name = 'Land';
                   propsStruct.thr_man = false;
                   propsStruct.filter_msgs = filter_msgs_sysid;
               case 11 % Drift
                   propsStruct.name = 'Drift';
                   propsStruct.thr_man = false;
                   propsStruct.filter_msgs = filter_msgs_sysid;
               case 13 % Sport
                   propsStruct.name = 'Sport';
                   propsStruct.thr_man = false;
                   propsStruct.filter_msgs = filter_msgs_sysid;
               case 14 % Flip
                   propsStruct.name = 'Flip';
                   propsStruct.thr_man = false;
                   propsStruct.filter_msgs = filter_msgs_sysid;
               case 15 % Autotune
                   propsStruct.name = 'Autotune';
                   propsStruct.thr_man = false;
                   propsStruct.filter_msgs = filter_msgs_stabilize;
               case 16 % Poshold
                   propsStruct.name = 'Poshold';
                   propsStruct.thr_man = false;
                   propsStruct.filter_msgs = filter_msgs_sysid;
               case 17 % Brake
                   propsStruct.name = 'Brake';
                   propsStruct.thr_man = false;
                   propsStruct.filter_msgs = filter_msgs_sysid;
               case 18 % Throw
                   propsStruct.name = 'Throw';
                   propsStruct.thr_man = false;
                   propsStruct.filter_msgs = filter_msgs_sysid;
               case 19 % Avoid_ADSB
                   propsStruct.name = 'Avoid_ADSB';
                   propsStruct.thr_man = false;
                   propsStruct.filter_msgs = filter_msgs_sysid;
               case 20 % Guided_NoGPS
                   propsStruct.name = 'Guided_NoGPS';
                   propsStruct.thr_man = false;
                   propsStruct.filter_msgs = filter_msgs_sysid;
               case 21 % SmartRTL
                   propsStruct.name = 'SmartRTL';
                   propsStruct.thr_man = false;
                   propsStruct.filter_msgs = filter_msgs_sysid;
               case 22 % Flowhold
                   propsStruct.name = 'Flowhold';
                   propsStruct.thr_man = false;
                   propsStruct.filter_msgs = filter_msgs_sysid;
               case 23 % Follow
                   propsStruct.name = 'Follow';
                   propsStruct.thr_man = false;
                   propsStruct.filter_msgs = filter_msgs_sysid;
               case 24 % Zigzag
                   propsStruct.name = 'Zigzag';
                   propsStruct.thr_man = false;
                   propsStruct.filter_msgs = filter_msgs_sysid;
               case 25 % Systemid
                   propsStruct.name = 'Systemid';
                   propsStruct.thr_man = true;
                   propsStruct.filter_msgs = filter_msgs_sysid;
               case 26 % Autorotate
                   propsStruct.name = 'Autorotate';
                   propsStruct.thr_man = false;
                   propsStruct.filter_msgs = filter_msgs_sysid;
           end
       end
    end
end