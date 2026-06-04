function [data, info] = modeSwitchRequest
%ModeSwitch gives an empty data for ardupilot_msgs/ModeSwitchRequest
% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'ardupilot_msgs/ModeSwitchRequest';
[data.mode, info.mode] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0);
info.MessageType = 'ardupilot_msgs/ModeSwitchRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'mode';
