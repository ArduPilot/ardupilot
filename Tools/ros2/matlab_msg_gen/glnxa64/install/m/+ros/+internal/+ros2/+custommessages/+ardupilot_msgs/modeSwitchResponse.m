function [data, info] = modeSwitchResponse
%ModeSwitch gives an empty data for ardupilot_msgs/ModeSwitchResponse
% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'ardupilot_msgs/ModeSwitchResponse';
[data.status, info.status] = ros.internal.ros2.messages.ros2.default_type('logical',1,0);
[data.curr_mode, info.curr_mode] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0);
info.MessageType = 'ardupilot_msgs/ModeSwitchResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'status';
info.MatPath{2} = 'curr_mode';
