function [data, info] = armMotorsRequest
%ArmMotors gives an empty data for ardupilot_msgs/ArmMotorsRequest
% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'ardupilot_msgs/ArmMotorsRequest';
[data.arm, info.arm] = ros.internal.ros2.messages.ros2.default_type('logical',1,0);
info.MessageType = 'ardupilot_msgs/ArmMotorsRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'arm';
