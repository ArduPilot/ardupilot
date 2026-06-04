function [data, info] = takeoffRequest
%Takeoff gives an empty data for ardupilot_msgs/TakeoffRequest
% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'ardupilot_msgs/TakeoffRequest';
[data.alt, info.alt] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
info.MessageType = 'ardupilot_msgs/TakeoffRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'alt';
