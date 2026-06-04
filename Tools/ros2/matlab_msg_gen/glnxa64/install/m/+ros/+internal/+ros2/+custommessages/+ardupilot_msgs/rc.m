function [data, info] = rc
%Rc gives an empty data for ardupilot_msgs/Rc
% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'ardupilot_msgs/Rc';
[data.header, info.header] = ros.internal.ros2.custommessages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.is_connected, info.is_connected] = ros.internal.ros2.messages.ros2.default_type('logical',1,0);
[data.receiver_rssi, info.receiver_rssi] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0);
[data.channels, info.channels] = ros.internal.ros2.messages.ros2.default_type('int16',32,1);
[data.active_overrides, info.active_overrides] = ros.internal.ros2.messages.ros2.default_type('logical',32,1);
info.MessageType = 'ardupilot_msgs/Rc';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'is_connected';
info.MatPath{7} = 'receiver_rssi';
info.MatPath{8} = 'channels';
info.MatPath{9} = 'active_overrides';
