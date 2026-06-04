function [data, info] = airspeed
%Airspeed gives an empty data for ardupilot_msgs/Airspeed
% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'ardupilot_msgs/Airspeed';
[data.header, info.header] = ros.internal.ros2.custommessages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.true_airspeed, info.true_airspeed] = ros.internal.ros2.custommessages.geometry_msgs.vector3;
info.true_airspeed.MLdataType = 'struct';
[data.eas_2_tas, info.eas_2_tas] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
info.MessageType = 'ardupilot_msgs/Airspeed';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'true_airspeed';
info.MatPath{7} = 'true_airspeed.x';
info.MatPath{8} = 'true_airspeed.y';
info.MatPath{9} = 'true_airspeed.z';
info.MatPath{10} = 'eas_2_tas';
