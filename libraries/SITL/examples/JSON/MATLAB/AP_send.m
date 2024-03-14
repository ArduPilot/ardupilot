function AP_send(gyro, attitude, accel, velocity, position, rpm, time)
global u
if isempty(u)
   return 
end
persistent past_time
if isempty(past_time)
    past_time = -1;
end
if past_time == time
    error('Send repeat time');
end
past_time = time;

% build structure representing the JSON string to be sent
JSON.timestamp = time;
JSON.imu.gyro = gyro;
JSON.imu.accel_body = accel;
JSON.position = position;
JSON.attitude = attitude;
JSON.velocity = velocity;
JSON.motor.rpm = rpm;

% Report to AP
pnet(u,'printf',sprintf('\n%s\n',jsonencode(JSON)));
pnet(u,'writepacket');

end