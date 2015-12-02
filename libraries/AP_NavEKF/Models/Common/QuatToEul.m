% Convert from a quaternion to a 321 Euler rotation sequence in radians

function Euler = QuatToEul(quat)

Euler = zeros(3,1);

Euler(1) = atan2(2*(quat(3)*quat(4)+quat(1)*quat(2)),  quat(1)*quat(1) - quat(2)*quat(2) - quat(3)*quat(3) + quat(4)*quat(4));
Euler(2) = -asin(2*(quat(2)*quat(4)-quat(1)*quat(3)));
Euler(3) = atan2(2*(quat(2)*quat(3)+quat(1)*quat(4)),  quat(1)*quat(1) + quat(2)*quat(2) - quat(3)*quat(3) - quat(4)*quat(4));