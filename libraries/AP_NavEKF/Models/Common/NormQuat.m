% normalise the quaternion
function quaternion = normQuat(quaternion)

quatMag = sqrt(quaternion(1)^2 + quaternion(2)^2 + quaternion(3)^2 + quaternion(4)^2);
quaternion(1:4) = quaternion / quatMag;
