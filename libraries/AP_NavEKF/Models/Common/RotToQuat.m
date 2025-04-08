% convert froma rotation vector in radians to a quaternion
function quaternion = RotToQuat(rotVec)

vecLength = sqrt(rotVec(1)^2 + rotVec(2)^2 + rotVec(3)^2);

if vecLength < 1e-6
  quaternion = [1;0;0;0];
else
  quaternion = [cos(0.5*vecLength); rotVec/vecLength*sin(0.5*vecLength)];
end