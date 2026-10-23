function dcm = dcmFromEuler(roll, pitch, yaw)
% Function to calculate the DCM from Euler Angles.
% Euler Angles are given in radian.
% Taken from Matrix3<T>::from_euler()
sr = sin(roll);
cr = cos(roll);
sp = sin(pitch);
cp = cos(pitch);
sy = sin(yaw);
cy = cos(yaw);

dcm = zeros(3,3);
% First row
dcm(1,1) = cp * cy;
dcm(1,2) = (sr * sp * cy) - (cr * sy);
dcm(1,3) = (cr * sp * cy) + (sr * sy);
% Second row
dcm(2,1) = cp * sy;
dcm(2,2) = (sr * sp * sy) + (cr * cy);
dcm(2,3) = (cr * sp * sy) - (sr * cy);
% Third row
dcm(3,1) = -sp;
dcm(3,2) = sr * cp;
dcm(3,3) = cr * cp;


end