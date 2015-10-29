function quat = AlignTilt( ...
    quat, ... % quaternion state vector
    initAccel)  % initial accelerometer vector
% check length
lengthAccel = sqrt(dot([initAccel(1);initAccel(2);initAccel(3)],[initAccel(1);initAccel(2);initAccel(3)]));
% if length is > 0.7g and < 1.4g initialise tilt using gravity vector
if (lengthAccel > 5 && lengthAccel < 14)
    % calculate length of the tilt vector
    tiltMagnitude = atan2(sqrt(dot([initAccel(1);initAccel(2)],[initAccel(1);initAccel(2)])),-initAccel(3));
    % take the unit cross product of the accel vector and the -Z vector to
    % give the tilt unit vector
    if (tiltMagnitude > 1e-3)
        tiltUnitVec = cross([initAccel(1);initAccel(2);initAccel(3)],[0;0;-1]);
        tiltUnitVec = tiltUnitVec/sqrt(dot(tiltUnitVec,tiltUnitVec));
        tiltVec = tiltMagnitude*tiltUnitVec;
        quat = [cos(0.5*tiltMagnitude); tiltVec/tiltMagnitude*sin(0.5*tiltMagnitude)];
    end
end

end