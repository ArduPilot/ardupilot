function quat = AlignHeading( ...
    quat, ... % quaternion state vector
    magMea, ... % body frame magnetic flux measurements
    declination)  % Estimated magnetic field delination at current location

% Calculate the predicted magnetic declination
Tbn = Quat2Tbn(quat);
magMeasNED = Tbn*magMea;
predDec = atan2(magMeasNED(2),magMeasNED(1));

% Calculate the measurement innovation
innovation = predDec - declination;

if (innovation > pi)
    innovation = innovation - 2*pi;
elseif (innovation < -pi)
    innovation = innovation + 2*pi;
end

% form the NED rotation vector
deltaRotNED = -[0;0;innovation];

% rotate into body axes
% Calculate the body to nav cosine matrix
Tbn = Quat2Tbn(quat);
deltaRotBody = transpose(Tbn)*deltaRotNED;

% Convert the error rotation vector to its equivalent quaternion
% error = truth - estimate
rotationMag = abs(innovation);
if rotationMag<1e-6
    deltaQuat = single([1;0;0;0]);
else
    deltaQuat = [cos(0.5*rotationMag); [deltaRotBody(1);deltaRotBody(2);deltaRotBody(3)]/rotationMag*sin(0.5*rotationMag)];
end

% Update the quaternion states by rotating from the previous attitude through
% the delta angle rotation quaternion
quat = [quat(1)*deltaQuat(1)-transpose(quat(2:4))*deltaQuat(2:4); quat(1)*deltaQuat(2:4) + deltaQuat(1)*quat(2:4) + cross(quat(2:4),deltaQuat(2:4))];

% normalise the updated quaternion states
quatMag = sqrt(quat(1)^2 + quat(2)^2 + quat(3)^2 + quat(4)^2);
if (quatMag > 1e-12)
    quat = quat / quatMag;
end

end