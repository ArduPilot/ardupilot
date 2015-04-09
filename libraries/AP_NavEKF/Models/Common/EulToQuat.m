function quaterion = EulToQuat(Euler)

% Convert from a 321 Euler rotation sequence specified in radians to a
% Quaternion

quaterion = zeros(4,1);

Euler    = Euler * 0.5;
cosPhi   = cos(Euler(1));
sinPhi   = sin(Euler(1));
cosTheta = cos(Euler(2));
sinTheta = sin(Euler(2));
cosPsi   = cos(Euler(3));
sinPsi   = sin(Euler(3));

quaterion(1,1) = (cosPhi*cosTheta*cosPsi + sinPhi*sinTheta*sinPsi);
quaterion(2,1) = (sinPhi*cosTheta*cosPsi - cosPhi*sinTheta*sinPsi);
quaterion(3,1) = (cosPhi*sinTheta*cosPsi + sinPhi*cosTheta*sinPsi);
quaterion(4,1) = (cosPhi*cosTheta*sinPsi - sinPhi*sinTheta*cosPsi);

return;

   
