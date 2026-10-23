function q = fromAxisAngle(vec)
% Create a quaternion from its axis-angle representation. Just the rotation
% vector is given as input.
% Based on QuaternionT<T>::from_axis_angle(Vector3<T>)

q = zeros(4,1);
angle = single(sqrt(vec(1)^2 + vec(2)^2 + vec(3)^2));

if angle == 0
    q(1) = single(1);
    q(2) = single(0);
    q(3) = single(0);
    q(4) = single(0);
else
    axis = vec ./ angle;
    % The following lines are based on QuaternionT<T>::from_axis_angle(const Vector3<T> &axis, T theta)
    if angle == 0
        q(1) = single(1);
        q(2) = single(0);
        q(3) = single(0);
        q(4) = single(0);
    else
        st2 = single(sin(0.5*angle));
        q(1) = single(cos(0.5*angle));
        q(2) = axis(1) * st2;
        q(3) = axis(2) * st2;
        q(4) = axis(3) * st2;
    end
end