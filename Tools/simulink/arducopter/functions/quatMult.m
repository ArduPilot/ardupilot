function qRes = quatMult(q1, q2)
% Quaternion multiplication. Code based on QuaternionT<T>::operator*() in
% quaternion.cpp

qRes = single(zeros(4,1));
qRes(1) = q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3) - q1(4)*q2(4);
qRes(2) = q1(1)*q2(2) + q1(2)*q2(1) + q1(3)*q2(4) - q1(4)*q2(3);
qRes(3) = q1(1)*q2(3) - q1(2)*q2(4) + q1(3)*q2(1) + q1(4)*q2(2);
qRes(4) = q1(1)*q2(4) + q1(2)*q2(3) - q1(3)*q2(2) + q1(4)*q2(1);
end