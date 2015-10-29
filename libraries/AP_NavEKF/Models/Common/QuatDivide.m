function q_out = QuatDivide(qin1,qin2)

q0 = qin1(1);
q1 = qin1(2);
q2 = qin1(3);
q3 = qin1(4);

r0 = qin2(1);
r1 = qin2(2);
r2 = qin2(3);
r3 = qin2(4);

q_out(1,1) = (qin2(1)*qin1(1) + qin2(2)*qin1(2) + qin2(3)*qin1(3) + qin2(4)*qin1(4));
q_out(2,1) = (r0*q1 - r1*q0 - r2*q3 + r3*q2);
q_out(3,1) = (r0*q2 + r1*q3 - r2*q0 - r3*q1);
q_out(4,1) = (r0*q3 - r1*q2 + r2*q1 - r3*q0);
