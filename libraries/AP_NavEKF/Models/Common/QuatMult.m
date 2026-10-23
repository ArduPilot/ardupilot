function quatOut = QuatMult(quatA,quatB)
% Calculate the following quaternion product quatA * quatB using the 
% standard identity 

quatOut = [quatA(1)*quatB(1)-quatA(2:4)'*quatB(2:4); quatA(1)*quatB(2:4) + quatB(1)*quatA(2:4) + cross(quatA(2:4),quatB(2:4))];