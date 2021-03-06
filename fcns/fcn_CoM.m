function [CoM] = fcn_CoM(q,p)

CoM = zeros(3,1);

  CoM(1,1)=(p(5)*(p(9)*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - p(10)*(cos(q(1))*sin(q(2)) +...
          cos(q(2))*sin(q(1))) + p(2)*cos(q(1)) + p(3)*cos(q(1))*cos(q(2)) - p(3)*sin(q(1))*sin(q(2))) + p(4)*(p(2)*...
         cos(q(1)) + p(6)*cos(q(1)) - p(7)*sin(q(1))))/(p(4) + p(5));
  CoM(2,1)=(p(5)*(p(9)*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + p(10)*(cos(q(1))*...
         cos(q(2)) - sin(q(1))*sin(q(2))) + p(2)*sin(q(1)) + p(3)*cos(q(1))*sin(q(2)) + p(3)*cos(q(2))*sin(q(1))) + p(4)*(p(2)*...
         sin(q(1)) + p(7)*cos(q(1)) + p(6)*sin(q(1))))/(p(4) + p(5));
  CoM(3,1)=(p(4)*p(8) + p(5)*p(11))/(p(4) + p(5));

 