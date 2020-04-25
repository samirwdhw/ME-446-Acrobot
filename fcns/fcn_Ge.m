function [Ge] = fcn_Ge(q,p)

Ge = zeros(2,1);

  Ge(1,1)=p(5)*p(1)*(p(3)*cos(q(1) + q(2)) + p(9)*cos(q(1) + q(2)) + p(2)*cos(q(1)) - p(10)*...
         sin(q(1) + q(2))) + p(4)*p(1)*(p(2)*cos(q(1)) + p(6)*cos(q(1)) - p(7)*sin(q(1)));
  Ge(2,1)=p(5)*p(1)*(p(3)*cos(q(1) + q(2)) + p(9)*cos(q(1) + q(2)) - p(10)*sin(q(1) + q(2)));

 