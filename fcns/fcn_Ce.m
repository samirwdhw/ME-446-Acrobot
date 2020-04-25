function [Ce] = fcn_Ce(q,dq,p)

Ce = zeros(2,2);

  Ce(1,1)=-p(2)*p(5)*dq(2)*(p(3)*sin(q(2)) + p(10)*cos(q(2)) + p(9)*sin(q(2)));
  Ce(1,2)=-p(2)*p(5)*(dq(1) + dq(2))*(p(3)*sin(q(2)) + p(10)*cos(q(2)) + p(9)*sin(q(2)));
  Ce(2,1)=p(2)*p(5)*dq(1)*(p(3)*sin(q(2)) + p(10)*cos(q(2)) + p(9)*sin(q(2)));
  Ce(2,2)=0;

 