function [p1] = fcn_p1(q,p)

p1 = zeros(3,1);

  p1(1,1)=p(2)*cos(q(1));
  p1(2,1)=p(2)*sin(q(1));
  p1(3,1)=0;

 