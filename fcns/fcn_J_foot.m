function [J_foot] = fcn_J_foot(q,p)

J_foot = zeros(3,2);

  J_foot(1,1)=- p(3)*sin(q(1) + q(2)) - p(2)*sin(q(1));
  J_foot(1,2)=-p(3)*sin(q(1) + q(2));
  J_foot(2,1)=p(3)*cos(q(1) + q(2)) + p(2)*cos(q(1));
  J_foot(2,2)=p(3)*cos(q(1) + q(2));
  J_foot(3,1)=0;
  J_foot(3,2)=0;

 