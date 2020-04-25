function [v_foot] = fcn_v_foot(q,p)

v_foot = zeros(3,1);

  v_foot(1,1)=- dq1*(p(3)*sin(q(1) + q(2)) + p(2)*sin(q(1))) - p(3)*dq2*sin(q(1) + q(2));
  v_foot(2,1)=dq1*(p(3)*cos(q(1) + q(2)) + p(2)*cos(q(1))) + p(3)*dq2*cos(q(1) + q(2));
  v_foot(3,1)=0;

 