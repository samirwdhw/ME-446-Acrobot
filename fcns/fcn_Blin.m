function [Blin] = fcn_Blin(q,dq,p)

Blin = zeros(4,1);

  Blin(1,1)=0;
  Blin(2,1)=0;
  Blin(3,1)=-(p(20) + p(3)^2*p(5) + p(5)*p(9)^2 + p(5)*p(10)^2 + 2*p(3)*p(5)*p(9) + p(2)*p(3)*p(5)*...
         cos(q(2)) + p(2)*p(5)*p(9)*cos(q(2)) - p(2)*p(5)*p(10)*sin(q(2)))/(p(14)*p(20) + p(2)^2*p(3)^2*p(5)^2 + p(2)^2*...
         p(5)^2*p(9)^2 + p(20)*p(2)^2*p(4) + p(14)*p(3)^2*p(5) + p(20)*p(2)^2*p(5) + p(20)*p(4)*p(6)^2 + p(14)*p(5)*...
         p(9)^2 + p(20)*p(4)*p(7)^2 + p(14)*p(5)*p(10)^2 + p(2)^2*p(3)^2*p(4)*p(5) + 2*p(2)^2*p(3)*p(5)^2*p(9) +...
          p(2)^2*p(4)*p(5)*p(9)^2 + p(3)^2*p(4)*p(5)*p(6)^2 + p(2)^2*p(4)*p(5)*p(10)^2 + p(3)^2*p(4)*p(5)*...
         p(7)^2 - p(2)^2*p(3)^2*p(5)^2*cos(q(2))^2 + p(4)*p(5)*p(6)^2*p(9)^2 + p(4)*p(5)*p(6)^2*p(10)^2 + p(4)*p(5)*p(9)^2*...
         p(7)^2 + p(4)*p(5)*p(7)^2*p(10)^2 + 2*p(20)*p(2)*p(4)*p(6) + 2*p(14)*p(3)*p(5)*p(9) - p(2)^2*p(5)^2*p(9)^2*...
         cos(q(2))^2 + p(2)^2*p(5)^2*p(10)^2*cos(q(2))^2 + 2*p(2)*p(3)^2*p(4)*p(5)*p(6) + 2*p(2)^2*p(3)*p(4)*p(5)*p(9) +...
          2*p(2)*p(4)*p(5)*p(6)*p(9)^2 + 2*p(3)*p(4)*p(5)*p(6)^2*p(9) + 2*p(2)*p(4)*p(5)*p(6)*p(10)^2 + 2*p(3)*...
         p(4)*p(5)*p(9)*p(7)^2 - 2*p(2)^2*p(3)*p(5)^2*p(9)*cos(q(2))^2 + p(2)^2*p(3)*p(5)^2*p(10)*sin(2*q(2)) +...
          p(2)^2*p(5)^2*p(9)*p(10)*sin(2*q(2)) + 4*p(2)*p(3)*p(4)*p(5)*p(6)*p(9));
  Blin(4,1)=(p(14) + p(20) + p(2)^2*p(4) + p(2)^2*p(5) + p(3)^2*p(5) + p(4)*p(6)^2 + p(5)*p(9)^2 +...
          p(4)*p(7)^2 + p(5)*p(10)^2 + 2*p(2)*p(4)*p(6) + 2*p(3)*p(5)*p(9) + 2*p(2)*p(3)*p(5)*cos(q(2)) + 2*p(2)*...
         p(5)*p(9)*cos(q(2)) - 2*p(2)*p(5)*p(10)*sin(q(2)))/(p(14)*p(20) + p(2)^2*p(3)^2*p(5)^2 + p(2)^2*p(5)^2*...
         p(9)^2 + p(20)*p(2)^2*p(4) + p(14)*p(3)^2*p(5) + p(20)*p(2)^2*p(5) + p(20)*p(4)*p(6)^2 + p(14)*p(5)*p(9)^2 +...
          p(20)*p(4)*p(7)^2 + p(14)*p(5)*p(10)^2 + p(2)^2*p(3)^2*p(4)*p(5) + 2*p(2)^2*p(3)*p(5)^2*p(9) + p(2)^2*...
         p(4)*p(5)*p(9)^2 + p(3)^2*p(4)*p(5)*p(6)^2 + p(2)^2*p(4)*p(5)*p(10)^2 + p(3)^2*p(4)*p(5)*p(7)^2 - p(2)^2*...
         p(3)^2*p(5)^2*cos(q(2))^2 + p(4)*p(5)*p(6)^2*p(9)^2 + p(4)*p(5)*p(6)^2*p(10)^2 + p(4)*p(5)*p(9)^2*p(7)^2 +...
          p(4)*p(5)*p(7)^2*p(10)^2 + 2*p(20)*p(2)*p(4)*p(6) + 2*p(14)*p(3)*p(5)*p(9) - p(2)^2*p(5)^2*p(9)^2*...
         cos(q(2))^2 + p(2)^2*p(5)^2*p(10)^2*cos(q(2))^2 + 2*p(2)*p(3)^2*p(4)*p(5)*p(6) + 2*p(2)^2*p(3)*p(4)*p(5)*p(9) +...
          2*p(2)*p(4)*p(5)*p(6)*p(9)^2 + 2*p(3)*p(4)*p(5)*p(6)^2*p(9) + 2*p(2)*p(4)*p(5)*p(6)*p(10)^2 + 2*p(3)*...
         p(4)*p(5)*p(9)*p(7)^2 - 2*p(2)^2*p(3)*p(5)^2*p(9)*cos(q(2))^2 + p(2)^2*p(3)*p(5)^2*p(10)*sin(2*q(2)) +...
          p(2)^2*p(5)^2*p(9)*p(10)*sin(2*q(2)) + 4*p(2)*p(3)*p(4)*p(5)*p(6)*p(9));

 