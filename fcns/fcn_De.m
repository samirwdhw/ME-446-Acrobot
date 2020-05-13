function [De] = fcn_De(q,p)

De = zeros(2,2);

  De(1,1)=p(14) + p(20) + p(2)^2*p(4) + p(2)^2*p(5) + p(3)^2*p(5) + p(4)*p(6)^2 + p(5)*p(9)^2 +...
          p(4)*p(7)^2 + p(5)*p(10)^2 + 2*p(2)*p(4)*p(6) + 2*p(3)*p(5)*p(9) + 2*p(2)*p(3)*p(5)*cos(q(2)) + 2*p(2)*...
         p(5)*p(9)*cos(q(2)) - 2*p(2)*p(5)*p(10)*sin(q(2));
  De(1,2)=p(20) + p(3)^2*p(5) + p(5)*p(9)^2 + p(5)*p(10)^2 + 2*p(3)*p(5)*p(9) + p(2)*p(3)*p(5)*...
         cos(q(2)) + p(2)*p(5)*p(9)*cos(q(2)) - p(2)*p(5)*p(10)*sin(q(2));
  De(2,1)=p(20) + p(3)^2*p(5) + p(5)*p(9)^2 + p(5)*p(10)^2 + 2*p(3)*p(5)*p(9) + p(2)*p(3)*p(5)*...
         cos(q(2)) + p(2)*p(5)*p(9)*cos(q(2)) - p(2)*p(5)*p(10)*sin(q(2));
  De(2,2)=p(20) + p(28)*p(24)^2 + p(3)^2*p(5) + p(5)*p(9)^2 + p(5)*p(10)^2 + 2*p(3)*p(5)*p(9);

 