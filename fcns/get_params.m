function p = get_params()

% % parameters for matrix calculations
% [g, HB, LB, DB, LH, DK, LK, M1, M2, M3, M4, ...
%        rx1, ry1, rz1, rx2, ry2, rz2, rx3, ry3, rz3, rx4, ry4, rz4, J1, J2, J3, J4] = fcn_params;

g = 9.81;

% Link Lengths
L1 = 0.3;
L2 = 0.3;

% Link Masses
M1 = 1;
M2 = 1;

% Center of masses

rx1 = -L1/2;
ry1 = 0;
rz1 = 0;

rx2 = -L2/2;
ry2 = 0;
rz2 = 0;

Jxx1 = 0;
Jyy1 = 0;
Jzz1 = M1*L1^2/12;
Jxy1 = 0;
Jxz1 = 0;
Jyz1 = 0;

Jxx2 = 0;
Jyy2 = 0;
Jzz2 = M2*L2^2/12;
Jxy2 = 0;
Jxz2 = 0;
Jyz2 = 0;

p.N_animate = 30;

p.params = [g, L1, L2, M1, M2, ...
    rx1, ry1, rz1, rx2, ry2, rz2, ...
    Jxx1, Jyy1, Jzz1, Jxy1, Jxz1, Jyz1,...
    Jxx2, Jyy2, Jzz2, Jxy2, Jxz2, Jyz2];
    
    
    
    
    