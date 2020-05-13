% generate the dynamics for the hopping leg with boom
% Author: Yanran Ding and João Ramos
% Last modified: 2020/03/05
clear all

%% --- define symbols ---
syms q1 q2 real
syms dq1 dq2 real
syms L1 L2 real
syms M1 M2 real
syms rx1 ry1 rz1 rx2 ry2 rz2 real
syms Jxx1 Jyy1 Jzz1 Jxy1 Jxz1 Jyz1 real
syms Jxx2 Jyy2 Jzz2 Jxy2 Jxz2 Jyz2 real
syms NH Kv Kt Rw Irotor real
syms g real
syms u

%% --- variable lists ---
% Physical parameters of the robot
m_list_params = {
    'g'  'p(1)'; %Gravity
    
    'L1' 'p(2)'; 
    'L2' 'p(3)';
    
    'M1' 'p(4)'; %Total mass of links
    'M2' 'p(5)';
    
    'rx1' 'p(6)'; %Positio of CoM in local link frame
    'ry1' 'p(7)';
    'rz1' 'p(8)';
    
    'rx2' 'p(9)';
    'ry2' 'p(10)';
    'rz2' 'p(11)';
    
    'Jxx1' 'p(12)'; %Inertia tensor term in local link frame
    'Jyy1' 'p(13)';
    'Jzz1' 'p(14)';
    'Jxy1' 'p(15)';
    'Jxz1' 'p(16)';
    'Jyz1' 'p(17)';
    
    'Jxx2' 'p(18)'; %Inertia tensor term in local link frame
    'Jyy2' 'p(19)';
    'Jzz2' 'p(20)';
    'Jxy2' 'p(21)';
    'Jxz2' 'p(22)';
    'Jyz2' 'p(23)';
    
    'NH'        'p(24)';
    'Kv'        'p(25)';
    'Kt'        'p(26)';
    'Rw'        'p(27)';
    'Irotor'    'p(28)';
    };

J1 = [Jxx1 Jxy1 Jxz1;
      Jxy1 Jyy1 Jyz1;
      Jxz1 Jyz1 Jzz1];
  
J2 = [Jxx2 Jxy2 Jxz2;
      Jxy2 Jyy2 Jyz2;
      Jxz2 Jyz2 Jzz2];

% joint positions
m_list_q = {
    'q1' 'q(1)';
    'q2' 'q(2)';};

% joint velocities
m_list_dq = {
    'dq1' 'dq(1)';
    'dq2' 'dq(2)';};

% control input
m_list_u = {
    'u' 'u';};

%% --- variables ---
q = [q1 q2]';
dq = [dq1 dq2]';

%% --- forward kinematics ---
% Frames:
% 0 - origin
% 1 - boom top
% 2 - hip axis
% 3 - knee axis
% toe - robot foot

r0 = [L1; 0; 0];
R01 = rz(q1);
T01 = [R01 R01*r0;
       0, 0, 0, 1];

r1 = [L2; 0; 0];
R12 = rz(q2);
T12 = [R12 R12*r1;
       0, 0, 0, 1];
%% Joints positions
% 1 - Hand
% 2 - Waist

p1 = T01(1:3,4);
write_fcn_m('fcn_p1.m',{'q','p'},[m_list_q;m_list_params],{p1,'p1'});

R02 = R01 * R12;
T02 = T01 * T12;
p2 = T02(1:3,4);
write_fcn_m('fcn_p2.m',{'q','p'},[m_list_q;m_list_params],{p2,'p2'});

% J_foot
J_foot = simplify(jacobian(p2,q));
write_fcn_m('fcn_J_foot.m',{'q','p'},[m_list_q;m_list_params],{J_foot,'J_foot'});

% Foot Velocity
v_foot = J_foot*dq;
write_fcn_m('fcn_v_foot.m', {'q','p'}, [m_list_q;m_list_params],{v_foot,'v_foot'});

%% Center of Mass (com) positions and velocities of each link
p_1com = T01*[rx1; ry1; rz1; 1]; 
p_1com = p_1com(1:3,1);
v_1com = jacobian(p_1com,q) * dq;

p_2com = T02*[rx2; ry2; rz2; 1];
p_2com = p_2com(1:3,1);
v_2com = jacobian(p_2com,q) * dq;

CoM = (M1*p_1com + M2*p_2com)/(M1 + M2);
write_fcn_m('fcn_CoM.m',{'q', 'p'},[m_list_q;m_list_params],{CoM,'CoM'});

%% Angular velocity jacobians
u1 = [0; 0; 1];
u2 = [0; 0; 1];
Jw1 = [u1 [0; 0; 0]];
Jw2 = [u1 R01*u2];

% Angular velocity of frames in respect to the world frame
w1 = Jw1 * dq;
w2 = Jw2 * dq;

%% --- Energy and Lagrangian ---
KE_1 = 0.5 * v_1com' * M1 * v_1com + 0.5 * w1' * R01 * J1 * transpose(R01) * w1;
KE_2 = 0.5 * v_2com' * M2 * v_2com + 0.5 * w2' * R02 * J2 * transpose(R02) * w2;

% Kinetic energy
KE = simplify(KE_1 + KE_2);   

% Potential energy
PE = M1*[0 g 0]*p_1com + M2*[0 g 0]*p_2com;

%To calculate the actuation selection matrix:
Upsilon = [q2]; %where control torques go: hip and knee only, first two joints are passive

%% --- Euler-Lagrange Equation ---
[De, Ce, Ge, Be] = std_dynamics(KE,PE,q,dq, Upsilon);

Jrotor = diag([0, Irotor*NH^2]);
Bdamp = diag([0,Kv*Kt*NH^2/Rw]);

De = De + Jrotor;
Ce = Ce + Bdamp;

write_fcn_m('fcn_De.m',{'q', 'p'},[m_list_q;m_list_params],{De,'De'});
write_fcn_m('fcn_Ce.m',{'q', 'dq', 'p'},[m_list_q;m_list_dq;m_list_params],{Ce,'Ce'});
write_fcn_m('fcn_Ge.m',{'q', 'p'},[m_list_q;m_list_params],{Ge,'Ge'});
write_fcn_m('fcn_Be.m',{'q', 'p'},[m_list_q;m_list_params],{Be,'Be'});


%% --- Linearized Matrices ---

dynamics = simplify([dq(1); dq(2); De\(Be*u - Ce*dq - Ge)]);

Alin = simplify(jacobian(dynamics, [q;dq]));
Blin = simplify(jacobian(dynamics, u));

write_fcn_m('fcn_Alin.m',{'q', 'dq', 'p', 'u'},[m_list_q;m_list_dq;m_list_params;m_list_u],{Alin,'Alin'});
write_fcn_m('fcn_Blin.m',{'q', 'dq', 'p'},[m_list_q;m_list_dq;m_list_params],{Blin,'Blin'});

