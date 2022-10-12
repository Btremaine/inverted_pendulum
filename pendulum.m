% pendulum.m
% inverted pendulum
% design LQR at linearized set point
% then run in Simulink

% MKS units
m = 0.5;
L= 1.0;
k= 0.5;
g= 9.81;

J = m*L^2;

% eqt. of motion: tau - m*g*L*sin(th) - k*th_dot = J*th_ddot
% where J = mL^2
% 
% th_ddot = -(g/L)*sin(th)  -k/(m*L^2) * th_dot - (1/(m*L^2))*tau
% 
% linearize around th= pi, sin(th) ~= pi - th
% and augment with integral of position error.
% err = th_ref - th
%
% states [th_dot, th, y]^T
%  
A = [-k/J  +m*g*L/J  0 ;
      1    0         0 ;
      0    1         0]; 

B = [1/J   0;
     0     0;
     0    -1];

C = [1 0 0 ;
     0 1 0 ;
     0 0 1];
D = 0;
SYS = ss(A,B(:,1),C,D);

Ts= 0.1;
sys= c2d(SYS,Ts);


CO= ctrb(A,B(:,1));
OB= obsv(A,C);

lam = 1.0;
Q= 0.25*lam;
R= 0.10*lam;
[K, S, CLP] = lqr(SYS,Q,R);

Ad= sys.A;
Bd= sys.B;
Cd= sys.C;
Dd= sys.D;

dstep(Ad-Bd*K,Bd,Cd,Dd);
