function Mz = controller_lqr(yawRateRef,vyRef,yawRate,vy,vx,deltaF)
% Inputs: 
%    yawRateRef: yaw rate reference in rad/s
%    vyref: lateral velocity reference in m/s
%    yawRate: yaw rate feedback state in rad/s
%    vy: lateral velocity feedback state in m/s
%    vx: longitudinal velocity feedback state in m/s
% Outputs:
%    Mz: yaw moment control signal in N*m
global m l_f l_r C_f C_r dt I_z

A = -((10*C_f*cos(deltaF)*l_f^2)/vx + (13*C_r*l_r^2)/(5*vx))/I_z;
 
B = (cos(deltaF) + 1/2)/I_z;

% A = -((4*C_f*cos(deltaF)*l_f^2)/vx + (4*C_r*l_r^2)/vx)/I_z;

% B = (cos(deltaF) + 1/2)/I_z;

% A = -((2*C_f*cos(deltaF)*l_f^2)/vx + (2*C_r*l_r^2)/vx)/I_z;
 
% B = (cos(deltaF) + 1/2)/I_z;

Q = 10000;

R = 1;

K = lqrd(A,B,Q,R,dt); %LQR controller

e_yaw = yawRate - yawRateRef;

amp = 100000;

Mz = amp*K*e_yaw;
 
% a11 = -2*(C_r+C_f)/m/vx;
% a12 = -2*(C_f*l_f-C_r*l_r)/m/vx-vx;
% a21 = -2*(C_f*l_f-C_r*l_r)/I_z/vx;
% a22 = -2*(C_f*l_f^2+C_r*l_r^2)/I_z/vx;
% 
% A = [a11, a12; a21, a22];
% Ad = A*dt+eye(2);
% 
% b11 = 2*C_f/m;
% b21 = 2*l_f*C_f/I_z;
% Bd = [b11;b21];
% c11 = 0;
% c21 = 1/I_z;
% Cd = [c11;c21];
% Q = [1, 0; 0 100];
% R = [1 0 ; 0 0.01];
% ...
% [K] = lqrd(Ad,[Bd,Cd],Q,R,dt); %LQR controller
% e_vy = vy - vyRef;
% e_yaw = yawRate - yawRateRef;
% input = K*[e_vy;e_yaw];
% Mz = input(2);