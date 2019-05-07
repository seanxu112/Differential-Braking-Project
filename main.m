% clear all
close all
clc

global cellFxAlpha_front cellFxAlpha_rear cellFySigma_front cellFySigma_rear
global g m L l_f l_r C_f C_r mue dt I_z W 

%% (1) Load Vehicle Parameters
load('tireModel.mat') % combined tire model
m = 2237; % vehicle mass (kg)
W = 1.720; % width
L = 3.020; % wheelbase
g = 9.81; % gravity acceleration (m/s^2)
mue = 0.6; % road friction coefficient
l_f = 1.8; % distance between c.g. and front axle (m)
C_f = 5.9563e+04; % front tire cornering stiffness (N/rad)
C_r = 8.8998e+04; % rear tire cornering stiffness (N/rad)
l_r = L - l_f; % distance between c.g. and rear axle (m)
% m_f = m/L*l_r; % vehicle front mass (kg)
% m_r = m/L*l_f; % vehicle rear mass (kg)
I_z = 5550.045; % yaw moment of inertia
dt = 0.01;                     % sampling rate
Tf = 15;                        % End of integration
time = 0:dt:Tf;                % Integration interval

%% (2) Compute driver steering input
mag = 20*pi/180; % Driver's steering input magnitude
Tfsteer = 0.3; % Driver's steering input final time
steerT = 0:dt:Tfsteer;
deltaF = mag*ones(1,length(time));
deltaF(1:length(steerT)) = mag/2-mag/2*cos(pi/Tfsteer*steerT);

%% (3) Reference Test
% Commented out because this code is not necessary - MW 20190505
% [yawRateRef1, vyRef1] = referenceGen(5*pi/180,10);
% [yawRateRef2, vyRef2] = referenceGen(30*pi/180,8);
% disp(['yawRateRef1, vyRef1 = ', num2str(yawRateRef1), ', ' num2str(vyRef1)]);
% disp(['yawRateRef2, vyRef2 = ', num2str(yawRateRef2), ', ' num2str(vyRef2)]);

%% (4) Set Initial Conditions

vx = 10; 
vy = 0; 
yaw = 1*pi/2; 
yawRate = 0; 
X = 0; 
Y = 0;
x0 = [vx vy yaw yawRate X Y];
yawRateRef_ESC = zeros(1,length(time));
yawRateRef_SMC = zeros(1,length(time));
yawRateRef_NoESC = zeros(1,length(time));
yawRateRef_lqr = zeros(1,length(time));
vyRef_ESC = zeros(1,length(time));
vyRef_NoESC = zeros(1,length(time));
vyRef_SMC = zeros(1,length(time)+1);
vyRef_lqr = zeros(1,length(time));
xESC = zeros(6,length(time)+1);
xSMC = zeros(6,length(time)+1);
xNoESC = zeros(6,length(time)+1);
xlqr = zeros(6,length(time)+1);
xESC(:,1) = x0';
xNoESC(:,1) = x0';
xSMC(:,1) = x0';
xlqr(:,1) = x0';
alpha_front_off = zeros(size(time));
alpha_rear_off = zeros(size(time));

%% (5) Run Simulations with ESC OFF
for i = 1:length(time)
    %read current state
    vx_NoESC = xNoESC(1,i);
    vy_NoESC = xNoESC(2,i);
    yaw_NoESC = xNoESC(3,i);
    yawRate_NoESC = xNoESC(4,i);
    X_NoESC = xNoESC(5,i);
    Y_NoESC = xNoESC(6,i);
    
    % Reference generator
    [yawRateRef_NoESC(i), vyRef_NoESC(i), ~] = referenceGen(deltaF(i),vx_NoESC);
    
    alpha_front_off(i) = deltaF(i) - atan((vy_NoESC+l_f)/vx_NoESC); 
    alpha_rear_off(i) = deltaF(i) - atan((vy_NoESC+l_r)/vx_NoESC); 
    % Simulation
    xNoESC(:,i+1) = vehicleDyn_better(xNoESC(:,i),deltaF(i),0,0,0,0);
end


%% (6) Run Close-Loop Simulations with LQR ESC ON

for i = 1:length(time)
    %read current state
    vx = xlqr(1,i);
    vy = xlqr(2,i);
    yaw = xlqr(3,i);
    yawRate = xlqr(4,i);
    X = xlqr(5,i);
    Y = xlqr(6,i);
    
    % Reference generator
    [yawRateRef_lqr(i), vyRef_lqr(i), ~] = referenceGen(deltaF(i),vx);
    
    % Discrete-time LQR controller
    Mz = controller_lqr(yawRateRef_lqr(i),vyRef_lqr(i),yawRate,vy,vx,deltaF(i));
    
    % Braking logic
    [Fxlf, Fxlr, Fxrf, Fxrr] = brakingLogic(Mz,vx,vy,yawRate,deltaF(i));
    
%     alpha_front_on(i) = deltaF(i) - atan((vy+l_f)/vx); 
%     alpha_rear_on(i) = deltaF(i) - atan((vy+l_r)/vx); 
    % Simulation
    xlqr(:,i+1) = vehicleDyn_better(xlqr(:,i),deltaF(i),Fxlf,Fxlr,Fxrf,Fxrr);

end


%% Old code

for i = 1:length(time)
    %read current state
    vx = xESC(1,i);
    vy = xESC(2,i);
    yaw = xESC(3,i);
    yawRate = xESC(4,i);
    X = xESC(5,i);
    Y = xESC(6,i);
    
    % Reference generator
    [yawRateRef_ESC(i), vyRef_ESC(i), ~] = referenceGen(deltaF(i),vx);
    
    % Discrete-time LQR controller
    Mz = ESCdlqr(yawRateRef_ESC(i),vyRef_ESC(i),yawRate,vy,vx);
    
    % Braking logic
    [Fxlf, Fxlr, Fxrf, Fxrr] = brakingLogic(Mz,vx,vy,yawRate,deltaF(i));
    
    alpha_front_on(i) = deltaF(i) - atan((vy+l_f)/vx); 
    alpha_rear_on(i) = deltaF(i) - atan((vy+l_r)/vx); 
    % Simulation
    xESC(:,i+1) = vehicleDyn_better(xESC(:,i),deltaF(i),Fxlf,Fxlr,Fxrf,Fxrr);

end

%% (7) Run Close-Loop Simulations with SMC ESC ON

vx = xSMC(1,1);
[yawRateRef_prev, ~, betaRef_prev] = referenceGen(deltaF(1),vx);
prev_t = 0;
% prev_beta = 0;
Mz_vec = [];
s_vec = [];
for i = 1:length(time)
    %read current state
    vx = xSMC(1,i);
    vy = xSMC(2,i);
    yaw = xSMC(3,i);
    yawRate = xSMC(4,i);
    X = xSMC(5,i);
    Y = xSMC(6,i);
    t = time(i);
    
    % Reference generator
    [yawRateRef_SMC(i), vyRef_SMC(i), betaRef_SMC] = referenceGen(deltaF(i),vx);
    yaw_des_dd = derivative_estimation([yawRateRef_prev, yawRateRef_SMC(i)], [prev_t, t]);
    if (vy ~=0)
        beta = atan(vx/vy);
    else 
        beta = 0;
    end
    if (i==1)
        prev_beta = beta;
    end
    beta_d = derivative_estimation([prev_beta, beta], [prev_t,t]);
    beta_des_d = derivative_estimation([betaRef_prev, betaRef_SMC], [prev_t, t]);
    
    % Discrete-time LQR controller
    [Mz,s] =  sliding_mode_db(yawRate, yawRateRef_SMC(i), yaw_des_dd, beta, betaRef_SMC, beta_d, beta_des_d, deltaF(i));
%     Mz = ESCdlqr(yawRateRef_ESC(i),vyRef_ESC(i),yawRate,vy,vx)
    Mz_vec = [Mz_vec, Mz];
    s_vec = [s_vec, s];
    % Braking logic
    [Fxlf, Fxlr, Fxrf, Fxrr] = brakingLogic(Mz,vx,vy,yawRate,deltaF(i));
    
    alpha_front_on(i) = deltaF(i) - atan((vy+l_f)/vx); 
    alpha_rear_on(i) = deltaF(i) - atan((vy+l_r)/vx); 
    % Simulation
    xSMC(:,i+1) = vehicleDyn_better(xSMC(:,i),deltaF(i),Fxlf,Fxlr,Fxrf,Fxrr);
    
    prev_t = t;
    prev_beta = beta;
    yawRateRef_prev = yawRateRef_SMC(i);
    betaRef_prev = betaRef_SMC;
    
end
figure(5)
plot(s_vec)
%% (8) Plot Routines 
figure (1)
subplot(2,1,1)
plot(time,xSMC(2,1:end-1),'Linewidth', 2)
hold on
plot(time,vyRef_SMC(1:end-1),'--','Linewidth', 2)
plot(time,xNoESC(2,1:end-1))
plot(time,vyRef_NoESC(1:end),'--')
hold off
xlabel('time (sec)')
ylabel('v_y (m/s)')
legend('v_y (ESC on)','v_y ref (ESC on)', 'v_y (ESC off)','v_y ref (ESC off)')
legend('Location','southeast')
subplot(2,1,2)
plot(time,xSMC(4,1:end-1),'Linewidth', 2)
hold on
plot(time,yawRateRef_SMC(1:end),'--','Linewidth', 2)
plot(time,xNoESC(4,1:end-1))
plot(time,yawRateRef_NoESC(1:end),'--')
xlabel('time (sec)')
ylabel('yaw rate (rad/s)')
legend('yaw rate (ESC on)','yaw rate ref (ESC on)', 'yaw rate (ESC off)','yaw rate ref (ESC off)')
legend('Location','southeast')

figure (2)
plot(xSMC(5,:),xSMC(6,:))
hold on
plot(xNoESC(5,:),xNoESC(6,:))
plot(xESC(5,:), xESC(6,:))
plot(xlqr(5,:), xlqr(6,:),'g')
plot([xESC(5,1)+l_f*cos(xESC(3,1)), xESC(5,1)-l_r*cos(xESC(3,1))],...
    [xESC(6,1)+l_f*sin(xESC(3,1)),xESC(6,1)-l_r*sin(xESC(3,1))],'c','LineWidth',2)
plot([xESC(5,end)+l_f*cos(xESC(3,end)), xESC(5,end)-l_r*cos(xESC(3,end))],...
    [xESC(6,end)+l_f*sin(xESC(3,end)),xESC(6,end)-l_r*sin(xESC(3,end))],'c','LineWidth',2)
plot([xNoESC(5,end)+l_f*cos(xNoESC(3,end)), xNoESC(5,end)-l_r*cos(xNoESC(3,end))],...
    [xNoESC(6,end)+l_f*sin(xNoESC(3,end)),xNoESC(6,end)-l_r*sin(xNoESC(3,end))],'c','LineWidth',2)
% plot([xlqr(5,end)+l_f*cos(xlqr(3,end)), xlqr(5,end)-l_r*cos(xlqr(3,end))],...
%     [xlqr(6,end)+l_f*sin(xlqr(3,end)),xlqr(6,end)-l_r*sin(xlqr(3,end))],'c','LineWidth',2)
xlabel('X (m)');
ylabel('Y (m)');
legend('Traj. with ESC on','Traj. with ESC off')

figure (3)
plot(time,xESC(3,1:end-1)*180/pi)
hold on
plot(time,xNoESC(3,1:end-1)*180/pi)
xlabel('time (sec)')
ylabel('yaw (deg)')
legend('yaw with ESC on(deg)','yaw with ESC off(deg)')
legend('Location','southeast')

%%
figure(4)
hold on
plot(time,alpha_rear_off)
plot(time,alpha_front_off)
plot(time,alpha_rear_on)
plot(time,alpha_front_on)
legend('Rear Wheel Off', 'Front Wheel Off', 'Rear Wheel On', 'Front Wheel On')
xlabel('time (sec)')
ylabel('Slipping Angle (Rad)')
title('With and Without ESC for Front and Rear Tires')

% for i = 1:2:length(time)
%     
%    simulate_car(time(i), xSMC(:,i), deltaF(i), min(xSMC(5,:)), max(xSMC(5,:)), min(xSMC(6,:)), max(xSMC(6,:)), xSMC)
% 
% end