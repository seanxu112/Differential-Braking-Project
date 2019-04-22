clc
close all
clear all


delta = @(t) deg2rad(2);
%delta = @(t) ...;
x0 = [1 0 0 0 0]';
tspan = [0 20];
u = [100 100];
Ts = 0;

[t_no_brake,x_no_brake] = ode45(@(t,x) vehicle_dynamics_ode(t,x,zeros(2,1),delta(t), Ts), tspan, x0);
[t_brake,x_brake] = ode45(@(t,x) vehicle_dynamics_ode(t,x,u,delta(t), Ts), tspan, x0);
% [t_pos,pos] = ode45(@(t,x) position_ode(t,x,delta(t)), tspan,x0_x);

figure()
hold on
plot(x0(4),x0(5), '*')
plot(x_no_brake(:,4),x_no_brake(:,5))
xlim([-50, 350])
ylim([-50, 350])
title('Position')
plot(x_brake(:,4),x_brake(:,5))
legend('Initial Position', 'No Brake Trajectory', 'Brake Trjectory' )
% figure()
% plot(t_no_brake,x_no_brake(:,4))