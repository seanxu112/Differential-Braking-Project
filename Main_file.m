clc
close all
clear all


delta = @(t) deg2rad(1);
%delta = @(t) ...;
x0 = [10 2 0 0 0 0]';
tspan = [0 10];
u = zeros(2,1);
Ts = 100;

[t,x] = ode45(@(t,x) vehicle_dynamics_ode(t,x,u,delta(t), Ts), tspan, x0);
% [t_pos,pos] = ode45(@(t,x) position_ode(t,x,delta(t)), tspan,x0_x);
figure()
plot(x(:,4),x(:,5))
xlim([-50, 150])
ylim([-50, 150])
title('Position')
hold on
plot(x0(4),x0(5), '*')
legend('Initial Position')
figure()
plot(t,x(:,4))