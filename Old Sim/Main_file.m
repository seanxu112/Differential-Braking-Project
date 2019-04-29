clc
close all
clear all

tmax = 20;
% delta = @(t) deg2rad(5);%min(deg2rad(20)*t/(tmax-5),deg2rad(20));
delta =@(t) (2*pi/180*((t>5).* (t<=7))+-2*pi/180*((t>12).* (t<=14)));

x0 = [10 0 0 0 0]';
tspan = [0 tmax];
u = [50 50];

Ts = 0;

[t_no_brake,x_no_brake] = ode45(@(t,x) vehicle_dynamics_ode(t,x,zeros(2,1),delta(t), Ts), tspan, x0);
[t_brake,x_brake] = ode45(@(t,x) vehicle_dynamics_ode(t,x,u,delta(t), Ts), tspan, x0);
% [t_pos,pos] = ode45(@(t,x) position_ode(t,x,delta(t)), tspan,x0_x);
figure()
plot(x0(4),x0(5), '*')
hold on
plot(x_brake(:,4),x_brake(:,5))
xlim([-50, 500])
ylim([-50, 20])
title('Position')
hold on
plot(x_no_brake(:,4),x_no_brake(:,5))
xlim([-20, 500])
ylim([-40, 20])
title('Position')
legend('Initial Position',  'Brake Trjectory', 'No Brake Trajectory' )
% figure()
% plot(t_no_brake,x_no_brake(:,4))
