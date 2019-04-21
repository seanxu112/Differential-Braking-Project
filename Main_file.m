clc
delta = 0;
x0 = [10 10 0]';
tspan = [0 10];
u = zeros(2,1);
Ts = 0;

[t,x] = ode45(@(t,x) vehicle_dynamics_ode(t,x,u,delta, Ts), tspan, x0);