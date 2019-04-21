delta = 0;
x0 = [0 0 0]';
[t,x] = ode45(@(t,x) vehicle_dynamics_ode(t,x,u,delta), tspan, x0);