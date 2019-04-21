function xdot = vehicle_dynamics_ode(t,x,u,delta)
    vehicle_params;
    PBfl = u(1);    %Brake Pressure of the front left wheel
    PBfr = u(2);    %Brake Pressure of the front right wheel
    vx = x(1);      %Longitudinal Velocity
    vy = x(2);      %Lateral Velocity
    phi_dot = x(3);     %Yaw Rate
    d = 5;          %Track Width %%%%%%%%%%% Need to Change %%%%%%%
    delta =         %Some Designed input of steering angle
    
    xdot = zeros(3,1);
    f = zeros(3,1);
    g = zeros(3,1);
    
    f(1) = phi_dot*vy-Cf/m*(delta - (v+phi_dot*a)/vx)*delta + 1/m*Ts/reff;
    f(2) = -phi_dot*vx+Cr/m*(phi_dot*b-vy)/vx-...
                Cf/m*((vy+phi_dot*a)/vx-delta)+1/m*Ts/reff*delta;
    f(3) = a/Jz*Cf*(delta-(vy+phi_dot*a)/vx)-...
                b/Jz*Cr*((phi_dot*b-v)/vx)+a/Jz*Ts/reff*delta;
    
    fB = calc_rear_brake_pressure_coef(ax);
    g31 = 1/Jz*(d/2*(KBf/reff+fB*KBr/reef)-a*delta*KBf/reff);
    g12 = -1/Jz*(d/2*(KBf/reff+fB*KBr/reef)+a*delta*KBf/reff);
    g(1) = -1/m*(KBf/reff+fB*KBf/reff)*(PBfl+PBfr);
    g(2) = 1/m*delta*KBf/reff*(PBfl+PBfr);
    g(3) = g31*PBfl+g32*PBfr;
end