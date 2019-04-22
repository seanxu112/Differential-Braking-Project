function xdot = vehicle_dynamics_ode(t,x,u,delta,Ts)
    vehicle_params;
    PBfl = u(1);    %Brake Pressure of the front left wheel
    PBfr = u(2);    %Brake Pressure of the front right wheel
    u = x(1);      %Longitudinal Velocity
    v = x(2);      %Lateral Velocity
    phi_dot = x(3);     %Yaw Rate
    d = 5;          %Track Width %%%%%%%%%%% Need to Change %%%%%%%
    %delta =         %Some Designed input of steering angle
    V = sqrt(x(1)^2 + x(2)^2);
    beta = atan(x(2)/x(1));
    if (isnan(beta))
        beta = 0;
    end
    
    xdot = zeros(6,1);
    f = zeros(3,1);
    g = zeros(3,1);
    
    f(1) = phi_dot*v-Cf/m*(delta - (v+phi_dot*a)/u)*delta + 1/m*Ts/reff;
    f(2) = -phi_dot*u+Cr/m*(phi_dot*b-v)/u-...
                Cf/m*((v+phi_dot*a)/u-delta)+1/m*Ts/reff*delta;
    f(3) = a/Jz*Cf*(delta-(v+phi_dot*a)/u)-...
                b/Jz*Cr*((phi_dot*b-v)/u)+a/Jz*Ts/reff*delta;
    
    ax = f(1);          % Verify
    fB = calc_rear_brake_pressure_coef(ax);
    g31 = 1/Jz*(d/2*(KBf/reff+fB*KBr/reff)-a*delta*KBf/reff);
    g12 = -1/Jz*(d/2*(KBf/reff+fB*KBr/reff)+a*delta*KBf/reff);
    g(1) = -1/m*(KBf/reff+fB*KBf/reff)*(PBfl+PBfr);
    g(2) = 1/m*delta*KBf/reff*(PBfl+PBfr);
    g(3) = g31*PBfl+g12*PBfr;
    xdot(1:3) = f+g;
    xdot(4) = V*cos(beta+x(6));
    xdot(5) = V*sin(beta+x(6));
    xdot(6) = V*cos(beta)/(a+b)*(tan(delta));
end