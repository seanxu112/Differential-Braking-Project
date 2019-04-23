function xdot = vehicle_dynamics_ode(t,x,u,delta,Ts)
    persistent ax ay_prev t_prev ay delta_prev tpp ax_prev
    if isempty(ax)
        ax = 0;
%         ax_prev = 0;
        t_prev = 0;
        ay_prev = 0;
        ay = 0;
        delta_prev = 0;
        tpp = 0;
        ax_prev = 0;
    end
    
    
    %%%%%%%%% Vehicle Parameters %%%%%%%%%%
    vehicle_params;
    V = sqrt(x(1)^2 + x(2)^2);
    
    if (1)
        Cf = 1000;
        Cr = 1000;
    end
    %%%%%%%%%%%%%  Need to Fix Cf Calculation %%%%%%%%%%%%
%     if ((t==0)||(t_prev-tpp==0))
%         Cf = 1000;
%         Cr = 1000;
%     else
%         ay_dot = (ay-ay_prev)/(t_prev-tpp);
%         ax_dot = (ax-ax_prev)/(t_prev-tpp);
%         delta_dot = (delta-delta_prev)/(t_prev-tpp);
%         ay_F = ay*cos(delta)-ax*sin(delta);         %Front Wheel Lateral Acceleration
%         ay_F_dot = ay_dot*cos(delta)-ay*sin(delta)*delta_dot - ...
%             ax_dot*sin(delta)-ax*cos(delta)*delta_dot;      %Front Wheel Lateral Jerk
%         [Cf, Cr, Kus, uch] = calc_tire_params(ay_F_dot,ay_F,x(3),V,delta_dot);
%     end
%     t-t_prev
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    PBfl = u(1);    %Brake Pressure of the front left wheel
    PBfr = u(2);    %Brake Pressure of the front right wheel
    u = x(1);      %Longitudinal Velocity
    v = x(2);      %Lateral Velocity
    phi_dot = x(3);     %Yaw Rate
    d = 5;          %Track Width %%%%%%%%%%% Need to Change %%%%%%%
    %delta =         %Some Designed input of steering angle
    
    beta = atan(x(2)/x(1));
    if (isnan(beta))
        beta = 0;
    end
    
    
    %%%%%%%%% Dynamic Equations %%%%%%%%
    xdot = zeros(5,1);

    f = zeros(3,1);
    g = zeros(3,1);
    
    f(1) = phi_dot*v-Cf/m*(delta - (v+phi_dot*a)/u)*delta + 1/m*Ts/reff;
    f(2) = -phi_dot*u+Cr/m*(phi_dot*b-v)/u-...
                Cf/m*((v+phi_dot*a)/u-delta)+1/m*Ts/reff*delta;
    f(3) = a/Jz*Cf*(delta-(v+phi_dot*a)/u)-...
                b/Jz*Cr*((phi_dot*b-v)/u)+a/Jz*Ts/reff*delta;
%     Cf
%     ax = f(1);          % Verify
    fB = calc_rear_brake_pressure_coef(ax);
    g31 = 1/Jz*(d/2*(KBf/reff+fB*KBr/reff)-a*delta*KBf/reff);
    g32 = -1/Jz*(d/2*(KBf/reff+fB*KBr/reff)+a*delta*KBf/reff);
    g(1) = -1/m*(KBf/reff+fB*KBf/reff)*(PBfl+PBfr);
    g(2) = -1/m*delta*KBf/reff*(PBfl+PBfr);
    g(3) = g31*PBfl+g32*PBfr;
    xdot(1:3) = f+g;

    xdot(4) = cos(phi_dot)*u - sin(phi_dot)*v;
    xdot(5) = sin(phi_dot)*u + cos(phi_dot)*v;
    
    ax = xdot(1);
    ay_prev = ay;
    ay = xdot(2);
    tpp = t_prev;
    t_prev = t;
    delta_prev = delta;

end