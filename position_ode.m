function xdot = position_ode(t,x,delta)
    vehicle_params;
    beta = atan(x(2)/x(1));
    if (isnan(beta))
        beta = 0;
    end
    xdot = zeros(3,1);
    V = sqrt(x(1)^2 + x(2)^2);
    xdot(1) = V*cos(beta+x(3));
    xdot(2) = V*sin(beta+x(3));
    xdot(3) = V*cos(beta)/(a+b)*(tan(delta)-tan(0));