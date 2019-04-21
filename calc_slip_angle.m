function [alpha_f, alpha_r] = calc_slip_angle(vx,vy,wz,delta)
    a = 1.46;   % m
    b = 1.55;   % m
    alpha_f = atan((vy+a*wz)/vx) - delta;
    alpha_r = atan((vy-b*wz)/vx);
end