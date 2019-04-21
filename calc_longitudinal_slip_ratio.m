function sigma_x = calc_longitudinal_slip_ratio(w_wheel,vx,a)
    reff = 1;   %%%%% Need to Change %%%%%%%
    sigma_x = (reff*w_wheel-vx)/((a>=0)*reff*w_wheel+(a<0)*vx);
end