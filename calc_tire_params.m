function [Cf, Cr, Kus, uch] = calc_tire_params(ay_dot,ay,phi_dot,V,delta_dot)
    vehicle_params;
    Cf = m*b/(a+b)*(ay_dot/(delta_dot-ay/V+phi_dot))
    Cr = 2 * Cf;
%     Cf = 30000;
%     Cr = 2*Cf;
    Kus = m*g/L*(b/Cf - a/Cr);
    uch = sqrt(g*L/Kus);        % Characteristic Veolocity
