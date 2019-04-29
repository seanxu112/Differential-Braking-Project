function fB = calc_rear_brake_pressure_coef(ax)
    vehicle_params;
    g = 9.81;
    h = 0.5;        % Height of the COG
    % KBf is the Brake Gain of the Front Wheel, 
    % KBr is the Brake Gain of the Rear Wheel
    fB = KBf*(a*g+ax*h)/(KBr*(b*g-ax*h));
    