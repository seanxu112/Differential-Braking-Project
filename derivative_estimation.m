function x_dot = derivative_estimation(x,t)
    x_diff = diff(x);
    t_diff = diff(t);
    x_dot = x_diff/t_diff;
    if isnan(x_dot)
        x_dot = 0;
    end
        
end