function Mz = ESC_feedback_linearization(yaw_d, yaw_des_d, yaw_des_dd, delta, vy, yawRate,vx)
    % Computes the moment Mz required to control system
    
    L = 3.020; % wheelbase
    l_f = 1.8; % distance between c.g. and front axle (m)
    C_f = 5.9563e+04; % front tire cornering stiffness (N/rad)
    C_r = 8.8998e+04; % rear tire cornering stiffness (N/rad)
    l_r = L - l_f; % distance between c.g. and rear axle (m)
    I_z = 5550.045; % yaw moment of inertia

    rho = 0.5;
    
    Fyfl = C_f * (delta - (vy + l_f * yawRate) / vx);
    Fyfr = C_f * (delta - (vy + l_f * yawRate) / vx);
    Fyrl = C_r * (0 - (vy - l_r * yawRate) / vx);
    Fyrr = C_r * (0 - (vy - l_r * yawRate) / vx);
    
    f = (1 / I_z) * (l_f * (Fyfl + Fyfr) * cos(delta) - l_r * (Fyrl + Fyrr));
    
    e = yaw_des_d - yaw_d;
    
    denominator = (cos(delta) + rho);
    nominator = I_z * (yaw_des_dd + e) - f ;
    
%     nominator = I_z * ( (yaw_des_dd + .0001*e) - f );
    
    Mz = nominator / denominator;
    
    
%     if (Mz >= 20E3)
%     %         Mz = min( max(cellFxAlpha_rear{2, index_r}(end,:)),max(cellFxAlpha_front{2, index_f}(end,:)));
%         Mz = 20E3;
%         
%         warning('Mz at maximum')
%     end
end

