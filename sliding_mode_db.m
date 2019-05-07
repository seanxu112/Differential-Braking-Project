function [Mz,s] = sliding_mode_db(yaw_d, yaw_des_d, yaw_des_dd, beta, beta_des, beta_d, beta_des_d, delta)
    global dt I_z 
    global cellFxAlpha_front cellFxAlpha_rear cellFySigma_front cellFySigma_rear

    persistent s_int;
    if isempty(s_int)
        s_int = 0;
    end
    % Controller Parameters
    rho = 0.5;
    ki = 0.1;
    eta = 0.3;
    Xi = 0.0005;
    
    s = yaw_d - yaw_des_d + Xi*(beta- beta_des);
    C1 = (rho+cos(delta))/I_z;
    num = (-ki*s_int - eta*s + yaw_des_dd - Xi*(beta_d - beta_des_d));
    Mz = num/C1;
%     index_f =  find(cell2mat(cellFxAlpha_front(1,:)) <= abs(alpharf),1,'last');
%     index_r = 
    if (Mz >= 20E3)
%         Mz = min( max(cellFxAlpha_rear{2, index_r}(end,:)),max(cellFxAlpha_front{2, index_f}(end,:)));
        Mz = 20E3;
    end
    s_int = s_int + s*dt;
    
end