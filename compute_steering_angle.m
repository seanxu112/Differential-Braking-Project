function delta_ss = compute_steering_angle(R)
%Computes steady state steering angle for negotiating a circular road of
%radius R

m = 2237; % vehicle mass (kg)
W = 1.720; % width
L = 3.020; % wheelbase
g = 9.81; % gravity acceleration (m/s^2)
mue = 0.6; % road friction coefficient
l_f = 1.8; % distance between c.g. and front axle (m)
C_f = 5.9563e+04; % front tire cornering stiffness (N/rad)
C_r = 8.8998e+04; % rear tire cornering stiffness (N/rad)
l_r = L - l_f; % distance between c.g. and rear axle (m)
% m_f = m/L*l_r; % vehicle front mass (kg)
% m_r = m/L*l_f; % vehicle rear mass (kg)
I_z = 5550.045; % yaw moment of inertia
dt = 0.01;                     % sampling rate
Tf = 4;                        % End of integration
time = 0:dt:Tf;                % Integration interval

v = 5;
delta_ss = (l_f + l_r) / R + ((m * l_r * C_r - m * l_f * C_f) * v^2 ) / (2 * C_f * C_r * (l_r + l_f));

end

