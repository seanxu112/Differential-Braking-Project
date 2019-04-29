function [yawRateRef, vyRef, betaRef] = referenceGen(deltaF,vx)
% Inputs: 
%    deltaF: Steering angle in radiants, 
%    vx: vehicle longitudinal velocity in m/s
% Outputs:
%    yawRateRef: yaw rate reference in rad/s
%    vyref: lateral velocity reference in m/s
global g m L l_f l_r C_f C_r mue 

% implement reference generator according to ESC notes
yawRateRef = vx/(L+((m*vx^2*(l_r*C_r-l_f*C_f))/(2*C_f*C_r*L)))*deltaF;
betaRef = (l_r-(l_f*m*vx^2)/(2*C_r*L))/(L+(m*vx^2*(l_r*C_r-l_f*C_f))/(2*C_f*C_r*L))*deltaF;
yaw_bound = 0.85 * mue*g/vx*sign(yawRateRef);
beta_bound = 6.72*pi/180;
if abs(yawRateRef)>yaw_bound
    yawRateRef = yaw_bound;
end
if abs(betaRef)<beta_bound
    betaRef = beta_bound * sign(betaRef);
end
...
...
...
vyRef = vx*betaRef;
