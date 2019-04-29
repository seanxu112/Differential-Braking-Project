function [ xdot ] = nlDynamics( x, u )
vehicle_params;
%returns the derivatives of the states
% Unpack input vars
vx = x(1);
vy = x(2);
wz = x(3);
X = x(4);
Y = x(5);
psi = x(6);

Fx = u(1);
delta = u(2);

% Front/rear slip angles
af = atan((vy+a*wz)/vx) - delta;
ar = atan((vy-b*wz)/vx);

% Compute xdot
xdot = zeros(6,1);

xdot(1) = vy*wz - (2/m)*Ff(af)*sin(delta) + Fx/m;
xdot(2) = -vx*wz + (2/m)*(Ff(af)*cos(delta) + Fr(ar));
xdot(3) = (2/Jz)*(a*Ff(af)*cos(delta)-b*Fr(ar));
xdot(4) = vx*cos(psi) - vy*sin(psi);
xdot(5) = vx*sin(psi) + vy*cos(psi);
xdot(6) = wz;

end

