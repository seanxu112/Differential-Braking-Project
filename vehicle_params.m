% Vehicle Params
m = 2237;   % kg
Jz = 5112;  % kg*m^2
a = 1.46;   % m
b = 1.55;   % m
g = 9.81;
% Tire model params
A = -6.8357;
B = 0.0325;
C = 238.9874;
% Tire forces
Ff = @(af) m*(a/(a+b))*A*sin(C*atan(B*af));
Fr = @(ar) m*(b/(a+b))*A*sin(C*atan(B*ar));


reff = 0.3;         % Effective Radius of Wheel  %%%%%% Need to Change %%%%%
miu = 0.2;          % Maximum tire–road friction coecient   %%%%% Need to Change %%%%%%
Cf = 1;               % Front tyre cornering tyre stiffness
Cr = 1;               % Rear tyre cornering tyre stiffness
L  = 1;               % Wheelbase
Kus = m*g/L*(b/Cf - a/Cr);
uch = sqrt(g*L/Kus);        % Characteristic Veolocity

KBf = 1;
KBr = 1;
