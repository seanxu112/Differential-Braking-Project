syms dphi Iz l_f Fyfl Fyfr deltaF l_r Fyrl Fyrr Mz C_f C_r vx vy

Fyfl = .5*2*C_f*(deltaF - ( vy + l_f*dphi )/vx);
Fyfr = .5*2*C_f*(deltaF - ( vy + l_f*dphi )/vx);
Fyrl = .5*2*C_r*( - ( vy - l_r*dphi )/vx);
Fyrr = .5*2*C_r*( - ( vy - l_r*dphi )/vx);

fx = (1/Iz)*(l_f*(Fyfl + Fyfr)*cos(deltaF) - l_r*(Fyrl + Fyrr));

gx = (1/Iz)*(cos(deltaF)+.5)*Mz;

A = jacobian(fx, dphi);

B = jacobian(gx, Mz);

