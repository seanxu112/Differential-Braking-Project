function xplus = vehicleDyn_better(x,deltaF,Fxlf,Fxlr,Fxrf,Fxrr)
global m l_f l_r I_z W dt mue g
global cellFxAlpha_front cellFxAlpha_rear cellFySigma_front cellFySigma_rear

Fzlf = mue*m*g*l_r/(l_r+l_f)/2;
Fzlr = mue*m*g*l_f/(l_r+l_f)/2;
Fzrf = mue*m*g*l_r/(l_r+l_f)/2;
Fzrr = mue*m*g*l_f/(l_r+l_f)/2;  

vxlf = x(1)-W/2*x(4);
vxrf = x(1)+W/2*x(4);
vylf = x(2)+l_f*x(4);
vyrf = x(2)+l_f*x(4);
vxlr = x(1)-W/2*x(4);
vxrr = x(1)+W/2*x(4);
vylr = x(2)-l_r*x(4);
vyrr = x(2)-l_r*x(4);

%% left-front tire
alphalf = deltaF - atan(vylf/vxlf);
index = find(cell2mat(cellFxAlpha_front(1,:)) <= abs(alphalf),1,'last');
if isempty(index)
    if abs(Fxlf) >= Fzlf
        s = sign(Fxlf);
        Fxlf = Fzlf*s;
        warning('Warning: The braking force (Fxlf) is over the maximum capacity of the tire force.');
    end
    Fylf = sqrt(Fzlf^2-Fxlf^2)*sign(alphalf);
else
    if Fxlf <= 0
        index1 = find(cellFxAlpha_front{2, index}(end,:) <= Fxlf,1,'last');
    else
        index1 = find(cellFxAlpha_front{2, index}(end,:) >= Fxlf,1,'first');
    end
    if isempty(index1)
        if abs(Fxlf) >= Fzlf
            s = sign(Fxlf);
            Fxlf = Fzlf*s;
            warning('Warning: The braking force (Fxlf) is over the maximum capacity of the tire force.');
        end
        Fylf = sqrt(Fzlf^2-Fxlf^2)*sign(alphalf);
    else
        sigmalf = cellFxAlpha_front{2, index}(1,index1);
        index2 = find(cell2mat(cellFySigma_front(1,:)) <= abs(sigmalf),1,'last');
        Fylf = interp1(cellFySigma_front{2, index2}(1,:),cellFySigma_front{2, index2}(2,:),alphalf);
    end
end
%% left-rear tire
alphalr = - atan(vylr/vxlr);
index = find(cell2mat(cellFxAlpha_rear(1,:)) <= abs(alphalr),1,'last');
if isempty(index)
    if abs(Fxlr) >= Fzlr
        s = sign(Fxlr);
        Fxlr = Fzlr*s;
        warning('Warning: The braking force (Fxlr) is over the maximum capacity of the tire force.');
    end
    Fylr = sqrt(Fzlr^2-Fxlr^2)*sign(alphalr);
else
    if Fxlr <= 0
        index1 = find(cellFxAlpha_rear{2, index}(end,:) <= Fxlr,1,'last');
    else
        index1 = find(cellFxAlpha_rear{2, index}(end,:) >= Fxlr,1,'first');
    end
    if isempty(index1)
        if abs(Fxlr) >= Fzlr
            s = sign(Fxlr);
            Fxlr = Fzlr*s;
            warning('Warning: The braking force (Fxlr) is over the maximum capacity of the tire force.');
        end
        Fylr = sqrt(Fzlr^2-Fxlr^2)*sign(alphalr);
    else
        sigmalr = cellFxAlpha_rear{2, index}(1,index1);
        index2 = find(cell2mat(cellFySigma_rear(1,:)) <= abs(sigmalr),1,'last');
        Fylr = interp1(cellFySigma_rear{2, index2}(1,:),cellFySigma_rear{2, index2}(2,:),alphalr);
    end
end
%% right-front tire
alpharf = deltaF - atan(vyrf/vxrf);
index = find(cell2mat(cellFxAlpha_front(1,:)) <= abs(alpharf),1,'last');
if isempty(index)
    if abs(Fxrf) >= Fzrf
        s = sign(Fxrf);
        Fxrf = Fzrf*s;
        warning('Warning: The braking force (Fxrf) is over the maximum capacity of the tire force.');
    end
    Fyrf = sqrt(Fzrf^2-Fxrf^2)*sign(alpharf);
else
    if Fxrf <= 0
        index1 = find(cellFxAlpha_front{2, index}(end,:) <= Fxrf,1,'last');
    else
        index1 = find(cellFxAlpha_front{2, index}(end,:) >= Fxrf,1,'first');
    end
    if isempty(index1)
        if abs(Fxrf) >= Fzrf
            s = sign(Fxrf);
            Fxrf = Fzrf*s;
            warning('Warning: The braking force (Fxrf) is over the maximum capacity of the tire force.');
        end
        Fyrf = sqrt(Fzrf^2-Fxrf^2)*sign(alpharf);
    else
        sigmarf = cellFxAlpha_front{2, index}(1,index1);
        index2 = find(cell2mat(cellFySigma_front(1,:)) <= abs(sigmarf),1,'last');
        Fyrf = interp1(cellFySigma_front{2, index2}(1,:),cellFySigma_front{2, index2}(2,:),alpharf);
    end
end
%% right-rear tire
alpharr = - atan(vyrr/vxrr);
index = find(cell2mat(cellFxAlpha_rear(1,:)) <= abs(alpharr),1,'last');
if isempty(index)
    if abs(Fxrr) >= Fzrr
        s = sign(Fxrr);
        Fxrr = Fzrr*s;
        warning('Warning: The braking force (Fxrr) is over the maximum capacity of the tire force.');
    end
    Fyrr = sqrt(Fzrr^2-Fxrr^2)*sign(alpharr);
else
    if Fxrr <= 0
        index1 = find(cellFxAlpha_rear{2, index}(end,:) <= Fxrr,1,'last');
    else
        index1 = find(cellFxAlpha_rear{2, index}(end,:) >= Fxrr,1,'first');
    end
    if isempty(index1)
        if abs(Fxrr) >= Fzrr
            s = sign(Fxrr);
            Fxrr = Fzrr*s;
            warning('Warning: The braking force (Fxrr) is over the maximum capacity of the tire force.');
        end
        Fyrr = sqrt(Fzrr^2-Fxrr^2)*sign(alpharr);
    else
        sigmarr = cellFxAlpha_rear{2, index}(1,index1);
        index2 = find(cell2mat(cellFySigma_rear(1,:)) <= abs(sigmarr),1,'last');
        Fyrr = interp1(cellFySigma_rear{2, index2}(1,:),cellFySigma_rear{2, index2}(2,:),alpharr);
    end
end

Nx = Fxlf*cos(deltaF)-Fylf*sin(deltaF)+Fxlr+Fxrf*cos(deltaF)-Fyrf*sin(deltaF)+Fxrr;
Ny = Fylf*cos(deltaF)+Fxlf*sin(deltaF)+Fylr+Fyrf*cos(deltaF)+Fxrf*sin(deltaF)+Fyrr;
Mz = W/2*(Fxrf*cos(deltaF)-Fyrf*sin(deltaF)+Fxrr)-...
     W/2*(Fxlf*cos(deltaF)-Fylf*sin(deltaF)+Fxlr)+...
     l_f*(Fylf*cos(deltaF)+Fxlf*sin(deltaF)+Fyrf*cos(deltaF)+Fxrf*sin(deltaF))-...
     l_r*(Fylr+Fyrr);

% state = [vx,vy,yaw,yawRate,X,Y]
xplus(1,1) = x(1)+(Nx/m+x(4)*x(2))*dt;
xplus(2,1) = x(2)+(Ny/m-x(4)*x(1))*dt;
xplus(3,1) = x(3)+x(4)*dt;
xplus(4,1) = x(4)+Mz/I_z*dt;
xplus(5,1) = x(5)+(x(1)*cos(x(3))-x(2)*sin(x(3)))*dt;
xplus(6,1) = x(6)+(x(1)*sin(x(3))+x(2)*cos(x(3)))*dt;