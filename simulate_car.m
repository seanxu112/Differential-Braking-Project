function simulate_car(t, x, delta, x_min, x_max, y_min, y_max, traj_vec)

%t = time vector
%x = state vector
%delta = steering angle vector

%read current state
vx = x(1);
vy = x(2);
yaw = x(3)-pi/2; %rotate by 90 degrees to correct for body frame coordinates
yawRate = x(4);
X = x(5);
Y = x(6);

global l_f l_r W 
%W =  width
%l_f = distance between c.g. and front axle (m)
%l_r distance between c.g. and rear axle (m)

td = .6; %tire diameter
tw = .275; %tire width
a = l_f; %distance from COM to front axle
b = l_r; %distance from COM to rear axle
d = W; %distance between tires on one axle
bumper_fr = .5; %distance from axles to bumpers
bumper_s = .2; %distance from tires to side bumpers
% delta = .5; %steering angle
% yaw = -.3; %yaw

x_com = x(5); %x COM
y_com = x(6); %y COM
 
%Car outline dimensions

car_diag_angle =  atan( ( a + bumper_fr )/( d/2 + bumper_s ) );
car_diag = sqrt( ( d/2 + bumper_s )^2 + ( a + bumper_fr )^2 );
 
fl = car_diag * [cos(yaw - car_diag_angle + pi());sin(yaw - car_diag_angle + pi())] + [x_com; y_com];

fr = car_diag * [cos(yaw + car_diag_angle);sin(yaw + car_diag_angle)] + [x_com; y_com];

bl = car_diag * [cos(yaw + car_diag_angle + pi());sin(yaw + car_diag_angle + pi())] + [x_com; y_com];

br = car_diag * [cos(yaw - car_diag_angle);sin(yaw - car_diag_angle)] + [x_com; y_com];

car_outline = [fl';
       fr';
       br';
       bl';
       fl'];
   
%COM of tires locations   
   
tire_com_diag_angle = atan( ( a )/( d/2 ) );
tire_com_diag = sqrt( ( d/2 )^2 + ( a )^2 );
   
tire_com_fl = tire_com_diag * [cos(yaw - tire_com_diag_angle + pi());sin(yaw - tire_com_diag_angle + pi())] + [x_com; y_com];
tire_com_fr = tire_com_diag * [cos(yaw + tire_com_diag_angle);sin(yaw + tire_com_diag_angle)] + [x_com; y_com];
tire_com_bl = tire_com_diag * [cos(yaw + tire_com_diag_angle + pi());sin(yaw + tire_com_diag_angle + pi())] + [x_com; y_com];
tire_com_br = tire_com_diag * [cos(yaw - tire_com_diag_angle);sin(yaw - tire_com_diag_angle)] + [x_com; y_com];

%Outline of tires

tire_diag_angle = atan( (td/2)/(tw/2) );
tire_diag = sqrt( ( td/2 )^2 + ( tw/2 )^2 );

tire_fl_1 = tire_diag * [cos( - tire_diag_angle + pi() + delta + yaw);sin( - tire_diag_angle + pi() + delta + yaw)] + tire_com_fl;
tire_fl_2 = tire_diag * [cos( tire_diag_angle + delta + yaw );sin( tire_diag_angle + delta + yaw)] + tire_com_fl;
tire_fl_3 = tire_diag * [cos( - tire_diag_angle + delta + yaw );sin( - tire_diag_angle + delta + yaw )] + tire_com_fl;
tire_fl_4 = tire_diag * [cos( tire_diag_angle + pi() + delta + yaw);sin( tire_diag_angle + pi() + delta + yaw)] + tire_com_fl;

tire_fl = [tire_fl_1';
           tire_fl_2';
           tire_fl_3';
           tire_fl_4';
           tire_fl_1'];
       
tire_fr_1 = tire_diag * [cos( - tire_diag_angle + pi() + delta + yaw);sin( - tire_diag_angle + pi() + delta + yaw)] + tire_com_fr;
tire_fr_2 = tire_diag * [cos( tire_diag_angle + delta + yaw );sin( tire_diag_angle + delta + yaw )] + tire_com_fr;
tire_fr_3 = tire_diag * [cos( - tire_diag_angle + delta + yaw );sin( - tire_diag_angle + delta + yaw )] + tire_com_fr;
tire_fr_4 = tire_diag * [cos( tire_diag_angle + pi() + delta + yaw);sin( tire_diag_angle + pi() + delta + yaw)] + tire_com_fr;

tire_fr = [tire_fr_1';
           tire_fr_2';
           tire_fr_3';
           tire_fr_4';
           tire_fr_1'];
       
tire_bl_1 = tire_diag * [cos( - tire_diag_angle + pi() + yaw);sin( - tire_diag_angle + pi() + yaw)] + tire_com_bl;
tire_bl_2 = tire_diag * [cos( tire_diag_angle + yaw );sin( tire_diag_angle + yaw )] + tire_com_bl;
tire_bl_3 = tire_diag * [cos( - tire_diag_angle + yaw );sin( - tire_diag_angle + yaw )] + tire_com_bl;
tire_bl_4 = tire_diag * [cos( tire_diag_angle + pi() + yaw);sin( tire_diag_angle + pi() + yaw)] + tire_com_bl;

tire_bl = [tire_bl_1';
           tire_bl_2';
           tire_bl_3';
           tire_bl_4';
           tire_bl_1'];
       
tire_br_1 = tire_diag * [cos( - tire_diag_angle + pi() + yaw);sin( - tire_diag_angle + pi() + yaw)] + tire_com_br;
tire_br_2 = tire_diag * [cos( tire_diag_angle + yaw );sin( tire_diag_angle + yaw )] + tire_com_br;
tire_br_3 = tire_diag * [cos( - tire_diag_angle + yaw );sin( - tire_diag_angle + yaw )] + tire_com_br;
tire_br_4 = tire_diag * [cos( tire_diag_angle + pi() + yaw);sin( tire_diag_angle + pi() + yaw)] + tire_com_br;

tire_br = [tire_br_1';
           tire_br_2';
           tire_br_3';
           tire_br_4';
           tire_br_1'];       

%Plot       
axis_min_y = -2;
axis_max_y = 30;
axis_min_x = -2;
axis_max_x = 20;

global marker

figure(5)
plot(car_outline(:,1),car_outline(:,2),'b')
hold on
axis equal
plot(tire_fl(:,1),tire_fl(:,2),'r')
plot(tire_fr(:,1),tire_fr(:,2),'r')
plot(tire_bl(:,1),tire_bl(:,2),'r')
plot(tire_br(:,1),tire_br(:,2),'r')
plot(traj_vec(5,:),traj_vec(6,:),'b')
% [rows, col] = size(traj_vec);
% 
% j_end = rows/6;
% 
% for j = 1:j_end
%     
%     traj_vec_ = traj_vec(j*6,:);
% 
%     plot(traj_vec_(5,:),traj_vec_(6,:))
%     
% end

ylim([fix(y_min)-5 , fix(y_max)+5])
xlim([fix(x_min)-5 , fix(x_max)+5])
grid on

set(gcf,'units','normalized','outerposition',[0 0 1 1])

hold off

if marker == 1
    
    pause(10);
    
    marker = 0;
    
end

end