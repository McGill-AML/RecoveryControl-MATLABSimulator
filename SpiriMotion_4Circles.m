function [dx, defl1, Fc_mag1, pc_w1, defl_rate1, defl2, Fc_mag2, pc_w2, defl_rate2] = SpiriMotion_4Circles(t,x,signal_c,wall_loc,wall_plane)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

global g m I Jr prop_loc Kt A d_air Cd V Tv Kp Kq Kr Dt Rbumper Cbumper Ixx Iyy Izz CM;

global flag_c_fine1 vi_c_fine1 flag_c_fine2 vi_c_fine2;

q = [x(10);x(11);x(12);x(13)]/norm(x(10:13));
R = quatRotMat(q);

x = reshape(x,[max(size(x)),1]);

dx = zeros(13,1);

%% Controller Signal
prop_speed = signal_c(1:4); %in RPM
prop_accel = signal_c(5:8); %in rad/s^2

prop_speed_rad = prop_speed * (2*pi/60); %in rad/s

%% Contact Detection
pc_w1 = [100;100;0];
defl1 = 0;
defl_rate1 = 0;

pc_w2 = [100;100;0];
defl2 = 0;
defl_rate2 = 0;


if abs(wall_loc - x(7)) <= 0.3 %Rbumper + sqrt(max(abs(prop_loc(1,:)))^2 + max(abs(prop_loc(2,:)))^2)
    if (sum(wall_plane == 'YZ')==2 || sum(wall_plane == 'ZY')==2)
        [ defl,defl_rate,pc_b,pc_w, pint1, pint2, theta1, theta2] = FindContact( R,x,wall_loc);
        % Save variables to base workspace
        assignin('base','pint11',pint1(:,1));
        assignin('base','pc_w1',pc_w(:,1));
        assignin('base','pint12',pint2(:,1));
        assignin('base','theta11',theta1(1));
        assignin('base','theta12',theta2(1));
        
        assignin('base','pint21',pint1(:,2));
        assignin('base','pc_w2',pc_w(:,2));
        assignin('base','pint22',pint2(:,2));
        assignin('base','theta21',theta1(2));
        assignin('base','theta22',theta2(2));
        
        defl1 = defl(1);
        defl2 = defl(2);
        defl_rate1 = defl_rate(1);
        defl_rate2 = defl_rate(2);
        pc_w1 = pc_w(:,1);
        pc_w2 = pc_w(:,2);
        
    end
end
assignin('base','defl1',defl1);
assignin('base','defl2',defl2);

% k_c = 1*10^5;    
% e_c = 0.95;
% n_c = 1.5;

k_c = 40;    
e_c = 0.82;
n_c = 0.54;

%% Calculate contact force and moment
if defl1 > 0
    
    if flag_c_fine1 == 0
        flag_c_fine1 = 1;
        vi_c_fine1 = sqrt(sum(x(1:3).^2));
    end
    
%     Fc_mag = 5*10^2*defl^1.5;
    
    lambda_c1 = 6*(1-e_c)*k_c/(((2*e_c-1)^2+3)*vi_c_fine1);    
    Fc_mag1 = k_c*defl1^n_c + lambda_c1*defl1^n_c*defl_rate1;

    Fc_w1 = [-Fc_mag1;0;0];
    Fc_b1 = R*Fc_w1;
        
    Mc1 = cross(pc_b(:,1),Fc_b1);
else
    if flag_c_fine1 == 1
        flag_c_fine1 = 0;
        vi_c_fine1 = 0;
    end
    
    Fc_mag1 = 0;
    Fc_b1 = [0;0;0];
    Mc1 = [0;0;0];
end

if defl2 > 0
    
    if flag_c_fine2 == 0
        flag_c_fine2 = 1;
        vi_c_fine2 = sqrt(sum(x(1:3).^2));
    end
    
%     Fc_mag = 5*10^2*defl^1.5;
    
    lambda_c2 = 6*(1-e_c)*k_c/(((2*e_c-1)^2+3)*vi_c_fine2);    
    Fc_mag2 = k_c*defl2^n_c + lambda_c2*defl2^n_c*defl_rate2;

    Fc_w2 = [-Fc_mag2;0;0];
    Fc_b2 = R*Fc_w2;
        
    Mc2 = cross(pc_b(:,2),Fc_b2);
else
    if flag_c_fine2 == 1
        flag_c_fine2 = 0;
        vi_c_fine2 = 0;
    end
    
    Fc_mag2 = 0;
    Fc_b2 = [0;0;0];
    Mc2 = [0;0;0];
end

% No Contact:
% Fc_mag1 = 0;
% Fc_b1 = [0;0;0];
% Mc1 = [0;0;0];
% Fc_mag2 = 0;
% Fc_b2 = [0;0;0];
% Mc2 = [0;0;0];

Fg = R*[0;0;-m*g];
Fa = Tv*[-0.5*d_air*V^2*A*Cd;0;0];
Ft = [0;0;-Kt*sum(prop_speed.^2)];

assignin('base','prop_speed',prop_speed);
assignin('base','prop_accel',prop_accel);

% Mx = signal_c(2)-Kp*x(4)^2;%-x(5)*Jr*sum(prop_speed);
% My = signal_c(3)-Kq*x(5)^2;%+x(4)*Jr*sum(prop_speed);
% Mz = signal_c(4)-Kr*x(6)^2; %;-Jr*sum(prop_accel);
% 
% prop_speed = prop_speed2;
% prop_speed_rad = prop_speed2_rad;

Mx = -Kt*prop_loc(2,:)*(prop_speed.^2)-Kp*x(4)^2-x(5)*Jr*sum(prop_speed_rad) + Mc1(1) + Mc2(1);
My = Kt*prop_loc(1,:)*(prop_speed.^2)-Kq*x(5)^2+x(4)*Jr*sum(prop_speed_rad) + Mc1(2) + Mc2(2);
Mz =  [-Dt Dt -Dt Dt]*(prop_speed.^2)-Kr*x(6)^2 -Jr*sum(prop_accel) + Mc1(3) + Mc2(3);


dx(1:3) = (Fg + Fa + Ft + Fc_b1 + Fc_b2 - m*cross(x(4:6),x(1:3)))/m;
dx(4:6) = inv(I)*([Mx;My;Mz]-cross(x(4:6),I*x(4:6)));
dx(7:9) = R'*x(1:3);
dx(10:13) = -0.5*quatmultiply([0;x(4:6)],q);

% dx(14) = [1 sin(x(14))*tan(x(15)) cos(x(14))*tan(x(15))]*x(4:6);
% dx(15) = [0 cos(x(14)) -sin(x(14))]*x(4:6);
% dx(16) = [0 sin(x(14))/cos(x(15)) cos(x(14))/cos(x(15))]*x(4:6); 


end

