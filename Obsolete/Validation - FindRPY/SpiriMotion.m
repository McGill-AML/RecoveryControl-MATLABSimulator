function [ dx,defl_contact, Fc_mag ] = SpiriMotion(t,x,signal_c,flag_c,vB_normal,pB_contact,Fc_mag,pW_wall)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

global g m I Jr prop_loc Kt A d_air Cd V Tv Kp Kq Kr Dt;


% T2 = RotMat('X',x(14))*RotMat('Y',x(15))*RotMat('Z',x(16));
q = [x(10);x(11);x(12);x(13)]/norm(x(10:13));
R = quatRotMat(q);

dx = zeros(16,1);
% prop_speed = signal_c(1:4);
% prop_accel = signal_c(5:8);
% omega = sqrt(signal_c(1)/4);
% prop_speed = [omega;-omega;omega;-omega];
% prop_accel = [0;0;0;0];
% 
% if flag_c == 1
% 
%     pW_contact = T'*pB_contact + [x(7);x(8);-x(9)];
%     defl_contact = sign(pW_contact(1)-pW_wall(1))*sum((pW_contact - pW_wall).^2);
%     if defl_contact > 0
%         Fc_mag = 1000000*defl_contact^1.5;
% 
% %         vB_normal = T*[-1;0;0];            
% %         vB_normal = vB_normal/norm(vB_normal);
%         FW_c = [-Fc_mag;0;0];
%         Fc = T*FW_c;
%         
% %         Fc = [Fc_mag*(vB_normal'*[1;0;0]);Fc_mag*(vB_normal'*[0;1;0]);Fc_mag*(vB_normal'*[0;0;1])];
%         rc = pB_contact;
%         Mc = cross(rc,Fc);
%     else
%         defl_contact = 0;
%         Fc = [0;0;0];
%         Mc = [0;0;0];
%         flag_c = 0;
%     end
% else
    defl_contact = 0;
    Fc = [0;0;0];
    Mc = [0;0;0];
% end


% Fg = R*[0;0;-m*g];
% Fa = Tv*[-0.5*d_air*V^2*A*Cd;0;0];
% % Ft = [0;0;signal_c(1)];
% Ft = [0;0;-Kt*sum(prop_speed.^2)];
% % Ft = [0;0;-signal_c(1)];
% 
% Mx = -Kt*prop_loc(2,:)*(prop_speed.^2)-Kp*x(4)^2-x(5)*Jr*sum(prop_speed) + Mc(1);
% My =  Kt*prop_loc(1,:)*(prop_speed.^2)-Kq*x(5)^2+x(4)*Jr*sum(prop_speed) + Mc(2);
% Mz =  [Dt -Dt Dt -Dt]*(prop_speed.^2)-Kr*x(6)^2-Jr*sum(prop_accel) + Mc(3);

% Mx = signal_c(2)-Kp*x(4)^2-x(5)*Jr*sum(prop_speed);
% My = signal_c(3)-Kq*x(5)^2+x(4)*Jr*sum(prop_speed);
% Mz = signal_c(4)-Kr*x(6)^2; %;-Jr*sum(prop_accel);

dx(1:3) = [0;0;0]; %(Fg + Fa + Ft + Fc - m*cross(x(4:6),x(1:3)))/m;
dx(4:6) = [0;0;0]; %inv(I)*([Mx;My;Mz]-cross(x(4:6),I*x(4:6)));
dx(7:9) = R'*x(1:3);
dx(10:13) = -0.5*quatmultiply([0;x(4:6)],q);

dx(14) = [1 sin(x(14))*tan(x(15)) cos(x(14))*tan(x(15))]*x(4:6);
dx(15) = [0 cos(x(14)) -sin(x(14))]*x(4:6);
dx(16) = [0 sin(x(14))/cos(x(15)) cos(x(14))/cos(x(15))]*x(4:6); 

% disp('T quaternion:');
% disp(T);
% disp('T euler:');
% disp(T2);

end

