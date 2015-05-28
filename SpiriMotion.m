function [ dx ] = SpiriMotion(t,x,signal_c)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

global g m I Jr prop_loc Kt A d_air Cd V Tv Kp Kq Kr Dt;

T2 = RotMat('X',x(14))*RotMat('Y',x(15))*RotMat('Z',x(16));
q = [x(10);x(11);x(12);x(13)]/norm(x(10:13));
T = quatRotMat(q);

dx = zeros(16,1);
prop_speed = signal_c(1:4);
prop_accel = signal_c(5:8);

Fg = T*[0;0;m*g];
Fa = Tv*[-0.5*d_air*V^2*A*Cd;0;0];
% Ft = [0;0;signal_c(1)];
Ft = [0;0;-Kt*sum(prop_speed.^2)];

Mx = -Kt*prop_loc(2,:)*(prop_speed.^2)-Kp*x(4)^2-x(5)*Jr*sum(prop_speed);
My =  Kt*prop_loc(1,:)*(prop_speed.^2)-Kq*x(5)^2+x(4)*Jr*sum(prop_speed);
Mz =  [Dt -Dt Dt -Dt]*(prop_speed.^2)-Kr*x(6)^2-Jr*sum(prop_accel);

% Mx = signal_c(2)-Kp*x(4)^2-x(5)*Jr*sum(prop_speed);
% My = signal_c(3)-Kq*x(5)^2+x(4)*Jr*sum(prop_speed);
% Mz = signal_c(4)-Kr*x(6)^2; %;-Jr*sum(prop_accel);

dx(1:3) = (Fg + Fa + Ft - m*cross(x(4:6),x(1:3)))/m;
dx(4:6) = inv(I)*([Mx;My;Mz]-cross(x(4:6),I*x(4:6)));
dx(7:9) = T'*x(1:3);
dx(10:13) = -0.5*quatmultiply([0;x(4:6)],q);

dx(14) = [1 sin(x(14))*tan(x(15)) cos(x(14))*tan(x(15))]*x(4:6);
dx(15) = [0 cos(x(14)) -sin(x(14))]*x(4:6);
dx(16) = [0 sin(x(14))/cos(x(15)) cos(x(14))/cos(x(15))]*x(4:6); 

% disp('T quaternion:');
% disp(T);
% disp('T euler:');
% disp(T2);

end

