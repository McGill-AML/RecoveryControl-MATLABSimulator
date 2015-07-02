function dx = SpiriMotion(t,x,signal_c,wall_loc,wall_plane,flag_c,vB_normal,pB_contact,Fc_mag,pW_wall)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

global g m I Jr prop_loc Kt A d_air Cd V Tv Kp Kq Kr Dt Rb;

% persistent pt1_p Pc_w_p;


% T2 = RotMat('X',x(14))*RotMat('Y',x(15))*RotMat('Z',x(16));
q = [x(10);x(11);x(12);x(13)]/norm(x(10:13));
R = quatRotMat(q);

x = reshape(x,[max(size(x)),1]);

dx = zeros(16,1);
prop_speed = signal_c(1:4);
prop_accel = signal_c(5:8);


% pt1 = [0;0;0];
% pt2 = [0;0;0];
% Pc_w = [0;0;0];

defl = 0;

if abs(wall_loc - x(7)) <= Rb
    if (sum(wall_plane == 'YZ')==2 || sum(wall_plane == 'ZY')==2)
        bumper_n = R'*[0;0;1]; %transform to world frame
        bumper_u = R'*[1;0;0]; %transform to world frame
        bumper_C = R'*[0;0;prop_loc(3,1)] + x(7:9);
        
        syms theta
        bumper_cross = cross(bumper_n,bumper_u);
        
        Cx = bumper_C(1);
        alpha = bumper_u(1); 
        beta = bumper_cross(1);
        theta1 = -log((wall_loc - Cx + (Cx^2 - 2*Cx*wall_loc - Rb^2*alpha^2 - Rb^2*beta^2 + wall_loc^2)^(1/2))/(Rb*(alpha - beta*i)))*i;
        theta2 = -log(-(Cx - wall_loc + (Cx^2 - 2*Cx*wall_loc - Rb^2*alpha^2 - Rb^2*beta^2 + wall_loc^2)^(1/2))/(Rb*(alpha - beta*i)))*i;
        theta1_s = theta1;
        assignin('base','theta1_s',theta1_s);
%         disp('before solve');
%         S = solve (bumper_u(1)*Rb*cos(theta)+bumper_cross(1)*Rb*sin(theta)==wall_loc - bumper_C(1),theta);
%         disp('after solve');
%         S = eval(S);
%         disp(theta1)
%         if abs(imag(theta1)) <= eps(1)*2%isreal(S) == 1
            theta1 = real(theta1);
            theta2 = real(theta2);
            if theta1 == theta2%size(S,1) == 1 %1 pt of intersection
%                 disp('1 pt of intersection');
                  defl = 0;
            else %if size(S,1) == 2 %2 pts of intersection
%                 disp('2 pt of intersection');
%                 pt1 = Rb*cos(S(1))*bumper_u + Rb*sin(S(1))*bumper_cross + bumper_C;
%                 pt2 = Rb*cos(S(2))*bumper_u + Rb*sin(S(2))*bumper_cross + bumper_C;
                
                pt1 = Rb*cos(theta1)*bumper_u + Rb*sin(theta1)*bumper_cross + bumper_C;
                pt2 = Rb*cos(theta2)*bumper_u + Rb*sin(theta2)*bumper_cross + bumper_C;
                
                axis_c = (pt1+pt2)/2 - bumper_C;
%                 axis_c(2) = (pt1(2) + pt2(2))/2;
%                 axis_c(2) = 0;
%                 axis_c = axis_c/norm(axis_c);
                
                axis_c_b = R*axis_c;
                axis_c_b = axis_c_b/norm(axis_c_b);
                
%                 if bumper_C(1) <= wall_loc
                if pt1(1) >= bumper_C(1)
                    Pc_b = [0;0;prop_loc(3,1)] + Rb*axis_c_b;
%                     Pc_w = bumper_C + Rb*axis_c;
                else
                    Pc_b = [0;0;prop_loc(3,1)] - Rb*axis_c_b;
%                     Pc_w = bumper_C - Rb*axis_c;
                end
                  Pc_w = R'*Pc_b + x(7:9);
%                 axis_c_b = R*axis_c;
%                 axis_c_b = axis_c_b/norm(axis_c_b);
%                 
%                 Pc_b = Rb*axis_c_b;
%                 Pc_w = R'*Pc_b + bumper_C;
                defl = Pc_w(1) - wall_loc;
     
                
                if defl <= 0
                    defl = 0;
%                     disp(defl)
%                     error('Deflection calc error');
                end

                assignin('base','pt1',pt1);
                assignin('base','Pc_w',Pc_w);
                assignin('base','pt2',pt2);

        end %else no contact        
        assignin('base','defl',defl);
    end
end

% omega = sqrt(signal_c(1)/(4*-Kt));
% prop_speed = [omega;-omega;omega;-omega];
% prop_accel = zeros(4,1);
% 
% if flag_c == 1
% 
%     pW_contact = T'*pB_contact + [x(7);x(8);-x(9)];
%     defl_contact = sign(pW_contact(1)-pW_wall(1))*sum((pW_contact - pW_wall).^2);
%     if defl_contact > 0
if defl > 0
        Fc_mag = 1000000*defl^1.5;
% 
% %         vB_normal = T*[-1;0;0];            
% %         vB_normal = vB_normal/norm(vB_normal);
        Fc_w = [-Fc_mag;0;0];
        Fc_b = R*Fc_w;
%         
% %         Fc = [Fc_mag*(vB_normal'*[1;0;0]);Fc_mag*(vB_normal'*[0;1;0]);Fc_mag*(vB_normal'*[0;0;1])];
%         rc = Pc_b;
        Mc = cross(Pc_b,Fc_b);
%     else
%         defl_contact = 0;
%         Fc = [0;0;0];
%         Mc = [0;0;0];
%         flag_c = 0;
%     end
% else
%     defl_contact = 0;
%     Fc_b = [0;0;0];
%     Mc = [0;0;0];
% end
else
    Fc_b = [0;0;0];
    Mc = [0;0;0];
end

Fg = R*[0;0;-m*g];
Fa = Tv*[-0.5*d_air*V^2*A*Cd;0;0];
% Ft = [0;0;signal_c(1)];
Ft = [0;0;-Kt*sum(prop_speed.^2)];
% Ft = [0;0;-signal_c(1)];

% Mx = signal_c(2)-Kp*x(4)^2;%-x(5)*Jr*sum(prop_speed);
% My = signal_c(3)-Kq*x(5)^2;%+x(4)*Jr*sum(prop_speed);
% Mz = signal_c(4)-Kr*x(6)^2; %;-Jr*sum(prop_accel);

Mx = -Kt*prop_loc(2,:)*(prop_speed.^2)-Kp*x(4)^2-x(5)*Jr*sum(prop_speed) + Mc(1);
My = Kt*prop_loc(1,:)*(prop_speed.^2)-Kq*x(5)^2+x(4)*Jr*sum(prop_speed) + Mc(2);
Mz =  [-Dt Dt -Dt Dt]*(prop_speed.^2)-Kr*x(6)^2-Jr*sum(prop_accel) + Mc(3);

dx(1:3) = (Fg + Fa + Ft + Fc_b - m*cross(x(4:6),x(1:3)))/m;
dx(4:6) = inv(I)*([Mx;My;Mz]-cross(x(4:6),I*x(4:6)));
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

