function [ pt1,pt2,Pc_w] = DetectContact1( x,wall_loc,wall_plane)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

% global g m I Jr prop_loc Kt A d_air Cd V Tv Kp Kq Kr Dt Rb;

global prop_loc Rb;

q = [x(10);x(11);x(12);x(13)]/norm(x(10:13));
R = quatRotMat(q);

x = reshape(x,[max(size(x)),1]);

pt1 = [0;0;0];
pt2 = [0;0;0];
Pc_w = [0;0;0];
S = 10000;

if (wall_loc - x(7)) <= Rb
    if (sum(wall_plane == 'YZ')==2 || sum(wall_plane == 'ZY')==2)
        bumper_n = R'*[0;0;1]; %transform to world frame
        bumper_u = R'*[1;0;0]; %transform to world frame
        bumper_C = R'*[0;0;prop_loc(3,1)] + x(7:9);
        
        syms theta
        bumper_cross = cross(bumper_n,bumper_u);
%         disp('before solve');
        S = solve (bumper_u(1)*Rb*cos(theta)+bumper_cross(1)*Rb*sin(theta)==wall_loc - bumper_C(1),theta);
%         disp('after solve');
        S = eval(S);

        if isreal(S) == 1
            if size(S,1) == 1 %1 pt of intersection
                disp('1 pt of intersection');
            elseif size(S,1) == 2 %2 pts of intersection
%                 disp('2 pt of intersection');
                pt1 = Rb*cos(S(1))*bumper_u + Rb*sin(S(1))*bumper_cross + bumper_C;
                pt2 = Rb*cos(S(2))*bumper_u + Rb*sin(S(2))*bumper_cross + bumper_C;
                
                axis_c = pt1 - bumper_C;
                axis_c(2) = 0;
                axis_c = axis_c/norm(axis_c);
                
                Pc_b = Rb*R*axis_c;
                Pc_w = R'*Pc_b + bumper_C;
                
%                 if bumper_C(1) <= wall_loc                
                    defl = Pc_w(1) - wall_loc;

                if defl < 0
                    error('Deflection calc error');
                end
%     
%                 pt1_b = R*pt1;
%                 pt2_b = R*pt2;
                if sum(Pc_w==[0;0;0])==3
                    disp(S);
                end
                
            else
                error('Error in calculating intersection between bumper circle and wall');
            end

        end %else no contact
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


end


