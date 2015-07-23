function [dx, defl, Fc_mag, pc_w, defl_rate] = SpiriMotion(t,x,signal_c,wall_loc,wall_plane)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

global g m I Jr prop_loc Kt A d_air Cd V Tv Kp Kq Kr Dt Rbumper Cbumper Ixx Iyy Izz CM;

global flag_c_fine vi_c_fine;

q = [x(10);x(11);x(12);x(13)]/norm(x(10:13));
R = quatRotMat(q);

x = reshape(x,[max(size(x)),1]);

dx = zeros(13,1);

%% Controller Signal
prop_speed = signal_c(1:4); %in RPM
prop_accel = signal_c(5:8); %in rad/s^2

prop_speed_rad = prop_speed * (2*pi/60); %in rad/s

%% Contact Detection
pc_w = [100;100;0];
defl = 0;
defl_rate = 0;
if abs(wall_loc - x(7)) <= Rbumper
    if (sum(wall_plane == 'YZ')==2 || sum(wall_plane == 'ZY')==2)
        
        % Transform virtual bumper definitions to world frame
        bumper_n = R'*[0;0;1]; %normal to bumper circle
        bumper_u = R'*[1;0;0]; %in plane with bumper circle
        bumper_loc = R'*Cbumper + x(7:9);
        
        % Solve for intersection angle
        bumper_cross = cross(bumper_n,bumper_u);
        
        theta1 = -log((wall_loc - bumper_loc(1) + (bumper_loc(1)^2 - 2*bumper_loc(1)*wall_loc - Rbumper^2*bumper_u(1)^2 - Rbumper^2*bumper_cross(1)^2 + wall_loc^2)^(1/2))/(Rbumper*(bumper_u(1) - bumper_cross(1)*i)))*i;
        theta2 = -log(-(bumper_loc(1) - wall_loc + (bumper_loc(1)^2 - 2*bumper_loc(1)*wall_loc - Rbumper^2*bumper_u(1)^2 - Rbumper^2*bumper_cross(1)^2 + wall_loc^2)^(1/2))/(Rbumper*(bumper_u(1) - bumper_cross(1)*i)))*i;
        
        % Save intersection angle to base workspace
        assignin('base','theta1',theta1);

        if abs(imag(theta1)) <= 1e-5
            theta1 = real(theta1);
            theta2 = real(theta2);
        
            % Calculate point of contact, deflection
            if theta1 == theta2 %1 pt of intersection
                defl = 0;
                
            else %2 pts of intersection

                % Find points of intersection
                pint1 = Rbumper*cos(theta1)*bumper_u + Rbumper*sin(theta1)*bumper_cross + bumper_loc;
                pint2 = Rbumper*cos(theta2)*bumper_u + Rbumper*sin(theta2)*bumper_cross + bumper_loc;

                % Find point of contact
                axisc_w = (pint1+pint2)/2 - bumper_loc; %axis pc lies on
                axisc_b = R*axisc_w;
                axisc_b = axisc_b/norm(axisc_b);

                if pint1(1) >= bumper_loc(1)
                    pc_b = [0;0;prop_loc(3,1)] + Rbumper*axisc_b;
                else
                    pc_b = [0;0;prop_loc(3,1)] - Rbumper*axisc_b;
                end

                pc_w = R'*pc_b + x(7:9);

                % Find deflection
                defl = pc_w(1) - wall_loc;
                
                % Find deflection rate
                defl_rate = R(:,1)'*([x(1);x(2);x(3)] + cross([x(4);x(5);x(6)],pc_b));

                if defl <= 0     

                    disp('Warning: Deflection calc error');
                    disp(defl);
                    defl = 0;
                    pc_b = [0;0;0];
                    pc_w = [100;100;0];
                    defl_rate = 0;

                end

                % Save variables to base workspace
                assignin('base','pint1',pint1);
                assignin('base','pc_w',pc_w);
                assignin('base','pint2',pint2);

            end %else no contact
            
        end
        
        if defl == 0 %Prevent Spiri from moving through walls
            
            usb_b = [0.02353;-CM(2);-(0.04047+0.006)];
            usb_w = R'*usb_b + x(7:9);
            
            if usb_w(1) >= wall_loc %Case where contact circle is fully vertical
                pc_b = usb_b;
                pc_w = usb_w;
                defl = pc_w(1) - wall_loc;
                defl_rate = R(:,1)'*([x(1);x(2);x(3)] + cross([x(4);x(5);x(6)],pc_b));
                disp('Contact pt @ USB');
                
            end
        end
    end
end

assignin('base','defl',defl);

%% Calculate contact force and moment
if defl > 0
    
    if flag_c_fine == 0
        flag_c_fine = 1;
        vi_c_fine = sqrt(sum(x(1:3).^2));
    end
    
%     Fc_mag = 5*10^2*defl^1.5;
    k_c = 1*10^5;    
    e_c = 0.95;
    n_c = 1.5;
    lambda_c = 6*(1-e_c)*k_c/(((2*e_c-1)^2+3)*vi_c_fine);
    
    Fc_mag = k_c*defl^n_c + lambda_c*defl^n_c*defl_rate;

    Fc_w = [-Fc_mag;0;0];
    Fc_b = R*Fc_w;
        
    Mc = cross(pc_b,Fc_b);
else
    if flag_c_fine == 1
        flag_c_fine = 0;
        vi_c_fine = 0;
    end
    
    Fc_mag = 0;
    Fc_b = [0;0;0];
    Mc = [0;0;0];
end

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

Mx = -Kt*prop_loc(2,:)*(prop_speed.^2)-Kp*x(4)^2-x(5)*Jr*sum(prop_speed_rad) + Mc(1);
My = Kt*prop_loc(1,:)*(prop_speed.^2)-Kq*x(5)^2+x(4)*Jr*sum(prop_speed_rad) + Mc(2);
Mz =  [-Dt Dt -Dt Dt]*(prop_speed.^2)-Kr*x(6)^2 -Jr*sum(prop_accel) + Mc(3);


dx(1:3) = (Fg + Fa + Ft + Fc_b - m*cross(x(4:6),x(1:3)))/m;
dx(4:6) = inv(I)*([Mx;My;Mz]-cross(x(4:6),I*x(4:6)));
dx(7:9) = R'*x(1:3);
dx(10:13) = -0.5*quatmultiply([0;x(4:6)],q);

% dx(14) = [1 sin(x(14))*tan(x(15)) cos(x(14))*tan(x(15))]*x(4:6);
% dx(15) = [0 cos(x(14)) -sin(x(14))]*x(4:6);
% dx(16) = [0 sin(x(14))/cos(x(15)) cos(x(14))/cos(x(15))]*x(4:6); 


end

