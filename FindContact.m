% function [ defl_out,defl_rate_out,pc_b_out,pc_w_out, pint1_out, pint2_out, theta1_out, theta2_out] = FindContact( R,x,wall_loc)
function [ defl,defl_rate,pc_b,pc_w, pint1, pint2, theta1, theta2, sliding_axis] = FindContact( R,x,wall_loc)

%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
global Cbumper Rbumper prop_loc Abumper

defl = [0;0;0;0];
defl_rate = [0;0;0;0];
pc_b = zeros(3,4);
pc_w = repmat([100;100;0],1,4);
pint1 = repmat([100;100;0],1,4);
pint2 = repmat([100;100;0],1,4);
theta1 = [0;0;0;0];
theta2 = [0;0;0;0];

n_b = zeros(3,4);
u_b = zeros(3,4);

n_b(:,1) = RotMat('Z',deg2rad(45))'*RotMat('Y',Abumper + deg2rad(90))'* [1;0;0];
n_b(:,2) = RotMat('Z',deg2rad(135))'*RotMat('Y',Abumper + deg2rad(90))'* [1;0;0];
n_b(:,3) = RotMat('Z',deg2rad(-135))'*RotMat('Y',Abumper + deg2rad(90))'* [1;0;0];
n_b(:,4) = RotMat('Z',deg2rad(-45))'*RotMat('Y',Abumper + deg2rad(90))'* [1;0;0];

u_b(:,1) = RotMat('Z',deg2rad(45))'*RotMat('Y',Abumper)'*[1;0;0];
u_b(:,2) = RotMat('Z',deg2rad(135))'*RotMat('Y',Abumper)'*[1;0;0];
u_b(:,3) = RotMat('Z',deg2rad(-135))'*RotMat('Y',Abumper)'*[1;0;0];
u_b(:,4) = RotMat('Z',deg2rad(-45))'*RotMat('Y',Abumper)'*[1;0;0];

sliding_axis = repmat([0;0;0],1,4);

for k = 1:4

    % Transform virtual bumper definitions to world frame
        bumper_n = R'*n_b(:,k); %normal to bumper circle
        bumper_u = R'*u_b(:,k); %in plane with bumper circle
    %     bumper_u = R'*([1;1;0]/norm([1;1;0])); %in plane with bumper circle
        bumper_loc = R'*prop_loc(:,k) + x(7:9);

        % Solve for intersection angle
        bumper_cross = cross(bumper_n,bumper_u);

        theta1(k) = -log((wall_loc - bumper_loc(1) + (bumper_loc(1)^2 - 2*bumper_loc(1)*wall_loc - Rbumper^2*bumper_u(1)^2 - Rbumper^2*bumper_cross(1)^2 + wall_loc^2)^(1/2))/(Rbumper*(bumper_u(1) - bumper_cross(1)*1i)))*1i;
        theta2(k) = -log(-(bumper_loc(1) - wall_loc + (bumper_loc(1)^2 - 2*bumper_loc(1)*wall_loc - Rbumper^2*bumper_u(1)^2 - Rbumper^2*bumper_cross(1)^2 + wall_loc^2)^(1/2))/(Rbumper*(bumper_u(1) - bumper_cross(1)*1i)))*1i;

        % Save intersection angle to base workspace


        if abs(imag(theta1(k))) <= 1e-5
            theta1(k) = real(theta1(k));
            theta2(k) = real(theta2(k));

            % Calculate point of contact, deflection
            if theta1(k) == theta2(k) %1 pt of intersection
                defl(k) = 0;

            else %2 pts of intersection

                % Find points of intersection
                pint1(:,k) = Rbumper*cos(theta1(k))*bumper_u + Rbumper*sin(theta1(k))*bumper_cross + bumper_loc;
                pint2(:,k) = Rbumper*cos(theta2(k))*bumper_u + Rbumper*sin(theta2(k))*bumper_cross + bumper_loc;

                % Find point of contact
                axisc_w = (pint1(:,k)+pint2(:,k))/2 - bumper_loc; %axis pc lies on
                axisc_b = R*axisc_w;
                axisc_b = axisc_b/norm(axisc_b);

                if pint1(1,k) >= bumper_loc(1)
                    pc_b(:,k) = prop_loc(:,k) + Rbumper*axisc_b;
                else
                    pc_b(:,k) = prop_loc(:,k) - Rbumper*axisc_b;
                end

                pc_w(:,k) = real(R'*pc_b(:,k) + x(7:9));

                % Find deflection
                defl(k) = pc_w(1,k) - wall_loc;

                % Find deflection rate
                defl_rate(k) = R(:,1)'*([x(1);x(2);x(3)] + cross([x(4);x(5);x(6)],pc_b(:,k)));
                sliding_axis_temp = R'*([x(1);x(2);x(3)] + cross([x(4);x(5);x(6)],pc_b(:,k)));
                sliding_axis_temp = R*[0;sliding_axis_temp(2:3)];
                sliding_axis(:,k) = sliding_axis_temp/norm(sliding_axis_temp);
                
                if defl(k) <= 0     
                    disp('Warning: Deflection calc error');
                    disp(defl(k));
                    defl(k) = 0;
                    pc_b(:,k) = [0;0;0];
                    pc_w(:,k) = [100;100;0];
                    defl_rate(k) = 0;

                end

            end %else no contact


        end

end
% 
% [~,defl_idx] = sort(defl);
% defl_out = [defl(defl_idx(4));defl(defl_idx(3));defl(defl_idx(2));defl(defl_idx(1))];
% defl_rate_out = [defl_rate(defl_idx(4));defl_rate(defl_idx(3));defl_rate(defl_idx(2));defl_rate(defl_idx(1))];
% pc_b_out = [pc_b(:,defl_idx(4)),pc_b(:,defl_idx(3)),pc_b(:,defl_idx(2)),pc_b(:,defl_idx(1))];
% pc_w_out = [pc_w(:,defl_idx(4)),pc_w(:,defl_idx(3)),pc_w(:,defl_idx(2)),pc_w(:,defl_idx(1))];
% pint1_out = [pint1(:,defl_idx(4)),pint1(:,defl_idx(3)),pint1(:,defl_idx(2)),pint1(:,defl_idx(1))];
% pint2_out = [pint2(:,defl_idx(4)),pint2(:,defl_idx(3)),pint2(:,defl_idx(2)),pint2(:,defl_idx(1))];
% theta1_out = [theta1(defl_idx(4));theta1(defl_idx(3));theta1(defl_idx(2));theta1(defl_idx(1))];
% theta2_out = [theta2(defl_idx(4));theta2(defl_idx(3));theta2(defl_idx(2));theta2(defl_idx(1))];


end

