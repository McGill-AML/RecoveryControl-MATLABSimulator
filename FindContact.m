function [ defl,defl_rate,pc_b,pc_w, pint1, pint2, theta1, theta2] = FindContact( R,x,wall_loc)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
global Cbumper Rbumper prop_loc Abumper

defl = [0;0];
defl_rate = [0;0];
pc_b = [[0;0;0],[0;0;0]];
pc_w = [[100;100;0],[100;100;0]];

pint1 = [[100;100;0],[100;100;0]];
pint2 = [[100;100;0],[100;100;0]];
theta1 = [0;0];
theta2 = [0;0];

contact_idx = 1;

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

for k = 1:4
    if contact_idx <= 2
    % Transform virtual bumper definitions to world frame
        bumper_n = R'*n_b(:,k); %normal to bumper circle
        bumper_u = R'*u_b(:,k); %in plane with bumper circle
    %     bumper_u = R'*([1;1;0]/norm([1;1;0])); %in plane with bumper circle
        bumper_loc = R'*prop_loc(:,k) + x(7:9);

        % Solve for intersection angle
        bumper_cross = cross(bumper_n,bumper_u);

        theta1(contact_idx) = -log((wall_loc - bumper_loc(1) + (bumper_loc(1)^2 - 2*bumper_loc(1)*wall_loc - Rbumper^2*bumper_u(1)^2 - Rbumper^2*bumper_cross(1)^2 + wall_loc^2)^(1/2))/(Rbumper*(bumper_u(1) - bumper_cross(1)*1i)))*1i;
        theta2(contact_idx) = -log(-(bumper_loc(1) - wall_loc + (bumper_loc(1)^2 - 2*bumper_loc(1)*wall_loc - Rbumper^2*bumper_u(1)^2 - Rbumper^2*bumper_cross(1)^2 + wall_loc^2)^(1/2))/(Rbumper*(bumper_u(1) - bumper_cross(1)*1i)))*1i;

        % Save intersection angle to base workspace


        if abs(imag(theta1(contact_idx))) <= 1e-5
            theta1(contact_idx) = real(theta1(contact_idx));
            theta2(contact_idx) = real(theta2(contact_idx));

            % Calculate point of contact, deflection
            if theta1(contact_idx) == theta2(contact_idx) %1 pt of intersection
                defl(contact_idx) = 0;

            else %2 pts of intersection

                % Find points of intersection
                pint1(:,contact_idx) = Rbumper*cos(theta1(contact_idx))*bumper_u + Rbumper*sin(theta1(contact_idx))*bumper_cross + bumper_loc;
                pint2(:,contact_idx) = Rbumper*cos(theta2(contact_idx))*bumper_u + Rbumper*sin(theta2(contact_idx))*bumper_cross + bumper_loc;

                % Find point of contact
                axisc_w = (pint1(:,contact_idx)+pint2(:,contact_idx))/2 - bumper_loc; %axis pc lies on
                axisc_b = R*axisc_w;
                axisc_b = axisc_b/norm(axisc_b);

                if pint1(1,contact_idx) >= bumper_loc(1)
                    pc_b(:,contact_idx) = prop_loc(:,k) + Rbumper*axisc_b;
                else
                    pc_b(:,contact_idx) = prop_loc(:,k) - Rbumper*axisc_b;
                end

                pc_w(:,contact_idx) = real(R'*pc_b(:,contact_idx) + x(7:9));

                % Find deflection
                defl(contact_idx) = pc_w(1,contact_idx) - wall_loc;

                % Find deflection rate
                defl_rate(contact_idx) = R(:,1)'*([x(1);x(2);x(3)] + cross([x(4);x(5);x(6)],pc_b(:,contact_idx)));

                contact_idx = contact_idx + 1;
                if defl(contact_idx-1) <= 0     
                    contact_idx = contact_idx - 1;
                    disp('Warning: Deflection calc error');
                    disp(defl(contact_idx));
                    defl(contact_idx) = 0;
                    pc_b(:,contact_idx) = [0;0;0];
                    pc_w(:,contact_idx) = [100;100;0];
                    defl_rate(contact_idx) = 0;

                end

            end %else no contact


        end

    end
end



end

