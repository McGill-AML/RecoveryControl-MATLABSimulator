function [Ti, Tc, defl_max, Fc_max, Vi, Vx, Vy, Vz, Roll, Pitch, Tilt, numContacts] = ContactStats( t_tot,X_tot,defl_tot, Fc_tot,vi_c_tot,traj_head)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

flag_c = 0;
idx_c = 1;

Ti = 0;
Tc = 0;
defl_max = 0;
Fc_max = 0;
Vi = 0;
Vx = 0;
Vy = 0;
Vz = 0;
Roll = 0;
Pitch = 0;
Tilt = 0;

for i = 1:size(t_tot)
    
   
    if defl_tot(i) == 0
        if flag_c == 1
            flag_c = 0;
            tf_indiv = t_tot(i);
            tc_indiv = tf_indiv - ti_indiv;
            Tc(idx_c) = tc_indiv;
            defl_max(idx_c) = defl_max_indiv;
            Fc_max(idx_c) = Fc_max_indiv;
            
            idx_c = idx_c + 1;
        end
    else
        if flag_c == 0
            flag_c = 1;
            ti_indiv = t_tot(i);
            defl_max_indiv = defl_tot(i);
            Fc_max_indiv = Fc_tot(i);
            Ti(idx_c) = ti_indiv;
            Vi(idx_c) = vi_c_tot(i);
            
            q = [X_tot(i,10);X_tot(i,11);X_tot(i,12);X_tot(i,13)]/norm(X_tot(i,10:13));
            R = quatRotMat(q);
            [roll, pitch, yaw] = quat2angle(q,'xyz');
%             axis_y1 = RotMat('Z',yaw)*[0;1;0];
%             axis_x2 = RotMat('Y',pitch)*RotMat('Z',yaw)*[1;1;0];    
            axis_y1 = RotMat('Z',yaw)'*[0;1;0];
            axis_x2 = RotMat('Y',pitch)'*RotMat('Z',yaw)'*[1;1;0];
            axis_zw = [0;0;1];
            
            V_w = R'*([X_tot(i,1);X_tot(i,2);X_tot(i,3)]);
            Vx(idx_c) = V_w(1); 
            Vy(idx_c) = V_w(2);
            Vz(idx_c) = V_w(3);
            Roll(idx_c) = roll;
            Pitch(idx_c) = pitch;
            Yaw(idx_c) = yaw;
            Tilt(idx_c) = FindTilt(R,traj_head);
        else
            defl_max_indiv = max(defl_max_indiv,defl_tot(i));
            Fc_max_indiv = max(Fc_max_indiv,Fc_tot(i));
            
        end
    end
end

if Ti(1) == 0
    display('No impact for this bumper');
    numContacts = 0;
else
    display('First Impact Information');
    display('------------------------');
    display(['   Init. Impact Time:', blanks(14), num2str(Ti(1)), '[s]']);
    display(['   Contact Duration:', blanks(15), num2str(Tc(1)*1000), ' [ms]']);
    display(['   Init. Velocity @ Contact Pt.:  ', blanks(1), num2str(Vi(1)), ' [m/s]']);
    display(['   Max. Deflection:  ', blanks(14), num2str(defl_max(1)*100), ' [cm]']);
    display(['   Max. Contact Force:  ', blanks(11), num2str(Fc_max(1)), ' [N]']);
    display(['   Init. Velocity.x @ CoM:',blanks(9),num2str(Vx(1)), '[m/s]']);
    % display(['   Init. Velocity.y @ CoM:',blanks(9),num2str(Vy(1)), '[m/s]']);
    % display(['   Init. Velocity.z @ CoM:',blanks(9),num2str(Vz(1)), '[m/s]']);
    % display(['   Init. Roll:', blanks(21),num2str(rad2deg(Roll(1))), '[deg]']);
    % display(['   Init. Pitch:', blanks(20),num2str(rad2deg(Pitch(1))), '[deg]']);
    % display(['   Init. Yaw:', blanks(22),num2str(rad2deg(Yaw(1))), '[deg]']);
    display(['   Init. Tilt:', blanks(21),num2str(rad2deg(Tilt(1))), '[deg]']);
    numContacts = max(size(Ti));
end

end

