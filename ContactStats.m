function [Tc, defl_max, Fc_max,Vi] = ContactStats( t_tot,defl_tot, Fc_tot,vi_c_tot)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

flag_c = 0;
idx_c = 1;
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
            Vi(idx_c) = vi_c_tot(i);
        else
            defl_max_indiv = max(defl_max_indiv,defl_tot(i));
            Fc_max_indiv = max(Fc_max_indiv,Fc_tot(i));
            
        end
    end
end

display('First Impact Information');
display('------------------------');
display(['   Contact Duration:', blanks(10), num2str(Tc(1)*1000), ' [ms]']);
display(['   Initial Impact Velocity:  ', blanks(1), num2str(Vi(1)), ' [m/s]']);
display(['   Max. Deflection:  ', blanks(9), num2str(defl_max(1)*100), ' [cm]']);
display(['   Max. Contact Force:  ', blanks(6), num2str(Fc_max(1)), ' [N]']);


end

