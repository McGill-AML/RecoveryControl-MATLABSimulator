function  bumperInfo = getsiminfo( Hist,bumper,traj_head)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

isContact = 0;
iContact = 1;

bumperInfo.timeImpacts = 0; 
bumperInfo.maxDefls = [];
bumperInfo.maxNormalForces = [];
bumperInfo.initialNormalVels = [];
bumperInfo.inclinations = [];
bumperInfo.numContacts = [];
bumperInfo.impactDurations = [];

timeHist = Hist.times;
stateHist = Hist.states';

if isfield(Hist,'localFlag') == 1
    initialNormalVelHist = Hist.localFlag.contact.initialNormalVels;
else
    initialNormalVelHist = Hist.globalFlag.contact.initialNormalVels;
end

for iTime = 1:size(timeHist)    
   
    if Hist.contacts(iTime).defl(bumper) == 0
        if isContact == 1
            isContact = 0;
            tf_indiv = timeHist(iTime);
            tc_indiv = tf_indiv - ti_indiv;
            bumperInfo.impactDurations(iContact) = tc_indiv;
            bumperInfo.maxDefls(iContact) = defl_max_indiv;
            bumperInfo.maxNormalForces(iContact) = Fc_max_indiv;
            
            iContact = iContact + 1;
        end
    else
        if isContact == 0
            isContact = 1;
            ti_indiv = timeHist(iTime);
            defl_max_indiv = Hist.contacts(iTime).defl(bumper);
            Fc_max_indiv = Hist.contacts(iTime).normalForceMag(bumper);
            bumperInfo.timeImpacts(iContact) = ti_indiv;
            bumperInfo.initialNormalVels(iContact) = initialNormalVelHist(bumper,iTime);
            
            q = [stateHist(iTime,10);stateHist(iTime,11);stateHist(iTime,12);stateHist(iTime,13)]/norm(stateHist(iTime,10:13));
            R = quat2rotmat(q);
            
            V_w = R'*([stateHist(iTime,1);stateHist(iTime,2);stateHist(iTime,3)]);
            Vx(iContact) = V_w(1); 

            bumperInfo.inclinations(iContact) = getinclination(R,traj_head);
        else
            defl_max_indiv = max(defl_max_indiv,Hist.contacts(iTime).defl(bumper));
            Fc_max_indiv = max(Fc_max_indiv,Hist.contacts(iTime).normalForceMag(bumper));
            
        end
    end
end

if bumperInfo.timeImpacts(1) == 0
    display('No impact for this bumper');
    bumperInfo.numContacts = 0;
else
    display('First Impact Information');
    display('------------------------');
    display(['   Init. Impact Time:', blanks(14), num2str(bumperInfo.timeImpacts(1)), '[s]']);
    display(['   Contact Duration:', blanks(15), num2str(bumperInfo.impactDurations(1)*1000), ' [ms]']);
    display(['   Init. Velocity @ Contact Pt.:  ', blanks(1), num2str(bumperInfo.initialNormalVels(1)), ' [m/s]']);
    display(['   Max. Deflection:  ', blanks(14), num2str(bumperInfo.maxDefls(1)*100), ' [cm]']);
    display(['   Max. Contact Force:  ', blanks(11), num2str(bumperInfo.maxNormalForces(1)), ' [N]']);
    display(['   Init. Velocity.x @ CoM:',blanks(9),num2str(Vx(1)), '[m/s]']);
    display(['   Init. Inclination:', blanks(14),num2str(rad2deg(bumperInfo.inclinations(1))), '[deg]']);
    bumperInfo.numContacts = max(size(bumperInfo.timeImpacts));
end

end

