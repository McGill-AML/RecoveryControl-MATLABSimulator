function Hist = updatehist(Hist, t, state, stateDeriv, Pose, Twist, Control, PropState, Contact, localFlag, Sensor, ...
                        sensParams, EKF, AEKF, SPKF, ASPKF, COMP, HINF, SPKF_full, EKF_att, SRSPKF,...
                        SRSPKF_full, ASPKF_opt, AHINF,SPKF_norm, useExpData,iSim, tStep)
    if useExpData == 0
        kk = round(iSim/tStep+2);
    else
        kk = iSim; 
    end
                    
    if useExpData == 0
    Hist.times(:,kk) = t;
    Hist.states(:,kk) = state;
    Hist.stateDerivs(:,kk) =  stateDeriv;
    
    Hist.poses(kk,1) = Pose;
    Hist.twists(kk,1) = Twist;
    Hist.controls(kk,1) = Control;    
    
    Hist.contacts(kk,1) =  Contact;
    Hist.propStates(kk,1) =  PropState;     
    
    Hist.localFlag.contact.initialNormalVels(:,kk) = localFlag.contact.initialNormalVel;
    Hist.localFlag.contact.isContacts(:,kk) = localFlag.contact.isContact;
    
    
    Hist.crash(kk,1) =  sensParams.crash;
    end
    
    Hist.sensors(kk,1) =  Sensor;
    
    
%     Hist.EKF = [Hist.EKF; EKF];
%     Hist.AEKF = [Hist.AEKF; AEKF];

    Hist.SPKF(kk,1) =  SPKF;
    Hist.ASPKF(kk,1) =  ASPKF;
    Hist.COMP(kk,1) =  COMP;
    Hist.HINF(kk,1) =  HINF;
%     Hist.SPKF_full = [Hist.SPKF_full; SPKF_full];
    Hist.EKF_att(kk) =  EKF_att;
    Hist.SRSPKF(kk,1) =  SRSPKF; 
%     Hist.SRSPKF_full = [Hist.SRSPKF_full; SRSPKF_full]; 
    Hist.ASPKF_opt(kk,1) =  ASPKF_opt;
    Hist.AHINF(kk,1) =  AHINF;
    Hist.SPKF_norm(kk,1) =  SPKF_norm;
    
    
end
