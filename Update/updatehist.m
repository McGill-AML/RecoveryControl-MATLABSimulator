function Hist = updatehist(Hist, t, state, stateDeriv, Pose, Twist, Control, PropState, Contact, localFlag, Sensor, ...
                        sensParams, EKF, AEKF, SPKF, ASPKF, COMP, HINF, SPKF_full, EKF_att, SRSPKF, SRSPKF_full, ASPKF_opt, AHINF)
                    
    Hist.times = [Hist.times;t];
    Hist.states = [Hist.states, state];
    Hist.stateDerivs = [Hist.stateDerivs, stateDeriv];
    
    Hist.poses = [Hist.poses;Pose];
    Hist.twists = [Hist.twists;Twist];
    Hist.controls = [Hist.controls;Control];    
    
    Hist.contacts = [Hist.contacts; Contact];
    Hist.propStates = [Hist.propStates; PropState];     
    Hist.localFlag.contact.initialNormalVels = [Hist.localFlag.contact.initialNormalVels,localFlag.contact.initialNormalVel];
    Hist.localFlag.contact.isContacts = [Hist.localFlag.contact.isContacts,localFlag.contact.isContact];
    
    Hist.sensors = [Hist.sensors; Sensor];
    
    Hist.crash = [Hist.crash; sensParams.crash];
    
    Hist.EKF = [Hist.EKF; EKF];
    Hist.AEKF = [Hist.AEKF; AEKF];
    Hist.SPKF = [Hist.SPKF; SPKF];
    Hist.ASPKF = [Hist.ASPKF; ASPKF];
    Hist.COMP = [Hist.COMP; COMP];
    Hist.HINF = [Hist.HINF; HINF];
    Hist.SPKF_full = [Hist.SPKF_full; SPKF_full];
    Hist.EKF_att = [Hist.EKF_att; EKF_att];
    Hist.SRSPKF = [Hist.SRSPKF; SRSPKF]; 
    Hist.SRSPKF_full = [Hist.SRSPKF_full; SRSPKF_full]; 
    Hist.ASPKF_opt = [Hist.ASPKF_opt; ASPKF_opt];
    Hist.AHINF = [Hist.AHINF; AHINF];

end
