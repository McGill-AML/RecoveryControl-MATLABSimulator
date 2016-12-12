function Hist = updatehist(Hist, t, state, stateDeriv, Pose, Twist, Control, PropState, Contact, localFlag, Sensor)
%updatehist.m Update "Hist", which records "discretized" data at tStep
%intervals (e.g., 200 Hz)
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: 
%-------------------------------------------------------------------------%

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

end
