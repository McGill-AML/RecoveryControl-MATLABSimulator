function ContHist = updateconthist(ContHist, stateDeriv, Pose, Twist, Control, PropState, Contact, globalFlag, Sensor)
%updateconthist.m Update "ContHist", which records "continous" data
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: 
%-------------------------------------------------------------------------%

    ContHist.stateDerivs = [ContHist.stateDerivs, stateDeriv];
    
    ContHist.poses = [ContHist.poses;Pose];
    ContHist.twists = [ContHist.twists;Twist];
    ContHist.controls = [ContHist.controls;Control];    
    
    ContHist.contacts = [ContHist.contacts; Contact];
    ContHist.propStates = [ContHist.propStates; PropState];     
    ContHist.globalFlag.contact.initialNormalVels = [ContHist.globalFlag.contact.initialNormalVels, globalFlag.contact.initialNormalVel];
    ContHist.globalFlag.contact.isContacts = [ContHist.globalFlag.contact.isContacts, globalFlag.contact.isContact];
    
    ContHist.sensors = [ContHist.sensors; Sensor];

end 