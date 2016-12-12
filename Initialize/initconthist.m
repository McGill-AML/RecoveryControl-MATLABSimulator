function ContHist = initconthist(timeInit, state, stateDeriv, Pose, Twist, Control, PropState, Contact, globalFlag, Sensor)
%initconthist.m Initialize ContHist struct for recording data
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: 
%-------------------------------------------------------------------------% 

% Initialize history of the state and its derivative
ContHist.states = state;
ContHist.stateDerivs = stateDeriv;
ContHist.times = timeInit;

% Initialize history of twist, pose and control structs
ContHist.poses = Pose;
ContHist.twists = Twist;
ContHist.controls = Control;

ContHist.contacts = Contact;
ContHist.propStates = PropState;
ContHist.globalFlag.contact.initialNormalVels = globalFlag.contact.initialNormalVel;
ContHist.globalFlag.contact.isContacts = globalFlag.contact.isContact;


ContHist.sensors = Sensor;

end