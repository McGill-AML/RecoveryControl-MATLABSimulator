function Hist = inithist(timeInit, state, stateDeriv, Pose, Twist, Control, PropState, Contact, localFlag, Sensor)
%inithist.m Initialize Hist struct for recording data
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: 
%-------------------------------------------------------------------------% 

% Initialize history of the state and its derivative
Hist.states = state;
Hist.stateDerivs = stateDeriv;
Hist.times = timeInit;

% Initialize history of twist, pose and control structs
Hist.poses = Pose;
Hist.twists = Twist;
Hist.controls = Control;

Hist.contacts = Contact;
Hist.propStates = PropState;
Hist.localFlag.contact.isContacts = localFlag.contact.isContact;
Hist.localFlag.contact.initialNormalVels = localFlag.contact.initialNormalVel;

Hist.sensors = Sensor;

end
