function Hist = inithist(Hist, timeInit, state, stateDeriv, PropState, Pose, Twist, Control, Contact, localFlag)

Hist.states = state;
Hist.stateDerivs = stateDeriv;
Hist.times = timeInit;
Hist.poses = Pose;
Hist.twists = Twist;
Hist.controls = Control;
Hist.contacts = Contact;
Hist.propStates = PropState;
Hist.localFlag.contact.isContacts = localFlag.contact.isContact;
Hist.localFlag.contact.initialNormalVels = localFlag.contact.initialNormalVel;

end
