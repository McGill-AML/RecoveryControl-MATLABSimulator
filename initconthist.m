function ContHist = initconthist(ContHist,timeInit,state,Contact,PropState,zeros(13,1),globalFlag)

    ContHist.globalFlag.contact.initialNormalVels = globalFlag.contact.initialNormalVel;
    ContHist.globalFlag.contact.isContacts = globalFlag.contact.isContact;
    ContHist.contacts = Contact;
    ContHist.propStates = PropState;
    ContHist.stateDerivs = stateDeriv;
    ContHist.times = timeInit;
    ContHist.states = state;
end