function ContHist = updateconthist(ContHist,Contact,PropState,stateDeriv,globalFlag)            

    ContHist.contacts = [ContHist.contacts; Contact];
    ContHist.propStates = [ContHist.propStates; PropState]; 
    ContHist.stateDerivs = [ContHist.stateDerivs, stateDeriv];
    ContHist.states = [ContHist.states, state];
    ContHist.globalFlag.contact.initialNormalVels = [ContHist.globalFlag.contact.initialNormalVels, globalFlag.contact.initialNormalVel];
    ContHist.globalFlag.contact.isContacts = [ContHist.globalFlag.contact.isContacts, globalFlag.contact.isContact];

end 