function localFlag = initflags

global globalFlag   
    
    % Contact flags
    localFlag.contact.isContact = zeros(4,1);
    localFlag.contact.initialNormalVel = zeros(4,1);
    globalFlag.contact = localFlag.contact;

end