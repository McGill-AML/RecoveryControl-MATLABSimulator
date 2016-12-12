function localFlag = initflags
%initflags.m Initialize global and local flags
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: 
%-------------------------------------------------------------------------% 
global globalFlag   
    
    % Contact flags
    localFlag.contact.isContact = zeros(4,1);
    localFlag.contact.initialNormalVel = zeros(4,1);
    globalFlag.contact = localFlag.contact; 

end