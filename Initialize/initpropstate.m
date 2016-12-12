function PropState = initpropstate()
%initposetwist.m Initialize PropState struct
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description:
%-------------------------------------------------------------------------%   

    PropState.rpm = zeros(4,1);
    PropState.rpmDeriv = zeros(4,1);
    
end