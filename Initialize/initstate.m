function [state, stateDeriv] = initstate(IC, xAcc)
%initstate.m Initialize full state of quadrotor 
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%
%   Index map:
%   [1 2 3 4 5 6 7 8 9 10 11 12 13]
%   [u v w p q r x y z q0 q1 q2 q3]
%-------------------------------------------------------------------------%   
    state = zeros(13,1);    
    stateDeriv = zeros(13,1); % The derivative of the state
       
    state(1:3) = IC.linVel; % u v w
    state(4:6) = IC.angVel;   % p q r
    state(7:9) = IC.posn;   % x y z
    state(10:13) = angle2quat(-(IC.attEuler(1)+pi),IC.attEuler(2),IC.attEuler(3),'xyz')';
    
    rotMat = quat2rotmat(state(10:13));
    stateDeriv(1:3) = rotMat*[xAcc;0;0];
    stateDeriv(7:9) = rotMat'*state(1:3);
end