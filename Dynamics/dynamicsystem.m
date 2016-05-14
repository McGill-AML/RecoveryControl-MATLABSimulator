% function [stateDeriv, defl1, Contact.normalForceMag(iBumper), pc_w1, defl_rate1, defl2, Fc_mag2, pc_w2, defl_rate2, defl3, Fc_mag3, pc_w3, defl_rate3, defl4, Fc_mag4, pc_w4, defl_rate4,rpm,prop_accel] = spiridynamics(t,state,tStep,rpm,ImpactParams,prop_speed_prev,propCmds)
function [stateDeriv, Contact, PropState] = dynamicsystem(t,state,tStep,rpmControl,ImpactParams,rpmPrev,propCmds)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

global g m I Kt Dt Jr PROP_POSNS;

global AERO_AREA AERO_DENS Cd Tv Kp Kq Kr

global timeImpact

global globalFlag

%% Save Global Variables locally
rpmChkpt = globalFlag.experiment.rpmChkpt;
rpmChkptIsPassed = globalFlag.experiment.rpmChkptIsPassed;

%% Contact Parameters
kContact = ImpactParams.compliantModel.k;    
eContact = ImpactParams.compliantModel.e;
nContact = ImpactParams.compliantModel.n;

wallLoc = ImpactParams.wallLoc;
wallPlane = ImpactParams.wallPlane;

velocitySliding = ImpactParams.frictionModel.velocitySliding;

%% Initialize Local Contact Vars
Contact = initcontact(0);
normalForceWorld = zeros(3,4);
normalForceBody = zeros(3,4);
contactMomentBody = zeros(3,4);
%tangentialForceWorld = zeros(3,4);
tangentialForceBody = zeros(3,4);

%% 
q = [state(10);state(11);state(12);state(13)]/norm(state(10:13));
rotMat = quat2rotmat(q);

state = reshape(state,[max(size(state)),1]);

stateDeriv = zeros(13,1);

%% Control Signal
% rpm = rpmControl;
maxRPM = 8000;
minRPM = 1000;
maxRPMDeriv = 14000; % in rpm/s

rpmMagnitude = abs(rpmControl);
rpmSaturated = [-1;1;-1;1].*max(min(rpmMagnitude,maxRPM),minRPM); %saturate motor speeds
% rpm = rpmSaturated;

maxDifference = maxRPMDeriv*tStep;
% rpmDifference = rpmSaturated - rpmPrev;
% rpmDifferenceSaturated = sign(rpmDifference).*max(abs(rpmDifference),maxDifference);
% rpm = rpmDifferenceSaturated + rpmPrev;
rpm = zeros(4,1);
for iBumper = 1:4
    if abs(rpmSaturated(iBumper)-rpmPrev(iBumper)) > maxDifference
        rpm(iBumper) = sign(rpmSaturated(iBumper)-rpmPrev(iBumper))*maxDifference + rpmPrev(iBumper);
    else
        rpm(iBumper) = rpmSaturated(iBumper);
    end
end


% rpmDerivDesired = (rpmSaturated - rpmPrev)/tStep;
% rpmDerivSaturated = max(abs(rpmDerivDesired),maxRPMDeriv).*sign(rpmDerivDesired);
% rpm = rpmPrev + rpmDerivSaturated*tStep;

%% Contact Detection

if abs(wallLoc - state(7)) <= 0.3 %BUMP_RADIUS + sqrt(max(abs(PROP_POSNS(1,:)))^2 + max(abs(PROP_POSNS(2,:)))^2)
    if (sum(wallPlane == 'YZ')==2 || sum(wallPlane == 'ZY')==2)
        Contact = findcontact(rotMat, state, wallLoc);
        
        for iBumper = 1:4
            if Contact.defl(iBumper) > 0

                if globalFlag.contact.isContact(iBumper) == 0
                    globalFlag.contact.isContact(iBumper) = 1;
                    globalFlag.contact.initialNormalVel(iBumper) = Contact.deflDeriv(iBumper);
                end

                lambdaContact = 6*(1-eContact)*kContact/(((2*eContact-1)^2+3)*globalFlag.contact.initialNormalVel(iBumper));    
                Contact.normalForceMag(iBumper) = kContact*Contact.defl(iBumper)^nContact + lambdaContact*Contact.defl(iBumper)^nContact*Contact.deflDeriv(iBumper);


                normalForceWorld(:,iBumper) = [-Contact.normalForceMag(iBumper);0;0];
                
                % For Crash 3
%                 if iBumper == 2
%                     normalForceWorld(:,iBumper) = 2*normalForceWorld(:,iBumper);
%                 end 
                
                normalForceBody(:,iBumper) = rotMat*normalForceWorld(:,iBumper);
                
                
                % Calculate tangential force on bumper from friction
                
                % Account for stiction
                slidingVelocityNorm = norm(Contact.slidingVelocityWorld);
                
                if slidingVelocityNorm <= velocitySliding
                    Contact.muSliding = slidingVelocityNorm*ImpactParams.frictionModel.muSliding/velocitySliding;
                else
                    Contact.muSliding = ImpactParams.frictionModel.muSliding;
                end                
               
                %Coulomb friction                
                Contact.tangentialForceWorld(:,iBumper) = -Contact.muSliding*Contact.normalForceMag(iBumper)*Contact.slidingDirectionWorld(:,iBumper);
                tangentialForceBody(:,iBumper) = rotMat*Contact.tangentialForceWorld(:,iBumper);
                
                %Total contact moment 
                contactMomentBody(:,iBumper) = cross(Contact.point.contactBody(:,iBumper),normalForceBody(:,iBumper)+tangentialForceBody(:,iBumper));
                
            else
                if globalFlag.contact.isContact(iBumper) == 1
                    globalFlag.contact.isContact(iBumper) = 0;
                    globalFlag.contact.initialNormalVel(iBumper) = 0;
                end

                Contact.normalForceMag(iBumper) = 0;
                normalForceBody(:,iBumper) = [0;0;0];
                contactMomentBody(:,iBumper) = [0;0;0];

            end

        end
        
    end
end


%% Calculate contact force and moment

% uncomment after friction is added
if timeImpact == 10000
    if sum(globalFlag.contact.isContact) > 0
        timeImpact = t;
        rpmChkpt = rpmPrev;
        rpmChkptIsPassed(1) = 1;
    end
end

if isempty(propCmds) == 0
    for iCmds = max(size(propCmds)):-1:1
        if t >= timeImpact + propCmds(iCmds).rpmTime   
            if rpmChkptIsPassed(iCmds) == 0        
                rpmChkpt = rpmPrev;
                rpmChkptIsPassed(iCmds) = 1;
            end

            for iMotor = 1:4
                  rpm(iMotor) = (propCmds(iCmds,1).rpmDeriv(iMotor)~=0)*(sign(rpm(iMotor))*(propCmds(iCmds,1).rpmDeriv(iMotor)*(t - timeImpact - propCmds(iCmds,1).rpmTime)+abs(rpmChkpt(iMotor))));

            end

            break    
        end
    end
end

globalFlag.experiment.rpmChkpt = rpmChkpt;
globalFlag.experiment.rpmChkptIsPassed = rpmChkptIsPassed;

rpmDeriv = (rpm2rad(rpm) - rpm2rad(rpmPrev))/tStep; %in rad/s^2

PropState.rpm = rpm;
PropState.rpmDeriv = rpmDeriv;

Fg = rotMat*[0;0;-m*g];
V = 0;
Fa = Tv*[-0.5*AERO_DENS*V^2*AERO_AREA*Cd;0;0];
Ft = [0;0;-Kt*sum(rpm.^2)];

totalContactForce = sum(normalForceBody,2) + sum(tangentialForceBody,2);

% totalNormalForce = [0;0;0]; %sum(normalForceBody,2);
totalContactMoment = sum(contactMomentBody,2);

%% Status Quo
Mx = -Kt*PROP_POSNS(2,:)*(rpm.^2)-Kp*state(4)^2-state(5)*Jr*sum(rpm2rad(rpm)) + totalContactMoment(1) ;
My = Kt*PROP_POSNS(1,:)*(rpm.^2)-Kq*state(5)^2+state(4)*Jr*sum(rpm2rad(rpm)) + totalContactMoment(2) ;
Mz =  [-Dt Dt -Dt Dt]*(rpm.^2)-Kr*state(6)^2 -Jr*sum(rpmDeriv) + totalContactMoment(3);

% %% No angular acceleration or gyroscopic moment
% Mx = -Kt*PROP_POSNS(2,:)*(rpm.^2)-Kp*state(4)^2 + totalContactMoment(1) ;
% My = Kt*PROP_POSNS(1,:)*(rpm.^2)-Kq*state(5)^2 + totalContactMoment(2) ;
% Mz =  [-Dt Dt -Dt Dt]*(rpm.^2)-Kr*state(6)^2 + totalContactMoment(3);

% %% No angular acceleration only
% Mx = -Kt*PROP_POSNS(2,:)*(rpm.^2)-Kp*state(4)^2 -state(5)*Jr*sum(rpm2rad(rpm)) + totalContactMoment(1) ;
% My = Kt*PROP_POSNS(1,:)*(rpm.^2)-Kq*state(5)^2 + state(4)*Jr*sum(rpm2rad(rpm)) + totalContactMoment(2) ;
% Mz =  [-Dt Dt -Dt Dt]*(rpm.^2)-Kr*state(6)^2 + totalContactMoment(3);

% No gyroscopic moment only
% Mx = -Kt*PROP_POSNS(2,:)*(rpm.^2)-Kp*state(4)^2 + totalContactMoment(1) ;
% My = Kt*PROP_POSNS(1,:)*(rpm.^2)-Kq*state(5)^2 + totalContactMoment(2) ;
% Mz =  [-Dt Dt -Dt Dt]*(rpm.^2)-Kr*state(6)^2 -Jr*sum(rpmDeriv) + totalContactMoment(3);

stateDeriv(1:3) = (Fg + Fa + Ft + totalContactForce - m*cross(state(4:6),state(1:3)))/m;
stateDeriv(4:6) = inv(I)*([Mx;My;Mz]-cross(state(4:6),I*state(4:6)));
stateDeriv(7:9) = rotMat'*state(1:3);
stateDeriv(10:13) = -0.5*quatmultiply([0;state(4:6)],q);



end

