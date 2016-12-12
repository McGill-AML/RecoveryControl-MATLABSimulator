function [stateDeriv, Contact, PropState] = dynamicsystem(t,state,tStep,rpmControl,ImpactParams,rpmPrev,propCmds)
%dynamicsystem.m Continuous dynamics of quadrotor, including contact
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: 
%     Continuous dynamics of quadrotor, including:
%         - normal flight dynamics based on control input rpm
%         - wall contact dynamics (normal force & friction force)
%         - propeller rpm rate saturation
%     Inputs:
%         - t: time required for using ode numerical integration functions
%         - state: quadrotor state (u,v,w,p,q,r,X,Y,Z,q0,q1,q2,q3)
%         - tStep: controller time step
%         - rpmControl: control input rpm
%         - ImpactParams: parameters for calculating impact normal and friction forces
%         - rpmPrev: previous motor/propeller rpm, needed for rpm rate saturation
%         - propCmds: used to match simulation propeller speeds to experiments
%     Outputs:
%         - stateDeriv: state rates
%         - Contact: struct containing contact dynamics calculated variables
%         - PropState: actual propeller rpm (different from controller input rpm)
%-------------------------------------------------------------------------%

global g m I Kt Dt Jr PROP_POSNS %Inertial and Geometric Parameters
global AERO_AREA AERO_DENS Cd Tv Kp Kq Kr %Aerodynamic Parameters
global timeImpact globalFlag %Simulation global variables

%% Save Global Variables locally

% Needed to match simulation propeller speeds to experiments
rpmChkpt = globalFlag.experiment.rpmChkpt;
rpmChkptIsPassed = globalFlag.experiment.rpmChkptIsPassed;

%% Set Contact Parameters and Variables
[kContact, eContact, nContact, wallLoc, wallPlane, velocitySliding] = setimpactparams(ImpactParams);

% Initialize contact variables
Contact = initcontact(0);
normalForceWorld = zeros(3,4);
normalForceBody = zeros(3,4);
contactMomentBody = zeros(3,4);
tangentialForceBody = zeros(3,4);

%% State Initialization
q = [state(10);state(11);state(12);state(13)]/norm(state(10:13));
rotMat = quat2rotmat(q);

state = reshape(state,[max(size(state)),1]); %make sure state is column vector
stateDeriv = zeros(13,1); %initialize stateDeriv

%% Saturate Control RPM Rate Signal
maxRPMDeriv = 70000; %27000 in rpm/s
maxDifference = maxRPMDeriv*tStep;

rpm = zeros(4,1);
for iBumper = 1:4
    if abs(rpmControl(iBumper)-rpmPrev(iBumper)) > maxDifference
        rpm(iBumper) = sign(rpmControl(iBumper)-rpmPrev(iBumper))*maxDifference + rpmPrev(iBumper);
    else
        rpm(iBumper) = rpmControl(iBumper);
    end
end

%% Contact Detection and Force Calculation

if abs(wallLoc - state(7)) <= 0.3 
    if (sum(wallPlane == 'YZ')==2 || sum(wallPlane == 'ZY')==2)
        Contact = findcontact(rotMat, state, wallLoc); %find contact points
        
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

%% Match simulation propeller speeds to experiments
% uncomment after friction is added
if timeImpact == 10000
    if sum(globalFlag.contact.isContact) > 0
        timeImpact = t;
        rpmChkpt = rpmPrev;
        rpmChkptIsPassed(1) = 1;
    end
end

if isempty(propCmds) == 0 %propCmds = [] if not a experiment match simulation
    for iCmds = max(size(propCmds)):-1:1
        if t >= timeImpact + propCmds(iCmds).rpmTime   
            if rpmChkptIsPassed(iCmds) == 0        
                rpmChkpt = rpmPrev;
                rpmChkptIsPassed(iCmds) = 1;
            end

            for iMotor = 1:4
                if isfield(propCmds,'rpmDeriv')
                  rpm(iMotor) = (propCmds(iCmds,1).rpmDeriv(iMotor)~=0)*(sign(rpm(iMotor))*(propCmds(iCmds,1).rpmDeriv(iMotor)*(t - timeImpact - propCmds(iCmds,1).rpmTime)+abs(rpmChkpt(iMotor))));
                elseif isfield(propCmds, 'rpm')
%                     disp('set rpm');
                  rpm(iMotor) = propCmds(iCmds,1).rpm(iMotor)*sign(rpm(iMotor));
                else
                    error('invalid Experiment.propCmds');
                end
                
            end

            break    
        end
    end
end

globalFlag.experiment.rpmChkpt = rpmChkpt;
globalFlag.experiment.rpmChkptIsPassed = rpmChkptIsPassed;

rpmDeriv = (rpm2rad(rpm) - rpm2rad(rpmPrev))/tStep; %in rad/s^2

%% Save rate saturated, experiment-matched rpm
PropState.rpm = rpm;
PropState.rpmDeriv = rpmDeriv;

%% Calculate External Forces and Moments
Fg = rotMat*[0;0;-m*g]; %Gravity 
V = 0;
Fa = Tv*[-0.5*AERO_DENS*V^2*AERO_AREA*Cd;0;0]; %Aerodynamic
Ft = [0;0;-Kt*sum(rpm.^2)]; %Thruster

totalContactForce = sum(normalForceBody,2) + sum(tangentialForceBody,2);
totalContactMoment = sum(contactMomentBody,2);

Mx = -Kt*PROP_POSNS(2,:)*(rpm.^2)-Kp*state(4)^2-state(5)*Jr*sum(rpm2rad(rpm)) + totalContactMoment(1) ;
My = Kt*PROP_POSNS(1,:)*(rpm.^2)-Kq*state(5)^2+state(4)*Jr*sum(rpm2rad(rpm)) + totalContactMoment(2) ;
Mz =  [-Dt Dt -Dt Dt]*(rpm.^2)-Kr*state(6)^2 -Jr*sum(rpmDeriv) + totalContactMoment(3);

%% Update State Derivative
stateDeriv(1:3) = (Fg + Fa + Ft + totalContactForce - m*cross(state(4:6),state(1:3)))/m;
stateDeriv(4:6) = inv(I)*([Mx;My;Mz]-cross(state(4:6),I*state(4:6)));
stateDeriv(7:9) = rotMat'*state(1:3);
stateDeriv(10:13) = -0.5*quatmultiply([0;state(4:6)],q);


end

