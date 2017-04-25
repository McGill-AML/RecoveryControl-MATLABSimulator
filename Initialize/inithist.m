function Hist = inithist(SimParams, state, stateDeriv, Pose, Twist, Control, PropState, Contact, localFlag,...
                        Sensor, sensParams, EKF, AEKF, SPKF, ASPKF, COMP, HINF, SPKF_full,EKF_att,SRSPKF,...
                        SRSPKF_full, ASPKF_opt, AHINF,SPKF_norm, useExpData, tStep, expLength)
                
if useExpData == 0
    hist_size = round((SimParams.timeFinal-SimParams.timeInit)/tStep)+1;   
else
    hist_size = expLength;
end

% Initialize history of the state and its derivative
if useExpData == 0
Hist.states = state;
Hist.states(:,hist_size) = state;

Hist.stateDerivs = stateDeriv;
Hist.stateDerivs(:,hist_size) = stateDeriv;

Hist.times = SimParams.timeInit;
Hist.times(:,hist_size) = SimParams.timeInit;

% Initialize history of twist, pose and control structs
Hist.poses = Pose;
Hist.poses(hist_size,1) = Pose;

Hist.twists = Twist;
Hist.twists(hist_size,1) = Twist;

Hist.controls = Control;
Hist.controls(hist_size,1) = Control;

Hist.contacts = Contact;
Hist.contacts(hist_size,1) = Contact;

Hist.propStates = PropState;
Hist.propStates(hist_size,1) = PropState;

Hist.localFlag.contact.isContacts = localFlag.contact.isContact;
Hist.localFlag.contact.isContacts(:,hist_size) = localFlag.contact.isContact;

Hist.localFlag.contact.initialNormalVels = localFlag.contact.initialNormalVel;
Hist.localFlag.contact.initialNormalVels(:,hist_size) = localFlag.contact.initialNormalVel;

Hist.crash = sensParams.crash;
Hist.crash(hist_size,1) = sensParams.crash;
end

Hist.sensors = Sensor;
Hist.sensors(hist_size,1) = Sensor;


% Hist.EKF = EKF;
% Hist.AEKF = AEKF;

Hist.SPKF = SPKF;
Hist.SPKF(hist_size,1) = SPKF;

Hist.ASPKF = ASPKF;
Hist.ASPKF(hist_size,1) = ASPKF;

Hist.COMP = COMP;
Hist.COMP(hist_size,1) = COMP;

Hist.HINF = HINF;
Hist.HINF(hist_size,1) = HINF;

% Hist.SPKF_full = SPKF_full;

Hist.EKF_att = EKF_att;
Hist.EKF_att(hist_size,1) = EKF_att;

Hist.SRSPKF = SRSPKF;
Hist.SRSPKF(hist_size,1) = SRSPKF;

% Hist.SRSPKF_full = SRSPKF_full;
Hist.ASPKF_opt = ASPKF_opt;
Hist.ASPKF_opt(hist_size,1) = ASPKF_opt;

Hist.AHINF = AHINF;
Hist.AHINF(hist_size,1) = AHINF;

Hist.SPKF_norm = SPKF_norm;
Hist.SPKF_norm(hist_size,1) = SPKF_norm;



end
