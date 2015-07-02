function [signal_c] = ControllerFaessler(x,ref_r,ref_v,ref_a,ref_head)

global m g I prop_loc Dt Kt

%%Faessler 2015
%Controller Gains
Pxy = 30;
Pz = 30;
Dxy = 6;
Dz = 6;
Ppos = diag([Pxy,Pxy,Pz]);
Dpos = diag([Dxy,Dxy,Dz]);
Prp = 10;
Pyaw = 1;
Ppq = 20;
Pr = 30;
Patt = diag([Ppq,Ppq,Pr]);

q = [x(10);x(11);x(12);x(13)]/norm(x(10:13));
T = quatRotMat(q);

%Attitude Controller
a_des = Ppos*(ref_r-x(7:9)')+Dpos*(ref_v-(T'*x(1:3)'))+(ref_a-g);
% qc = Xtotal(end,10:13)';
% Tc = T;%quatRotMat(qc);

% ezB = quatmultiply(q,[0;0;0;1]);
ezB = T'*[0;0;1];
ezB = ezB/norm(ezB);
c_des = ezB'*a_des;

ezB_des = a_des/norm(a_des);
alpha = acos(ezB'*ezB_des);
n = cross(ezB,ezB_des);
n = n/norm(n);
nB = T*n;
qe_rp = [cos(alpha/2);nB*sin(alpha/2)];

p_des = 2*Prp*qe_rp(2);
q_des = 2*Prp*qe_rp(3);
if qe_rp(1) < 0
    p_des = -p_des;
    q_des = -q_des;
end

exC = [cos(ref_head);sin(ref_head);0];
eyC = [-sin(ref_head);cos(ref_head);0];

exB_des = cross(eyC,ezB_des);
exB_des = exB_des/norm(exB_des);

if exB_des == 0
    r_des = 0;
else
    eyB_des = cross(ezB_des,exB_des);
    eyB_des = eyB_des/norm(eyB_des);
    T_des = [exB_des eyB_des ezB_des];
    quat_des = RotMat2Quat(T_des);
        
    qe_y = quatinv(quatmultiply(x(10:13)',qe_rp));
    qe_y = quatmultiply(qe_y,quat_des);
    r_des = 2*Pyaw*qe_y(3);
    if qe_y(1) < 0
        r_des = -r_des;
    end
end

w_meas = x(4:6)';
torque_des = I*Patt*([p_des;q_des;r_des]-w_meas) + cross(w_meas,I*w_meas);
A = [1 1 1 1;prop_loc(1,:);-prop_loc(2,:);Dt/Kt -Dt/Kt Dt/Kt -Dt/Kt];
thrust_des = inv(A)*[m*c_des;torque_des];
omega_des = sqrt(abs(thrust_des/-Kt));
omega_des = [omega_des(1);-omega_des(2);omega_des(3);-omega_des(4)];

error = -Kt*sum(omega_des.^2) - c_des;
signal_c = [c_des;torque_des];

end

