clear all;
% close all;
clc;

global CM prop_loc m Rb

%%Spiri System Parameters
InitSpiriParams;
r_ribbon = 0.31;

%%Simulation Parameters
traj_posn = [0 0 5;100 0 5];
traj_head = [0;0];
traj_time = [0;10];
t0 = traj_time(1);
tf = traj_time(end);
dt = 1/200;
% ref_r = [2 2 -5]';
% ref_head = pi/4;
q0 = quatmultiply([0;-1;0;0],[cos(traj_head(1)/2);0;0;sin(traj_head(1)/2)]);
q0 = q0/norm(q0);
x0 = [zeros(6,1);traj_posn(1,:)';q0;zeros(3,1)];
omega0 = zeros(4,1);

[posn,head] = CreateTrajectory(traj_posn,traj_head,traj_time,dt);

wall_loc = 1;
wall_plane = 'YZ';

%%Initial Variable Values
x0_step = x0;
Xtotal = x0';
ttotal = t0;
vztotal = [];
ez_prev = 0;
evz_prev = 0;
evx_prev = 0;
evy_prev = 0;
eyaw_prev = 0;
eroll_prev = 0;
epitch_prev = 0;
er_prev = 0;
omega_prev = omega0;

flag_c = 0;
flag_dec = 0;
defl2=0;
Fc2 = 0;

q = [Xtotal(end,10);Xtotal(end,11);Xtotal(end,12);Xtotal(end,13)]/norm(Xtotal(end,10:13));
R = quatRotMat(q);
ag = R*[0;0;-9.81];

ag_hist = ag';
u1_hist = [];
ez_hist = [];
evz_hist = [];

roll_hist = [];
roll2_hist = [];
pitch_hist = [];
pitch2_hist = [];
yaw_hist = [];
yaw2_hist = [];

rolldes_hist = [];
pitchdes_hist = [];
rdes_hist = [];

pt1_hist = [0;0;0];
pt2_hist = [0;0;0];
Pc_w_hist = [0;0;0];
defl_hist = 0;
theta_hist = 0;

Fc = [];
Pc = [];
traj_index = 1;
index_defl = 1;
numContacts = 0;
defl_contact = 0;
for i = t0:dt:tf-dt
    display(size(ttotal))
   
% % Wall @ 4m

%     q = [Xtotal(end,10);Xtotal(end,11);Xtotal(end,12);Xtotal(end,13)];
%     q = q/norm(q);
%     R = quatRotMat(q);
    T = [Xtotal(end,7);Xtotal(end,8);Xtotal(end,9)];
% 
%     if (4 - Xtotal(end,7)) <= r_ribbon
%         if flag_c == 0
%             [pB_contact,pW_wall,vi_contact,ti_contact,numContacts,flag_c] = Contact_Detect(i, Xtotal(end,:), r_ribbon, numContacts );
%             if flag_c == 1
%                 vi_c(numContacts) = sqrt(sum(Xtotal(end,1:3).^2));
%                 ti_c(numContacts) = ti_contact;
%                 pB_c(:,numContacts) = pB_contact;
%             end
%         end    
%     end
% 
%     defl_prev = defl_contact;
%     if flag_c == 1
% %         Contact_CalcDefl():
%           pW_contact = R'*pB_contact + T;
%           defl_contact = sign(pW_contact(1)-pW_wall(1))*sum((pW_contact - pW_wall).^2);
%           
%           if defl_contact <= 0
%               
%               flag_dec = 0;
%               flag_c = 0;
%               Fc_mag = 0;
%               defl_contact = 0;
%               t_c(numContacts) = i - ti_contact;
%               vf_c(numContacts) = sqrt(sum(Xtotal(end,1:3).^2));
%               
%           else
%               if flag_dec == 0
%                   if defl_contact < defl_prev
%                       flag_dec = 1;
%                   end
%               end
%               if flag_dec && (defl_contact > defl_prev) %Rebound detected
%                   flag_dec = 0;
%                   flag_c = 0;
%                   t_c(numContacts) = i - ti_contact;
%                   vf_c(numContacts) = sqrt(sum(Xtotal(end,1:3).^2));
%                   [pB_contact,pW_wall,vi_contact(index_defl),ti_contact,numContacts,flag_c] = Contact_Detect(i, Xtotal(end,:), r_ribbon, numContacts );
%                   
%                   if flag_c == 0
%                       Fc_mag = 0;
%                       defl_contact = 0;
%                   else
%                       disp('Rebound Detected');
%                       vi_c(numContacts) = sqrt(sum(Xtotal(end,1:3).^2));
%                       ti_c(numContacts) = ti_contact;
%                       pB_c(:,numContacts) = pB_contact;
%                       %Contact_CalcDefl():
%                       pW_contact = R'*pB_contact + T;
%                       defl_contact = sign(pW_contact(1)-pW_wall(1))*sum((pW_contact - pW_wall).^2);
%                       if defl_contact <= 0
%                           disp('Code Logic Error, Fiona!');
%                       end
%                   end
%               end
%               
%               if flag_c == 1
% %                   vB_normal = R*[-1;0;0];            
% %                   vB_normal = vB_normal/norm(vB_normal);
%                   
%                   %Hertz's Model
% %                   Fc_mag = 400*defl_contact^1.5;                  
%               end
%           end
%     else
        Fc_mag = 0;
        vB_normal = [];
        pB_contact = [];
        defl_contact = 0;  
        pW_contact = T;
        pW_wall = T;
%     end
%     
%     defl(index_defl) = defl_contact;    
%     Fc(index_defl) = Fc_mag;
%     Pc(:,index_defl) = pW_contact;
%     Pc_wall(:,index_defl) = pW_wall;
%     index_defl = index_defl + 1;

    %Trajectory Control Position
    ref_r = posn(traj_index,:)';
    ref_head = head(traj_index);
    traj_index = traj_index + 1;
    
    %Find Control Signal based on ref_r, ref_head
    if i ~= t0
        x0_step = X(end,:);
        ez_prev = ez;
        evz_prev = evz;
        evx_prev = evx;
        evy_prev = evy;
        eyaw_prev = eyaw;
        eroll_prev = eroll;
        epitch_prev = epitch;
        er_prev = er;
        omega_prev = omega;
    end
    [signal_c3,ez,evz,evx,evy,eyaw,eroll,epitch,er,omega,roll,pitch,yaw,roll_des,pitch_des,r_des] = ControllerZhang(Xtotal(end,:),i,t0,dt,ref_r,ref_head,ez_prev,evz_prev,eroll_prev,epitch_prev,er_prev,omega_prev);
%     disp(signal_c3);
    %     signal_c3 = [400;-400;400;-400;0;0;0;0];
    
       
    %Use Control Signal to propagate dynamics
    pt1 = [0;0;0];
    pt2 = [0;0;0];
    Pc_w = [0;0;0];
    defl = 0;
    theta1_s = 0;
%     [ pt1,pt2,Pc_w] = DetectContact1(Xtotal(end,:),wall_loc,wall_plane);
    [t,X] = ode45(@(t, X) SpiriMotion(t,X,signal_c3,wall_loc,wall_plane,flag_c,vB_normal,pB_contact,Fc_mag,pW_wall),[i i+dt],x0_step);
%     [~, ~, ~, pt1, Pc_w] = SpiriMotion([],[],signal_c3,wall_loc,wall_plane,flag_c,vB_normal,pB_contact,Fc_mag,pW_wall);

    %     [dx, ~, ~, pt1, Pc_w] = SpiriMotion(t,X(end,:)',signal_c3,wall_loc,wall_plane,flag_c,vB_normal,pB_contact,Fc_mag,pW_wall);
%         disp(defl_contact)
    
    q = [X(end,10);X(end,11);X(end,12);X(end,13)]/norm(X(end,10:13));
    R = quatRotMat(q);
    ag = R*[0;0;-9.81];
    
    if norm(ag)-9.81 > 0.001
        disp(norm(ag));
    end
    
    pt1_hist = [pt1_hist,pt1];
    pt2_hist = [pt2_hist,pt2];
    Pc_w_hist = [Pc_w_hist,Pc_w];
    defl_hist = [defl_hist;defl];
    theta_hist = [theta_hist;theta1_s];
    
    roll_hist = [roll_hist;roll];
%     roll2_hist = [roll2_hist;roll2];
    pitch_hist = [pitch_hist;pitch];
%     pitch2_hist = [pitch2_hist;roll2];
    yaw_hist = [yaw_hist;yaw];
%     yaw2_hist = [yaw2_hist;roll2];
    
    u1_hist = [u1_hist;signal_c3(1)];
    ez_hist = [ez_hist;ez];
    evz_hist = [evz_hist;evz];
    ag_hist = [ag_hist;ag'];
    
    rolldes_hist = [rolldes_hist;roll_des];
    pitchdes_hist = [pitchdes_hist;pitch_des];
    rdes_hist = [rdes_hist;r_des];
    
    Xtotal = [Xtotal;X(end,:)];
    ttotal = [ttotal;t(end)];
%     defl2 = [defl2;defl_contact2];
%     Fc2 = [Fc2;Fc_mag2];
       
   
    if Xtotal(end,9) <= 0
        display('Spiri has hit the floor :(');
        break;
    end  
%     
    
end
% F_calc = m*(vf_c - vi_c)./t_c;


% Graphs( ttotal,Xtotal,ttotal(1:end-1),defl,Fc );
Graphs( ttotal,Xtotal,ttotal,defl2,Fc2 );

figure();
plot(ttotal(1:end-1),roll_hist,ttotal(1:end-1),pitch_hist,ttotal(1:end-1),yaw_hist);
legend('roll','pitch','yaw');

figure();
plot(ttotal(1:end-1),rolldes_hist,ttotal(1:end-1),pitchdes_hist,ttotal(1:end-1),rdes_hist);
legend('roll_des','pitch_des','r_des');

figure()
plot(ttotal,defl_hist)
hold on
plot(ttotal,Xtotal(:,7)+Rb-wall_loc)
% 
% figure();
% plotyy(ttotal(1:end-1),u1_hist,ttotal(1:end-1),ez_hist);
% 
% figure();
% plot(ttotal,ag_hist(:,1),ttotal,ag_hist(:,2),ttotal,ag_hist(:,3));
% SpiriVisualization(ttotal,Xtotal);

% SpiriVisualization1(ttotal,Xtotal,'XZ',wall_loc,wall_plane)



