clear all;
% close all;
clc;

global CM

%%Spiri System Parameters
InitSpiriParams;
r_ribbon = 0.31;

%%Simulation Parameters
traj_posn = [0 2.5 -5;5 2.5 -5];
traj_head = [0;0];
traj_time = [0;10];
t0 = traj_time(1);
tf = traj_time(end);
dt = 1/30;
% ref_r = [2 2 -5]';
% ref_head = pi/4;
x0 = [zeros(6,1);traj_posn(1,:)';[1;0;0;0];zeros(3,1)];
omega0 = zeros(4,1);

[posn,head] = CreateTrajectory(traj_posn,traj_head,traj_time,dt);

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

Pc_x = 0.31;
Pc_y = 0;
Pc_z = 0;
flag_c = 0;

index = 1;
index_defl = 1;
for i = t0:dt:tf-dt
%     display(i)
    %Determine if Contact has occured
    % determine contact pt from middle section of sphere
    % track penetration of contact pt
    
    %Wall @ x = 4m
    if (4 - Xtotal(end,7)) <= r_ribbon
        if flag_c == 0 
            q = [Xtotal(end,10);Xtotal(end,11);Xtotal(end,12);Xtotal(end,13)];
            q = q/norm(q);
            R = quatRotMat(q);
            T = [Xtotal(end,7);Xtotal(end,8);-Xtotal(end,9)];
            
            ang=0:0.01:2*pi; 
            xB_ribbon=r_ribbon*cos(ang);
            yB_ribbon=r_ribbon*sin(ang);
            zB_ribbon=zeros(size(ang));
            
            pW_ribbon = R'*[xB_ribbon;yB_ribbon;zB_ribbon] + repmat(T,size(ang));
            pW_dist = abs(repmat(4,size(ang))-pW_ribbon(1,:));
            [min_dist, min_idx] = min(pW_dist);
          
            pB_contact = [xB_ribbon(min_idx);yB_ribbon(min_idx);zB_ribbon(min_idx)];
            pW_wall = [4;pW_ribbon(2,min_idx);pW_ribbon(3,min_idx)];
            
            vB_normal = CM - pB_contact;
            vB_normal = vB_normal/norm(vB_normal);
            
            vi_contact = sqrt(sum(Xtotal(end,1:3).^2));
            
            flag_c = 1
        end             
       
        q = [Xtotal(end,10);Xtotal(end,11);Xtotal(end,12);Xtotal(end,13)];
        q = q/norm(q);
        R = quatRotMat(q);
        T = [Xtotal(end,7);Xtotal(end,8);-Xtotal(end,9)];
        
        pW_contact = R'*pB_contact + T;
        defl_contact = sum((pW_contact - pW_wall).^2);
        defl(index_defl) = defl_contact;
        defl_time(index_defl) = i;
%         index_defl = index_defl + 1;
        defl_prev = defl(index_defl-1);
    else
        vB_normal = [];
        pB_contact = [];
        defl_contact = [];  
        defl_time(index_defl) = i;
        defl(index_defl) = 0;
%         index_defl = index_defl + 1;
        vi_contact = 0;
        defl_prev = 0;
        if flag_c == 1 
            
            flag_c = 0
        end
    end
    
    %Trajectory Control Position
    ref_r = posn(index,:)';
    ref_head = head(index);
    index = index + 1;
    
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
    [signal_c3,ez,evz,evx,evy,eyaw,eroll,epitch,er,omega] = ControllerZhang(Xtotal(end,:),i,t0,dt,ref_r,ref_head,ez_prev,evz_prev,eroll_prev,epitch_prev,er_prev,omega_prev);
    
    %Find contact Force using Hung & Crossley's contact model
    e_ribbon = 0.5;
    k_ribbon = 1*10^5; %[N/m]
    n_ribbon = 1.5;
    
    %lambda from Herbert & McWhannell
    lambda_ribbon = 6*(1-e_ribbon)/(((2*e_ribbon-1)^2+3)*vi_contact);
    
    Fc_mag = abs(defl_contact^n_ribbon*(lambda_ribbon*(defl_contact-defl_prev)/dt + k_ribbon));
 
    
    %Use Control Signal to propagate dynamics
    [t,X] = ode45(@(t, X) SpiriMotion(t,X,signal_c3,flag_c,vB_normal,pB_contact,Fc_mag),[i i+dt],x0_step);

    Xtotal = [Xtotal;X(end,:)];
    ttotal = [ttotal;t(end)];    
    index_defl = index_defl + 1;
end

Graphs( ttotal,Xtotal,defl_time,defl );


% SpiriVisualization(ttotal,Xtotal);





