% 
% Contact.point.contactWorld(:,iBumper) = real(rotMat'*Contact.point.contactBody(:,iBumper) + state(7:9));
% 
% % Contact point velocity, world
% contactPointVelocityWorld = rotMat'*([state(1);state(2);state(3)] + cross([state(4);state(5);state(6)],Contact.point.contactBody(:,iBumper)));
% % for validation only:
% Contact.pointVelocityWorld(:,iBumper) = contactPointVelocityWorld ;
% Contact.slidingVelocityWorld(:,iBumper) = [0;contactPointVelocityWorld(2:3)];
% Contact.slidingDirectionWorld(:,iBumper) = Contact.slidingVelocityWorld(:,iBumper)/norm(Contact.slidingVelocityWorld(:,iBumper));

% contactPointVelocityWorld = zeros(3,numel(Plot.times));
% for i = 1:numel(Plot.times)
%     q = Plot.quaternions(:,i);
%     q = q/norm(q);
%     rotMat = quat2rotmat(q);
%     contactPointVelocityWorld(:,i) = rotMat'*(Plot.linVels(:,i) + cross(Plot.angVels(:,i),Plot.contactPointBodys_bump2(i,:)'));
% end
%% Plot Bumper 2 and Quadrotor states
figure('Position',[962 25 960 949]);
subplot(4,2,1)
plot(Plot_mu4.times,Plot_mu4.normalForces(2,:));
xlim([0.5 0.75]);
title('Normal Force');
ylabel('$\mathbf{F}_n$ (N)','Interpreter','LaTex','FontSize',20);
xlabel('Time (s)');

subplot(4,2,2)
plot(Plot_mu4.times,Plot_mu4.tangentialForceWorlds_bump2(:,2:3));
xlim([0.5 0.75]);
title('Tangential Force');
ylabel('$\mathbf{F}_t$ (N)','Interpreter','LaTex','FontSize',20);
xlabel('Time (s)');
legend('Y','Z');

subplot(4,2,3)
plot(Plot_mu4.times,Plot_mu4.angVels(1:2,:));
legend('p','q','Location','southeast');
xlim([0.5 0.75]);
title('Angular Velocity');
ylabel('$\mathbf{\omega}$ (rad/s)','Interpreter','LaTex','FontSize',20);
xlabel('Time (s)');

subplot(4,2,4)
plot(Plot_mu4.times,Plot_mu4.quaternions);
xlim([0.5 0.75]);
title('Quaternion');
ylabel('$\mathbf{q}$','Interpreter','LaTex','FontSize',20);
xlabel('Time (s)');
legend('q0','q1','q2','q3')

subplot(4,2,5)
plot(Plot_mu4.times,Plot_mu4.linVels);
xlim([0.5 0.75]);
title('Body Linear Velocity');
ylabel('$\mathbf{v}$ (m/s)','Interpreter','LaTex','FontSize',20);
xlabel('Time (s)');
legend('u','v','w')

subplot(4,2,6)
plot(Plot_mu4.times,Plot_mu4.contactPointBodys_bump2);
xlim([0.5 0.75]);
title('Body Contact Point');
ylabel('$\mathbf{r}_C$ (m)','Interpreter','LaTex','FontSize',20);
xlabel('Time (s)');
legend('x','y','z')

subplot(4,2,7)
plot(Plot_mu4.times,Plot_mu4.contactPtVelocityWorlds_bump2(:,2:3));
hold on
plot(Plot_mu4.times,colnorm(Plot_mu4.contactPtVelocityWorlds_bump2'),'k')
legend('Y','Z','magnitude');
xlim([0.5 0.75]);
title('World Contact Point Tangential Velocity');
ylabel('$\dot{\mathbf{p}}_{C,T}$ (m/s)','Interpreter','LaTex','FontSize',20);

xlabel('Time (s)');

subplot(4,2,8)
plot(Plot_mu4.times,Plot_mu4.slidingDirectionWorlds_bump2(:,2:3));
xlim([0.5 0.75]);
title('World Contact Point Tangential Velocity Direction');
legend('Y','Z');
% ylabel('$\hat{\dot{p}}_{C,T}$','Interpreter','LaTex','FontSize',20);
ylabel('$\dot{\mathbf{p}}_{C,T}/||\dot{\mathbf{p}}_{C,T}||$','Interpreter','LaTex','FontSize',20);
xlabel('Time (s)');


suptitle('Bumper 2 Data, Crash I-10, \mu=0.4');

%% Calculate propagated error on contact point velocity
defaultError = [1e-3;1e-3;1e-3];
error_contactptvelocityworldmagnitude = zeros(size(Plot_mu4.times));
for i = 1:numel(Plot_mu4.times)
    crossproduct = cross(Plot_mu4.angVels(:,i),Plot_mu4.contactPointBodys_bump2(i,:)');
    error_crossproduct = propagateError('CrossProduct',Plot_mu4.angVels(:,i),Plot_mu4.contactPointBodys_bump2(i,:)',defaultError,defaultError);
    
    contactptvelocitybody = Plot_mu4.linVels(:,i) + crossproduct;
    error_contactptvelocitybody = propagateError('VectorAddition',Plot_mu4.linVels(:,i),crossproduct,[1e-5;1e-5;1e-5],error_crossproduct);
    
    rotMat = quat2rotmat(Plot_mu4.quaternions(:,i));
    contactptvelocityworld = rotMat'*contactptvelocitybody;
    error_contactptvelocityworld = propagateError('RotateVectorBody2World',Plot_mu4.quaternions(:,i),contactptvelocitybody,1e-3,error_contactptvelocitybody);
    
    error_contactptvelocityworldmagnitude(i) = propagateError('VectorMagnitude',contactptvelocityworld,[],error_contactptvelocityworld,[]);
end