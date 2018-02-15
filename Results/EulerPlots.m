close all
hold on
grid on
numPitch = 46;
numOffset = 71;
for iPitch = 1%:numPitch
    for iOffset = 2%:numOffset
        trial = 71*(iPitch-1)+iOffset;
        disp(trial);
        offset = cell2mat(Batch(trial,1));
        pitch = cell2mat(Batch(trial,2));
        times = cell2mat(Batch(trial,7));
        positions = cell2mat(Batch(trial,6));
        deflections = cell2mat(Batch(trial,7));
        recoveryStages = cell2mat(Batch(trial,8));
        states = cell2mat(Batch(trial,11));
        RPM = cell2mat(Batch(trial,12));
        U = cell2mat(Batch(trial,13));
        EulerAngles = cell2mat(Batch(trial,14));
        phi = EulerAngles(1,:);
        theta = EulerAngles(2,:);
        psi = EulerAngles(3,:);
    end
end
%% Euler Angles
% plot(times(:),phi(:));
% plot(times(:),theta(:));
% plot(times(:),psi(:));
% legend('Roll(\phi)','Pitch(\theta)', 'Yaw(\psi)');
% title(strcat('Euler Angles (\phi, \theta, \psi) for Pitch = ',num2str(-pitch),'^o'));
% xlabel('Time(s)');
% ylabel('EulerAngles(rad)');
 
%% Propeller RPMs
% plot(times(:),RPM(:,1));
% plot(times(:),RPM(:,2));
% plot(times(:),RPM(:,3));
% plot(times(:),RPM(:,4));
% legend('\Omega_1','\Omega_2', '\Omega_3', '\Omega_4');
% title(strcat('RPMs for Pitch = ',num2str(-pitch),'^o'));
% xlabel('Time(s)');
% ylabel('RPM(rad/s)');

%% Control Inputs U1,U2,U3,U4
% plot(times(:),U(:,1));
% legend('U_1');
% title(strcat('Thrust for Pitch = ',num2str(-pitch),'^o'));
% xlabel('Time(s)');
% ylabel('Input Thrust(N)');
% saveas(gcf,'Control Thrust.jpg');
% figure();
% hold on;
% grid on;
% plot(times(:),U(:,2));
% plot(times(:),U(:,3));
% plot(times(:),U(:,4));
% legend('U_2(M_x)', 'U_3(M_y)', 'U_4(M_z)');
% title(strcat('Desired Moments for Pitch = ',num2str(-pitch),'^o'));
% xlabel('Time(s)');
% ylabel('Desired Moments (Nm)');
% saveas(gcf,'Control Moments.jpg');


%% Height Loss
% floor=-ones(1,401);
% plot(times(:),states(9,:));
% plot(times(:),floor(:),'r-');
% legend('Height','Floor');
% title(strcat('Height variation for Pitch = ',num2str(-pitch),'^o'));
% xlabel('Time(s)');
% ylabel('Height(m)');

