function [  ] = Graphs( ttotal,Xtotal,defl_time,defl,Fc )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
figure();
subplot(2,2,1);
plot(ttotal,Xtotal(:,1),ttotal,Xtotal(:,2),ttotal,Xtotal(:,3));
legend('u','v','w');
subplot(2,2,2);
plot(ttotal,Xtotal(:,4),ttotal,Xtotal(:,5),ttotal,Xtotal(:,6));
legend('p','q','r');
subplot(2,2,3);
plot(ttotal,Xtotal(:,7),ttotal,Xtotal(:,8),ttotal,-Xtotal(:,9));
legend('x','y','-z');
subplot(2,2,4);
plot(ttotal,Xtotal(:,10),ttotal,Xtotal(:,11),ttotal,Xtotal(:,12),ttotal,Xtotal(:,13));
legend('q_0','q_1','q_2','q_3');

figure();
subplot(1,2,1);
plot(defl_time,defl*100);
xlabel('Time (s)');
ylabel('Deflection (cm)');
subplot(1,2,2);
plot(defl_time,Fc);
xlabel('Time (s)');
ylabel('Contact Force (N)');
end

