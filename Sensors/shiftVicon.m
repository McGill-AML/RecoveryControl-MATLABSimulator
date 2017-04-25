function Vicon_Vframe = shiftVicon(Vicon_Vframe, vicDelay)

Vicon_Vframe(:,1:end-vicDelay+1) = Vicon_Vframe(:,vicDelay:end);
Vicon_Vframe(:, end-vicDelay+1:end) = repmat(Vicon_Vframe(:,end-vicDelay),1,vicDelay);