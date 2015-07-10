function [Ts,PO] = ControllerStats(ttotal,Xtotal,Se,traj_posn,traj_head)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

Se_x = abs((traj_posn(end,1) - traj_posn(1,1))*Se);
Se_y = abs((traj_posn(end,2) - traj_posn(1,2))*Se);
Se_z = abs((traj_posn(end,3) - traj_posn(1,3))*Se);
Se_head = abs((traj_head(end) - traj_head(1))*Se);



Ts = zeros(4,1);
PO = zeros(4,1);

for i = size(Xtotal,1):-1:1
    OS_x = 100*(Xtotal(i,7) - traj_posn(end,1))/(traj_posn(end,1) - traj_posn(1,1));
    OS_y = 100*(Xtotal(i,8) - traj_posn(end,2))/(traj_posn(end,2) - traj_posn(1,2));
    OS_z = 100*(Xtotal(i,9) - traj_posn(end,3))/(traj_posn(end,3) - traj_posn(1,3));
    
    q = [Xtotal(i,10);Xtotal(i,11);Xtotal(i,12);Xtotal(i,13)]/norm(Xtotal(i,10:13));
    [~,~,yaw] = quat2angle(q,'xyz');    
    OS_head = 100*(yaw - traj_head(end))/(traj_head(end) - traj_head(1));
    
    if Se_x == 0
        OS_x = 0;
    end
    
    if Se_y == 0
        OS_y = 0;
    end
    
    if Se_z == 0
        OS_z = 0;
    end
    
    if Se_head == 0
        OS_head = 0;
    end
    
    if OS_x > PO(1)
        PO(1) = OS_x;
    end
    
    if OS_y > PO(2)
        PO(2) = OS_y;
    end
    
    if OS_z > PO(3)
        PO(3) = OS_z;
    end
    
    if OS_head > PO(4)
        PO(4) = OS_head;
    end
    
    if Ts(1) == 0 && Se_x ~= 0
        if abs(Xtotal(i,7)-traj_posn(end,1)) == Se_x
            Ts(1) = ttotal(i);
        elseif abs(Xtotal(i,7)-traj_posn(end,1)) > Se_x
            if i == size(Xtotal,1)
                display('x position does not settle');
            else
                Ts(1) = ttotal(i+1);
            end
        end
    end
    
    if Ts(2) == 0 && Se_y ~= 0
        if abs(Xtotal(i,8)-traj_posn(end,2)) == Se_y
            Ts(2) = ttotal(i);
        elseif abs(Xtotal(i,8)-traj_posn(end,2)) > Se_y
            if i == size(Xtotal,1)
                display('y position does not settle');
            else
                Ts(2) = ttotal(i+1);
            end
        end
    end
    
     if Ts(3) == 0 && Se_z ~= 0
        if abs(Xtotal(i,9)-traj_posn(end,3)) == Se_z
            Ts(3) = ttotal(i);
        elseif abs(Xtotal(i,9)-traj_posn(end,3)) > Se_z
            if i == size(Xtotal,1)
                display('z position does not settle');
            else
                Ts(3) = ttotal(i+1);
            end
        end
     end
    
     if Ts(4) == 0 && Se_head ~= 0
        if abs(yaw-traj_head(end)) == Se_head
            Ts(4) = ttotal(i);
        elseif abs(yaw-traj_head(end)) > Se_head
            if i == size(Xtotal,1)
                display('heading does not settle');
            else
                Ts(4) = ttotal(i+1);
            end
        end
    end
end

end

