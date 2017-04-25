function [q_out] = px4_rotmat2quat(rotMat)
q_out = zeros(4,1);

if trace(rotMat) > 0
    s = sqrt(trace(rotMat)+1);
    q_out(1) = s*0.5;
    s = 0.5/s;
    q_out(2) = (rotMat(3,2) - rotMat(2,3))*s;
    q_out(3) = (rotMat(1,3) - rotMat(3,1))*s;
    q_out(4) = (rotMat(2,1) - rotMat(1,2))*s;
else
    dcm_i = 1;
    for ii = 1:2
        if rotMat(ii+1,ii+1) > rotMat(dcm_i,dcm_i)
            dcm_i = ii+1;
        end 
    end
    dcm_j = mod(dcm_i+1,3)+1;
    dcm_k = mod(dcm_i+2,3)+1;
    s = sqrt((rotMat(dcm_i,dcm_i) - rotMat(dcm_j,dcm_j) - rotMat(dcm_k,dcm_k)) +1);
    q_out(dcm_i+1) = s*0.5;
    s = 0.5/s;
    q_out(dcm_j+1) = (rotMat(dcm_i,dcm_j)+rotMat(dcm_j,dcm_i))*s;
    q_out(dcm_k+1) = (rotMat(dcm_k,dcm_i)+rotMat(dcm_i,dcm_k))*s;
    q_out(1) = (rotMat(dcm_k,dcm_j)-rotMat(dcm_j,dcm_k))*s;
end
    
    