function [rmse_angle] = quat_error(truth, calc)

angle = zeros(1,length(truth));
for ii = 1:size(truth,2)
    err_quat = quatmultiply(truth(:,ii),quatconj(calc(:,ii)));
    
    angle(ii) = quat2axis_angle(err_quat)*180/pi;
    
end

rmse_angle = sqrt(mean(angle.^2));