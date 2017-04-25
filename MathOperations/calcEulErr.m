function [eulerErr] = calcEulErr(EA1, EA2)
%input two euler angles, find largest positive error between each term
eulerErr = zeros(3,1);
EA1 = EA1*180/pi;
EA2 = EA2*180/pi;
for ii = 1:3
    if EA1(ii) > EA2(ii) + 180
        eulerErr(ii) = EA2(ii) - EA1(ii) + 360;
    elseif EA2(ii) > EA1(ii) + 180
        eulerErr(ii) = -EA2(ii) + EA1(ii) + 360;
    else
        eulerErr(ii) = abs(EA1(ii) - EA2(ii));
    end
end