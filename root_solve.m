
coeffs = [(BUMP_RADII(iBumper)^2*bumperCrossProduct(2)*bumperTangentWorld(2)*2i + BUMP_RADII(iBumper)^2*bumperCrossProduct(1)*bumperTangentWorld(1)*2i - BUMP_RADII(iBumper)^2*bumperTangentWorld(2)^2 - BUMP_RADII(iBumper)^2*bumperTangentWorld(1)^2 + BUMP_RADII(iBumper)^2*bumperCrossProduct(2)^2 + BUMP_RADII(iBumper)^2*bumperCrossProduct(1)^2 )
          (-4*bumperCenterWorld(2)*BUMP_RADII(iBumper)*bumperTangentWorld(2) - 4*bumperCenterWorld(1)*BUMP_RADII(iBumper)*bumperTangentWorld(1) + bumperCenterWorld(2)*BUMP_RADII(iBumper)*bumperCrossProduct(2)*4i + bumperCenterWorld(1)*BUMP_RADII(iBumper)*bumperCrossProduct(1)*4i)
          (-2*BUMP_RADII(iBumper)^2*bumperTangentWorld(2)^2 - 2*BUMP_RADII(iBumper)^2*bumperTangentWorld(1)^2 - 2*BUMP_RADII(iBumper)^2*bumperCrossProduct(2)^2 - 2*BUMP_RADII(iBumper)^2*bumperCrossProduct(1)^2 + 4*poleRadius^2 - 4*bumperCenterWorld(2)^2 - 4*bumperCenterWorld(1)^2)
          (-4*bumperCenterWorld(2)*BUMP_RADII(iBumper)*bumperTangentWorld(2) - 4*bumperCenterWorld(1)*BUMP_RADII(iBumper)*bumperTangentWorld(1) - bumperCenterWorld(2)*BUMP_RADII(iBumper)*bumperCrossProduct(2)*4i - bumperCenterWorld(1)*BUMP_RADII(iBumper)*bumperCrossProduct(1)*4i)
          (-BUMP_RADII(iBumper)^2*bumperCrossProduct(2)*bumperTangentWorld(2)*2i - BUMP_RADII(iBumper)^2*bumperCrossProduct(1)*bumperTangentWorld(1)*2i - BUMP_RADII(iBumper)^2*bumperTangentWorld(2)^2 - BUMP_RADII(iBumper)^2*bumperTangentWorld(1)^2 + BUMP_RADII(iBumper)^2*bumperCrossProduct(2)^2 + BUMP_RADII(iBumper)^2*bumperCrossProduct(1)^2)];

soln = (roots(coeffs));

beta = [];
for iter=1:size(soln)
    if abs(imag(-log(soln(iter))*1i)) <= 1e-5
        beta = [beta; real(-log(soln(iter))*1i)];
    end
end
  
if 1
    if beta(1) == beta(2) %1 pt of intersection
        Contact(iBumper).defl = 0;
        disp('single contact point');
    else %2 pts of intersection
        point1 = BUMP_RADII(iBumper)*cos(beta(1))*bumperTangentWorld + BUMP_RADII(iBumper)*sin(beta(1))*bumperCrossProduct + bumperCenterWorld
        point2 = BUMP_RADII(iBumper)*cos(beta(2))*bumperTangentWorld + BUMP_RADII(iBumper)*sin(beta(2))*bumperCrossProduct + bumperCenterWorld
    end

else
    disp('No contact')
end



