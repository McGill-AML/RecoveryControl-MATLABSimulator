rb = 0.5;
ux = 0;
uy = 0;
pbx = -0.47001;
pby = 0.0;
rp = 0.1;
tx = 1;
ty = 0;

bumperTangentWorld = [ux uy 1];
bumperCrossProduct = [tx ty 0];
bumperCenterWorld = [pbx pby 0];
close all

for pbx = -0.4:0.001:-0.2

    coeffs = [(rb^2*ty*uy*2i + rb^2*tx*ux*2i - rb^2*uy^2 - rb^2*ux^2 + rb^2*ty^2 + rb^2*tx^2 )
        (-4*pby*rb*uy - 4*pbx*rb*ux + pby*rb*ty*4i + pbx*rb*tx*4i)
        (-2*rb^2*uy^2 - 2*rb^2*ux^2 - 2*rb^2*ty^2 - 2*rb^2*tx^2 + 4*rp^2 - 4*pby^2 - 4*pbx^2)
        (-4*pby*rb*uy - 4*pbx*rb*ux - pby*rb*ty*4i - pbx*rb*tx*4i)
        (-rb^2*ty*uy*2i - rb^2*tx*ux*2i - rb^2*uy^2 - rb^2*ux^2 + rb^2*ty^2 + rb^2*tx^2)];

    soln = (roots(coeffs));

    beta = [];
    for iter=1:size(soln)
        if abs(imag(-log(soln(iter))*1i)) <= 1e-5
            beta = [beta; real(-log(soln(iter))*1i)];
        end
    end
    
    scatter(pbx, beta(1),'r');
    hold on
    scatter(pbx, beta(2),'b');
    grid on
end

    
if 1
    if beta(1) == beta(2) %1 pt of intersection
        defl = 0;
        disp('single contact point');
    else %2 pts of intersection
        point1 = rb*cos(beta(1))*bumperTangentWorld + rb*sin(beta(1))*bumperCrossProduct + bumperCenterWorld
        point2 = rb*cos(beta(2))*bumperTangentWorld + rb*sin(beta(2))*bumperCrossProduct + bumperCenterWorld
    end

else
    disp('No contact')
end



