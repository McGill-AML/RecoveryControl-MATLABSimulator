clear all
bumperNormalWorld = [-0.0616 0.0616 0.9962];
bumperCenterWorld = [1 0.0049 0];% [1 0 0];
poleVertical = [0 0 1];
poleVertical = poleVertical/norm(poleVertical);
iBumper = 1;
BUMP_RADII(iBumper) = 0.125;

H = cross(bumperNormalWorld,poleVertical);%[(bumperNormalWorld(2)*poleVertical(3)-poleVertical(2)*bumperNormalWorld(3)) -(bumperNormalWorld(1)*poleVertical(3)-bumperNormalWorld(3)*poleVertical(1)) (bumperNormalWorld(1)*poleVertical(2)-poleVertical(1)*bumperNormalWorld(2))]; %cross(bumperNormalWorld,poleVertical)
I = cross(bumperNormalWorld,-bumperCenterWorld);%-[(bumperNormalWorld(2)*bumperCenterWorld(3)-bumperCenterWorld(2)*bumperNormalWorld(3)) -(bumperNormalWorld(1)*bumperCenterWorld(3)-bumperNormalWorld(3)*bumperCenterWorld(1)) (bumperNormalWorld(1)*bumperCenterWorld(2)-bumperCenterWorld(1)*bumperNormalWorld(2))]; %cross(bumperNormalWorld,-bumperCenterWorld)
A = norm(H);%sqrt(H(1)^2+H(2)^2+H(3)^2);%norm(H);
B = norm(I);%sqrt(I(1)^2+I(2)^2+I(3)^2);%norm(I);
D = dot(H,I);%H(1)*I(1) +H(2)*I(2)+H(3)*I(3);%dot(H,I)
E = dot(poleVertical,-bumperCenterWorld);%-(poleVertical(1)*bumperCenterWorld(1) + poleVertical(2)*bumperCenterWorld(2) + poleVertical(3)*bumperCenterWorld(3)); %dot(poleVertical,-bumperCenterWorld);

coeffs = [A^2 ...
          2*E*A^2 + 2*D ...
          E^2*A^2 + 4*D*E + B^2 - (BUMP_RADII(iBumper))^2*A^4 ...
          2*D*E^2 + 2*E*B^2 -2*(BUMP_RADII(iBumper))^2*A^2*D ...
          B^2*E^2 - D^2*(BUMP_RADII(iBumper))^2];

allSolutions = roots(coeffs)
realSolutions = real(allSolutions(abs(imag(allSolutions)) < 1e-9))

distances = [];
for iter = 1:length(realSolutions)
    G = poleVertical*realSolutions(iter) - bumperCenterWorld;
    J = dot(bumperNormalWorld,G);%bumperNormalWorld(1)*G(1) + bumperNormalWorld(2)*G(2) + bumperNormalWorld(3)*G(3);
    K = cross(bumperNormalWorld,G);%[(bumperNormalWorld(2)*G(3)-G(2)*bumperNormalWorld(3)) -(bumperNormalWorld(1)*G(3)-bumperNormalWorld(3)*G(1)) (bumperNormalWorld(1)*G(2)-G(1)*bumperNormalWorld(2))];
    distances(iter) = (J^2 + (sqrt(K(1)^2 + K(2)^2 + K(3)^2) - (BUMP_RADII(iBumper)))^2)/2;
end  

[~, minIndex] = min(distances);
t = realSolutions(minIndex);