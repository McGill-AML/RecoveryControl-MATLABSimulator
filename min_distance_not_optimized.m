clear all
N = [0 0 1];
N = N/norm(N);
C = [2 0 0];
M = [0 0 1];
M = M/norm(M);
r = 0.5;
tic

% H = cross(N,M);%[(N(2)*M(3)-M(2)*N(3)) -(N(1)*M(3)-N(3)*M(1)) (N(1)*M(2)-M(1)*N(2))]; %cross(N,M)
% I = cross(N,-C);%-[(N(2)*C(3)-C(2)*N(3)) -(N(1)*C(3)-N(3)*C(1)) (N(1)*C(2)-C(1)*N(2))]; %cross(N,-C)
A = norm(cross(N,M));%sqrt(H(1)^2+H(2)^2+H(3)^2);%norm(H);
B = norm(cross(N,-C));%sqrt(I(1)^2+I(2)^2+I(3)^2);%norm(I);
D = dot(cross(N,M),cross(N,-C));%A*B;
E = dot(M,-C);%-(M(1)*C(1) + M(2)*C(2) + M(3)*C(3)); %dot(M,-C);

coeffs = [A^2 ...
          2*E*A^2 + 2*D ...
          E^2*A^2 + 4*D*E + B^2 - r^2*A^4 ...
          2*D*E^2 + 2*E*B^2 -2*r^2*A^2*D ...
          B^2*E^2 - D^2*r^2];

allSolutions = roots(coeffs)
realSolutions = real(allSolutions(abs(imag(allSolutions)) < 1e-9))

%%
distances = [];
for iter = 1:length(realSolutions)
    G = M*realSolutions(iter) - C;
    distances(iter) = (dot(N,G)^2 +(norm(cross(N,G))-r)^2)/2
%     J = dot(N,G);%N(1)*G(1) + N(2)*G(2) + N(3)*G(3);
%     K = cross(N,G);%[(N(2)*G(3)-G(2)*N(3)) -(N(1)*G(3)-N(3)*G(1)) (N(1)*G(2)-G(1)*N(2))];
%     distances(iter) = (J^2 + (norm(K) - r)^2)/2 %sqrt(K(1)^2 + K(2)^2 + K(3)^2)
end  

[minDistance, minIndex] = min(distances);
%%
t = realSolutions(minIndex);

G = M*t - C;
J = dot(N,G);%N(1)*G(1) + N(2)*G(2) + N(3)*G(3);
L = G - J*N;

point = C + r*(G-dot(N,G)*N)/norm(G-dot(N,G)*N)%L/sqrt(L(1)^2+L(2)^2+L(3)^2)
point = C + r*L/norm(L)



