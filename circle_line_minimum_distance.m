N = [1 1 0];
N = N/norm(N);
C = [2 0 0];
M = [0 0 1];
M = M/norm(M);
r = 0.5;
tic
for j = 1
% answer should be 1.5
    syms t
    
    H = (norm(cross(N,M))^2*t^2 + 2*dot(cross(N,M),cross(N,-C))*t + norm(cross(N,-C))^2)*(t+dot(M,-C))^2 ...
         - r^2*(norm(cross(N,M))^2*t + dot(cross(N,M),cross(N,-C)))^2;

    coeffs = sym2poly(H);

    soln = double(roots(coeffs));

    K = [];
    for i = 1:size(soln)
        if isreal(soln(i))
            P = soln(i)*M;
            Del = P - C;
            K = [K; norm(double(C + r*(Del - dot(N,Del)*N)/norm(Del - dot(N,Del)*N)))];
        end
    end
end
toc
K