R = 2; 
C0 = [0;0;0];
U = [R;0;0];
V = [0;R;0];
N = [-1;0;0];

syms angle px py pz

C = C0 + cos(angle)*U + sin(angle)*V;

solve(

% (C-P).N=0

