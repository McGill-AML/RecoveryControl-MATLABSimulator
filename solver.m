syms beta 

% syms rb ux uy pbx pby rp tx ty
rb = 1;
ux = 1;
uy = 1;
pbx = 1;
pby = 1;
rp = 1;
tx = 1;
ty = 1;

eqn = (rb*(ux*cos(beta) + tx*sin(beta)) + pbx)^2 + ...
      (rb*(uy*cos(beta) + ty*sin(beta)) + pby)^2 - rp^2;

solve(eqn,beta,'ReturnConditions',true)
% 
% z1 = RootOf(rb^2*ty*uy*z^4*2i + rb^2*tx*ux*z^4*2i - rb^2*uy^2*z^4 - rb^2*ux^2*z^4 + rb^2*ty^2*z^4 + ...
%              rb^2*tx^2*z^4 - 4*pby*rb*uy*z^3 - 4*pbx*rb*ux*z^3 + pby*rb*ty*z^3*4i + pbx*rb*tx*z^3*4i - ...
%              2*rb^2*uy^2*z^2 - 2*rb^2*ux^2*z^2 - 2*rb^2*ty^2*z^2 - 2*rb^2*tx^2*z^2 + 4*rp^2*z^2 - 4*pby^2*z^2 - ...
%              4*pbx^2*z^2 - 4*pby*rb*uy*z - 4*pbx*rb*ux*z - pby*rb*ty*z*4i - pbx*rb*tx*z*4i - rb^2*ty*uy*2i - rb^2*tx*ux*2i ...
%              - rb^2*uy^2 - rb^2*ux^2 + rb^2*ty^2 + rb^2*tx^2);
% %- log(z1)*1i
 

% eqn = rb*(ux*cos(beta) +tx*sin(beta))+pbx + rb*(uy*cos(beta)+ty*sin(beta)) + pby - d;
% 
% answer = solve(eqn,beta)