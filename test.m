% syms alpha beta R theta wall_loc Cx
syms theta

Rot = RotMat('Y',30*pi/180);
u = Rot'*[1;0;0];
n = Rot'*[0;0;1];
% u2 = [cos(30*pi/180);0;-sin(30*pi/180)];
% n2 = [sin(30*pi/180);0;cos(30*pi/180)];
alpha = u(1);
temp = cross(n,u);
beta = temp(1);
R = 0.3;
wall_loc = 0.1;
Cx = 0;
S = solve (alpha*R*cos(theta)+beta*R*sin(theta)==wall_loc - Cx,theta);
S = eval(S);
if isreal(S) == 1

    pt1 = R*cos(S(1))*u + R*sin(S(1))*temp;
    pt2 = R*cos(S(2))*u + R*sin(S(2))*temp;
    
    pt1_b = Rot*pt1;
    pt2_b = Rot*pt2;
end
