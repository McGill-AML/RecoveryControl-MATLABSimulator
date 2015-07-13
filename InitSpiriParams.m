function []= InitSpiriParams()
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

global g m I Jr prop_loc Kt A d_air Cd V Tv Kp Kq Kr Dt alpha beta Ixx Iyy Izz CM Rbumper Cbumper;

g = 9.81;

%Mass Properties
m = 1.06; %kg 926g

%Inertia Properties in body fixed frame
Ixx = 0.00503;
Iyy = 0.00547;
Izz = 0.0101;
I = [Ixx 0 0;0 Iyy 0;0 0 Izz]; %vehicle moment of inertias, kg m^2
Jr = 2.3917*10^-5; %Propeller moment of inertia about rotation axis, kg m^2
% 
% %Propeller locations relative to CoM
% prop_x1 = 0.2*cos(pi/4);
% prop_x2 = -0.2*cos(pi/4);
% prop_x3 = -0.2*cos(pi/4);
% prop_x4 = 0.2*cos(pi/4);
% prop_y1 = 0.2*cos(pi/4);
% prop_y2 = 0.2*cos(pi/4);
% prop_y3 = -0.2*cos(pi/4);
% prop_y4 = -0.2*cos(pi/4);
% prop_z1 = 0;
% prop_z2 = 0;
% prop_z3 = 0;
% prop_z4 = 0;

load('locations');

% prop_loc = [dp1,dp2,dp3,dp4];
prop_loc = [0.13 -0.13 -0.13 0.13;0.13 0.13 -0.13 -0.13;-0.0373 -0.0373 -0.373 -0.373];
CM = CoM;
Rbumper = 0.31;
% Cbumper = [-CM(1);-CM(2);prop_loc(3,1)];
Cbumper = sum(prop_loc,2)/4;

%Thrust coefficient
% Kt = 0.000000054; %From Pleiades primitives.c 07-03-2015
% Kt = 3.721e-5 * 2; %from x4data_stock.m
Kt = 0.1;

%Aerodynamic Drag
A = 0; %Area seen by relative velocity vector
d_air = 0; %Air density
Cd = 0; %Drag coefficient
V = 0; 
Tv = zeros(3); %Wind to body rotation matrix
Kp = 0; %Aerodynamic drag constant
Kq = 0; %Aerodynamic drag constant
Kr = 0; %Aerodynamic drag constant
Dt = 0.1; %Drag torque factor of coaxial rotor pairs
% Dt = 2.2056e-6 * (2); %from x4data_stock.m
alpha = 0;
beta = 0;


end

