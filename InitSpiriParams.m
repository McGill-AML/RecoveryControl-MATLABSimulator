function []= InitSpiriParams()
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

global g m I Jr prop_loc Kt A d_air Cd V Tv Kp Kq Kr Dt alpha beta Ixx Iyy Izz CM Rbumper Cbumper Abumper;

g = 9.81;

%Mass Properties
m = 0.933; %kg 926g

%Inertia Properties in body fixed frame
Ixx = 0.008737;
Iyy = 0.008988;
Izz = 0.017143;
Ixy = -4.2e-7;
Iyz = -1.14e-6;
Izx = -5.289e-5;

I = [Ixx Ixy Izx;Ixy Iyy Iyz;Izx Iyz Izz]; %vehicle moment of inertias, kg m^2
% I = [Ixx 0 0;0 Iyy 0;0 0 Izz];
Jr = 2.20751e-5; %Propeller moment of inertia about rotation axis, kg m^2


load('locations2');
prop_loc = [p1, p2, p3, p4] - repmat(CoM,1,4); %prop locations relative to CoM
% prop_loc = [dp1,dp2,dp3,dp4];
% prop_loc = [0.13 -0.13 -0.13 0.13;0.13 0.13 -0.13 -0.13;-0.0373 -0.0373 -0.373 -0.373];
CM = CoM;
Rbumper = 0.11; %0.29
% Cbumper = sum(prop_loc,2)/4; %bumper center relative to CoM
Cbumper = prop_loc(:,1);
Abumper = deg2rad(11);

%Thrust coefficient
% Kt = 0.000000054; %From Pleiades primitives.c 07-03-2015
Kt = 7.015e-8; %Calculated from 8x4.5 APC Prop

%Drag Torque factor of coaxial rotor pairs
Dt = 9.61e-10; %Calculated from 8x4.5 APC Prop


%Aerodynamic Drag
A = 0; %Area seen by relative velocity vector
d_air = 0; %Air density
Cd = 0; %Drag coefficient
V = 0; 
Tv = zeros(3); %Wind to body rotation matrix
Kp = 0; %Aerodynamic drag constant
Kq = 0; %Aerodynamic drag constant
Kr = 0; %Aerodynamic drag constant
alpha = 0;
beta = 0;


end

