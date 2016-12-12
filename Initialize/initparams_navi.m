function [ImpactParams]= initparams_navi()
%initparams_navi.m Initialize Navi parameters
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: All inertial, geometric, propeller parameters
%-------------------------------------------------------------------------%   

global g m I Ixx Iyy Izz

global Jr Dt Kt PROP_POSNS BUMP_RADII BUMP_ANGLE

global AERO_AREA AERO_DENS Cd Tv Kp Kq Kr ALPHA BETA

global u2RpmMat BUMP_NORMS BUMP_TANGS BUMP_POSNS

global IMU_POSN

g = 9.81;

%% Mass Properties
m = 1.095; %kg

%% Inertia Properties in body fixed frame
Ixx = 0.01121976;
Iyy = 0.01122668;
Izz = 0.021082335;
Ixy = -5.62297e-05;
Iyz = -4.4954e-06;
Izx = -1.418e-08;

%% Propeller Parameters
load('proplocations_navi'); %thruster locations

PROP_POSNS = [p1, p2, p3, p4] - repmat(CoM,1,4); %prop locations relative to CoM

I = [Ixx Ixy Izx;Ixy Iyy Iyz;Izx Iyz Izz]; %vehicle moment of inertias, kg m^2
Jr = 2.20751e-5; %Propeller moment of inertia about rotation axis, kg m^2

%Thrust coefficient
% Kt = 0.000000054; %From Pleiades primitives.c 07-03-2015
Kt = 8.7e-8; %Calculated from thrust needed for Spiri to hover w/ white 8" props
%Kt = 7.015e-8; %Calculated from 8x4.5 APC Prop

%Drag Torque factor of coaxial rotor pairs
Dt = 0.1*Kt; %9.61e-10; %Calculated from 8x4.5 APC Prop


u2RpmMat = inv([-Kt -Kt -Kt -Kt;...
                -Kt*PROP_POSNS(2,1) -Kt*PROP_POSNS(2,2) -Kt*PROP_POSNS(2,3) -Kt*PROP_POSNS(2,4);...
                 Kt*PROP_POSNS(1,1) Kt*PROP_POSNS(1,2) Kt*PROP_POSNS(1,3) Kt*PROP_POSNS(1,4);...
                 -Dt Dt -Dt Dt]);

%% Aerodynamic Drag
AERO_AREA = 0; %Area seen by relative velocity vector
AERO_DENS = 0; %Air density
Cd = 0; %Drag coefficient
Tv = zeros(3); %Wind to body rotation matrix
Kp = 0; %Aerodynamic drag constant
Kq = 0; %Aerodynamic drag constant
Kr = 0; %Aerodynamic drag constant
ALPHA = 0;
BETA = 0;

%% Bumper Parameters
BUMP_RADII = [0.125; 0.125; 0.125; 0.125];
BUMP_ANGLE = deg2rad(5); %angle bumpers are tilted towards body center

BUMP_NORMS(:,1) = invar2rotmat('Z',deg2rad(45))'*invar2rotmat('Y',BUMP_ANGLE + deg2rad(90))'* [1;0;0];
BUMP_NORMS(:,2) = invar2rotmat('Z',deg2rad(135))'*invar2rotmat('Y',BUMP_ANGLE + deg2rad(90))'* [1;0;0];
BUMP_NORMS(:,3) = invar2rotmat('Z',deg2rad(-135))'*invar2rotmat('Y',BUMP_ANGLE + deg2rad(90))'* [1;0;0];
BUMP_NORMS(:,4) = invar2rotmat('Z',deg2rad(-45))'*invar2rotmat('Y',BUMP_ANGLE + deg2rad(90))'* [1;0;0];

BUMP_TANGS(:,1) = invar2rotmat('Z',deg2rad(45))'*invar2rotmat('Y',BUMP_ANGLE)'*[1;0;0];
BUMP_TANGS(:,2) = invar2rotmat('Z',deg2rad(135))'*invar2rotmat('Y',BUMP_ANGLE)'*[1;0;0];
BUMP_TANGS(:,3) = invar2rotmat('Z',deg2rad(-135))'*invar2rotmat('Y',BUMP_ANGLE)'*[1;0;0];
BUMP_TANGS(:,4) = invar2rotmat('Z',deg2rad(-45))'*invar2rotmat('Y',BUMP_ANGLE)'*[1;0;0];

load('bumperlocations_navi'); %bumper center locations

BUMP_POSNS = [b1, b2, b3, b4] - repmat(CoM,1,4); %bumper center locations relative to CoM

% BUMP_RADII(3) = 0.04; %change one of bumpers to top of Navi's head
% BUMP_NORMS(:,3) = [0;0;1];
% BUMP_TANGS(:,3) = [1;0;0];
% BUMP_POSNS(:,3) = [0;0;-0.061];

% Bumper stiffness params
ImpactParams.compliantModel.e = 0.9;
ImpactParams.compliantModel.k = 372;
% ImpactParams.compliantModel.k = 372*2;
ImpactParams.compliantModel.n = 0.66;

%% IMU Parameters
IMU_POSN = [19.205; 8.1034; -47.55] * 10^-3; %IMU posn relative to CM

end

