function [IC, Setpoint] = matchoffboardsetpoints(crash)
%mmatchoffboardsetpoints.m Match experiment post-collision position setpoint
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: Position setpoint is equivalent to target setpoint used by
%   PX4 offboard controller.
%-------------------------------------------------------------------------%

IC = initIC;
Setpoint = initsetpoint;

switch crash
    case 'VII-03'
        IC.posn = [0;0;0.6];  
        IC.attEuler = [0;0;deg2rad(180+9.19)];
        Setpoint.head = deg2rad(-9.19+180);
        Setpoint.posn = [1.5;0;0.6];
        Setpoint.time = 2;  
    case 'VII-04'
        IC.posn = [0;0;0.6];  
        IC.attEuler = [0;0;deg2rad(180+10.5)];
        Setpoint.head = deg2rad(-10.5+180);
        Setpoint.posn = [1.5;0;0.6];
        Setpoint.time = 2;
    case 'VII-05'
        IC.posn = [0;0;0.6];  
        IC.attEuler = [0;0;deg2rad(180+9.19)];
        Setpoint.head = deg2rad(-9.19+180);
        Setpoint.posn = [1.5;0;0.6];
        Setpoint.time = 2;          
    case 'VII-07'
        IC.posn = [0;0;0.6];  
        IC.attEuler = [0;0;deg2rad(180+15.58)];
        Setpoint.head = deg2rad(-15.58+180);
        Setpoint.posn = [1.5;0;0.6];
        Setpoint.time = 2;          
    case 'VII-08'
        IC.posn = [0.2;0;0.6];  
        IC.attEuler = [0;0;deg2rad(180+21.32)];
        Setpoint.head = deg2rad(-21.32+180);
        Setpoint.posn = [2.5;0;0.6];
        Setpoint.time = 2;
    case 'VII-10'
        IC.posn = [0;0;0.6];  
        IC.attEuler = [0;0;deg2rad(180+16.49)];
        Setpoint.head = deg2rad(-16.49+180);
        Setpoint.posn = [2;0;0.6];
        Setpoint.time = 2;          
    case 'VII-11'
        IC.posn = [0;0;0.6];
        IC.attEuler = [0;0;deg2rad(180+12.19)];
        Setpoint.head = deg2rad(180-12.19);
        Setpoint.posn = [1;0;0.6];
        Setpoint.time = 2;
    case 'VII-12'
        IC.posn = [0;0;0.6];  
        IC.attEuler = [0;0;deg2rad(180+14.12)];
        Setpoint.head = deg2rad(-14.12+180);
        Setpoint.posn = [2.5;0;0.6];
        Setpoint.time = 2;               
    case 'VII-13'
        IC.posn = [-0.5;0;0.6];  
        IC.attEuler = [0;0;deg2rad(180+8.91)];
        Setpoint.head = deg2rad(-8.91+180);
        Setpoint.posn = [1.5;0;0.6];
        Setpoint.time = 2;       
    case 'VII-14'
        IC.posn = [-0.5;0;0.6];  
        IC.attEuler = [0;0;deg2rad(180+15.14)];
        Setpoint.head = deg2rad(-15.14+180);
        Setpoint.posn = [2;0;0.6];
        Setpoint.time = 2;      
    case 'VII-15'
        IC.posn = [-0.5;0;0.6];  
        IC.attEuler = [0;0;deg2rad(180+13.11)];
        Setpoint.head = deg2rad(-13.11+180);
        Setpoint.posn = [2.5;0;0.6];
        Setpoint.time = 2;               
    case 'VII-16'
        IC.posn = [-0.5;0;0.6];  
        IC.attEuler = [0;0;deg2rad(180+8.79)];
        Setpoint.head = deg2rad(-8.79+180);
        Setpoint.posn = [3;0;0.6];
        Setpoint.time = 2;       
    case 'VII-17'
        IC.posn = [-1;0;0.6];  
        IC.attEuler = [0;0;deg2rad(180+9.87)];
        Setpoint.head = deg2rad(-9.87+180);
        Setpoint.posn = [2;0;0.6];
        Setpoint.time = 2;  
    case 'VII-18'
        IC.posn = [-1;0;0.6];  
        IC.attEuler = [0;0;deg2rad(180+8.2)];
        Setpoint.head = deg2rad(-8.2+180);
        Setpoint.posn = [2;0;0.6];
        Setpoint.time = 2;               
    case 'VII-19'
        IC.posn = [-1;0;0.6];  
        IC.attEuler = [0;0;deg2rad(180+5.68)];
        Setpoint.head = deg2rad(-5.68+180);
        Setpoint.posn = [3;0;0.6];
        Setpoint.time = 2;       
    case 'VII-20'
        IC.posn = [-1;0;0.6];  
        IC.attEuler = [0;0;deg2rad(180+9.22)];
        Setpoint.head = deg2rad(-9.22+180);
        Setpoint.posn = [4;0;0.6];
        Setpoint.time = 2;  
    case 'VII-21'
        IC.posn = [-1;0;0.6];  
        IC.attEuler = [0;0;deg2rad(180+5.51)];
        Setpoint.head = deg2rad(-5.51+180);
        Setpoint.posn = [5;0;0.6];
        Setpoint.time = 2;               
    case 'VII-22'
        IC.posn = [0.2;0;0.6];  
        IC.attEuler = [0;0;deg2rad(180+4.76)];
        Setpoint.head = deg2rad(-4.76+180);
        Setpoint.posn = [3;0;0.6];
        Setpoint.time = 2;       
    case 'VII-23'
        IC.posn = [0.2;0;0.6];  
        IC.attEuler = [0;0;deg2rad(180+14.53)];
        Setpoint.head = deg2rad(-14.53+180);
        Setpoint.posn = [4;0;0.6];
        Setpoint.time = 2;  
    case 'VII-24'
        IC.posn = [0.2;0;1];  
        IC.attEuler = [0;0;deg2rad(180+9.59)];
        Setpoint.head = deg2rad(-9.59+180);
        Setpoint.posn = [5;0;1];
        Setpoint.time = 2;               
    case 'VII-25'
        IC.posn = [0.2;0;3];  
        IC.attEuler = [0;0;deg2rad(180+10.74)];
        Setpoint.head = deg2rad(-10.74+180);
        Setpoint.posn = [6;0;3];
        Setpoint.time = 2;       
    case 'VII-26'
        IC.posn = [0.2;0;0.6];  
        IC.attEuler = [0;0;deg2rad(180+0.53)];
        Setpoint.head = deg2rad(-0.53+180);
        Setpoint.posn = [7;0;0.6];
        Setpoint.time = 2;          
    otherwise
        error('Invalid Matching Setpoint Experiment');
        
end

end