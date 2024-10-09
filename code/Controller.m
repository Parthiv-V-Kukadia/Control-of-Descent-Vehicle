function func = Controller
% INTERFACE
%
%   sensors
%       .t          (time)
%       .x          (horizontal position)
%       .xdot       (horizontal velocity)
%       .z          (vetical position)
%       .zdot       (vertical velocity)
%       .theta      (orientation)
%       .thetadot   (angular velocity)
%
%   references
%       .x          (desired horizontal position)
%       .xdot       (desired horizontal velocity)
%       .z          (desired vertical position)
%       .zdot       (desired vertical velocity)
%       .theta      (desired orientation)
%       .thetadot   (desired angular velocity)
%
%   parameters
%       .tStep      (time step)
%       .g          (Martian acceleration due to gravity)
%       .m          (mass of the descent vehicle)
%       .J          (moment of inertia about an axis passing through the center of mass perpendicular to the xz plane)
%       .maxthrust  (maximum allowable net thrust - both thrusters are identical; the minimum is 0)
%       
%   data
%       .whatever   (yours to define - put whatever you want into "data")
%
%   actuators
%       .lthrust     (thrust on left thruster)
%       .rthrust     (thrust on right thruster)
%
% Do not modify this function.
func.init = @initControlSystem;
func.run = @runControlSystem;
end

%
% STEP #1: Modify, but do NOT rename, this function. It is called once,

% before the simulation loop starts.
%

function [data] = initControlSystem(parameters, data)
 
%
% Here is a good place to initialize things...
%

g  = parameters.g;
m  = parameters.m;
J  = parameters.J;

%From DesignProblem01.m under processes
a = 2.6/2;
b = 3.4/2; 

%Defining other variables we are going to use
syms x xdot xDdot z zdot zDdot theta thetadot thetaDdot lthrust rthrust;

%Defining system of equations
xDdot = (lthrust/m) * cos(pi/4+theta) - rthrust/m*sin(pi/4+theta);
zDdot = (lthrust/m) * sin(pi/4+theta) + rthrust/m*cos(pi/4+theta) - g;
thetaDdot = (lthrust - rthrust) * (a - b) * (3*sqrt(2))/(m*(a^2 + b^2)); 

% State variables, with s as the state matrix as x is already defined in the problem 
s = [x;xdot; z;zdot; theta; thetadot];
sdot = [xdot;xDdot; zdot;zDdot; thetadot;thetaDdot];

%Input matrix u
u = [lthrust; rthrust] ;

%Desired state variable state values 
%xDdot, zDdot, thetaDdot = 0 - they aren't used in the simulation
references.x = 0 ;
references.xdot = 0 ;
references.z = 30 ; % must hover and be within 40m (max given in problem)
references.zdot = 0 ; 
references.theta = 0; %zero angle 
references.thetadot = 0;
lthrust_e = m*g/sqrt(2); %found by subbing equilibrium values into motion equations  
rthrust_e = lthrust_e; %both have to be equal at the end to successfully hover

%Initialization of equilibrium values for linearization of s and u
s_e = [references.x;references.xdot; references.z;references.zdot; references.theta;references.thetadot];
data.u_e = [lthrust_e;rthrust_e];

%Finding the symbolic matrices for A and B and linearizing around the
%equilibrium values

Asym  = jacobian(sdot,s) ;
Bsym  = jacobian(sdot,u) ;

A = double(subs(Asym,[s;u],[s_e;data.u_e])) ;
B = double(subs(Bsym,[s;u],[s_e;data.u_e])) ;

C = eye(6); %output matrix taken was[x;xdot;z;zdot;theta;thetadot], and a partial derivative with respect to s was taken to find C

%D  =  0, as there  is no input in the output matrix
%Calculating K using place function
p1 = -1;
p2 = -2; 
p3 = -3; 
p4 = -4; 
p5 = -5; 
p6 = -6; 

P = [p1 p2 p3 p4 p5 p6];
data.K = place(A,B,P);

end

%
% STEP #2: Modify, but do NOT rename, this function. It is called every
% time through the simulation loop.
%

function [actuators, data] = runControlSystem(sensors, references, parameters, data)

X = [sensors.x; sensors.xdot; sensors.z; sensors.zdot; sensors.theta; sensors.thetadot];
u = -data.K*X;

u1 = u(1);
u2 = u(2);

actuators.rthrust = u2 + (1/sqrt(2))*parameters.m*parameters.g;
actuators.lthrust = u1 + (1/sqrt(2))*parameters.m*parameters.g;

if actuators.rthrust > parameters.maxthrust;
    if actuators.rthrust >= actuators.lthrust;
        actuators.lthrust = (actuators.lthrust/actuators.rthrust)*parameters.maxthrust;
        actuators.rthrust = parameters.maxthrust;
    end
end

if actuators.lthrust > parameters.maxthrust;
    if actuators.lthrust >= actuators.rthrust;
        actuators.rthrust = (actuators.rthrust/actuators.lthrust)*parameters.maxthrust;
        actuators.lthrust = parameters.maxthrust;
    end
end

if actuators.lthrust < 0;
    actuators.lthrust < 0;
    actuators.lthrust = 0;
end

if actuators.rthrust < 0;
    actuators.rthrust < 0;
    actuators.rthrust = 0;
end

end
