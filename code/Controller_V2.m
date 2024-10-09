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
u_e = [lthrust_e; rthrust_e]; 

%Finding the symbolic matrices for A and B and linearizing around the
%equilibrium values

Asym  = jacobian(sdot,s) ;
Bsym  = jacobian(sdot,u) ;

A = double(subs(Asym,[s;u],[s_e;u_e])) ;
B = double(subs(Bsym,[s;u],[s_e;u_e])) ;

end

%
% STEP #2: Modify, but do NOT rename, this function. It is called every
% time through the simulation loop.
%

function [actuators, data] = runControlSystem(sensors, references, parameters, data)

u = [0;0];

actuators.rthrust = 0 + 2*(1/sqrt(2))*parameters.m*parameters.g;
actuators.lthrust = 0 + 2*(1/sqrt(2))*parameters.m*parameters.g;

end
