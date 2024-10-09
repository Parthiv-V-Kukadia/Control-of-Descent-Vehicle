clc;clear;


%from Designproblem01.m in processes
a  = 2.6/2;
b  = 3.4/2;
m = 950;
J = 1450.3333;
g = 3.711;


%Defining other Variables we are going to use
syms xDdot xdot x zDdot zdot z thetaDdot thetadot theta lthrust rthrust;
%Defining system of equations
xDdot = (lthrust/m) * cos(pi/4+theta) - rthrust/m*sin(pi/4+theta);
zDdot = (lthrust/m) * sin(pi/4+theta) + rthrust/m*cos(pi/4+theta) - g;
thetaDdot = (lthrust - rthrust) * (a - b) * sqrt(2)/(2*J); 

%State variables, with s as the state matrixx as x is a working variable
s = [x;xdot;z;zdot;theta;thetadot];
sdot = [xdot;xDdot;zdot;zDdot;thetadot;thetaDdot];

%Desired state variable state values
theta_e  = 0;
thetadot_e = 0;
%thetaDdot_e = 0; %however it isnt used in sim, same for xDdot, zDdot
z_e = 35; %The height from the ground <40 (max given in problem)
zdot_e  =  0;
%zDdot_e  = 0;
x_e = 0;
xdot_e = 0;
%xDdot_e = 0;
lthrust_e = (m*g)/sqrt(2); %found from xDdot by subbing equilibrium values
rthrust_e = lthrust_e;

%Input matrix u
u  = [xDdot;zDdot;thetaDdot];

%Initialization of equilibrium values for linearization of s and u
s_e = [theta_e;thetadot_e;zdot_e;z_e;xdot_e;x_e];
u_e  = [lthrust_e; rthrust_e];

%Finding the symbolic matrices for A and B and linearizing around the
%equilibrium values
Asym  = jacobian(sdot,s);
Bsym  = jacobian(sdot,u)

A = double(subs(Asym,[s;u],[s_e;u_e]))
B = double(subs(Bsym,[s;u],[s_e;u_e]))
