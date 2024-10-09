clear all; clc;
%Mass, Dimensions, Gravity
m = 950;
a = 1.3;
b = 1.7;
g = 3.711;

% - Moment of Inertia
J = m*((2*a)^2 + (2*b)^2)/12;

%Matrices
A=[0 1 0 0 0 0 ; 0 0 0 0 -g 0 ; 0 0 0 1 0 0 ; 0 0 0 0 0 0 ; 0 0 0 0 0 1 ; 0 0 0 0 0 0];
B=[0,0;-1/(sqrt(2)*m),1/(sqrt(2)*m);0 0 ;1/(sqrt(2)*m), 1/(sqrt(2)*m); 0,0;-(sqrt(2)/2)*(a-b)/J,(sqrt(2)/2)*(a-b)/J];
C=eye(6);
K=(place(A,B,[-1,-2,-3,-4,-5,-6]));

%initial conditions
x0=[0;10;4000;-50;0.1;0.01];

%Zero Input
[t,x]=ode45(@(t,x)(A)*x,[0:0.01:100],x0); %removed B*K as it = 0 in zero input
plot(t,C*x')
title('Zero Feedback Linear Model')
xlabel('time (s)')
ylabel('state variable units')
legend({'x-position(m)','xdot-velocity(m/s)','z-position(m)','zdot-velocity(m/s)','theta-angular position(rad)','thetadot-angular velocity(rad/s)'},'Location','southwest')