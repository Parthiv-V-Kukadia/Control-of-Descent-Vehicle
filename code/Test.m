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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Part 1

%Zero Input Ode Graph
x0=[0;10;4000;-50;0.1;0.01];
figure(1)
[t,x]=ode45(@(t,x)(A)*x,[0:0.01:100],x0); %removed B*K as it = 0 in zero input
plot(t,C*x')
title('Zero Feedback Linear Model');
xlabel('time (s)');
ylabel('state variable units');
legend({'x-position(m)','xdot-velocity(m/s)','z-position(m)','zdot-velocity(m/s)','theta-angular position(rad)','thetadot-angular velocity(rad/s)'},'Location','southwest');


%State feedback Ode graph
x00=[0;10;4000;-50;0.1;0.01];
figure(2)
[t,x]=ode45(@(t,x)(A-B*K)*x,[0:0.01:100],x00);
plot(t,C*x')
title('State Feedback Linear Model');
xlabel('time (s)');
ylabel('state variable units');
legend({'x-position(m)','xdot-velocity(m/s)','z-position(m)','zdot-velocity(m/s)','theta-angular position(rad)','thetadot-angular velocity(rad/s)'},'Location','northeast');
xlim([0,4]);

%Zero Input
Figure(3)
DesignProblem01('Controller_V2','datafile','output1.mat','tstop',210,'display',true)

%State feedback %Running this through the DesignProblem01.m code shows us
%the limitations of our controller as it will never converge seeing that
%our controller is a linear system trying to operate in a non-linear
%system.
figure(4)
DesignProblem01('Controller','datafile','output2.mat','tstop',210,'display',true)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%part 2 %Comparing initial conditions, to show that regardless of the
%conditions, it will always converge for a linear system, but never
%converge if we try to input it into a non-linear model.

%Easy conditions Ode graph
x000=[0;0;3500;0;0;0];
figure(5)
[t,x]=ode45(@(t,x)(A-B*K)*x,[0:0.01:100],x000);
plot(t,C*x')
title('Easy Conditions');
xlabel('time (s)');
ylabel('state variable units');
legend({'x-position(m)','xdot-velocity(m/s)','z-position(m)','zdot-velocity(m/s)','theta-angular position(rad)','thetadot-angular velocity(rad/s)'},'Location','northeast');
xlim([0,4]);

%Difficult conditions Ode graph
x0000=[40;10;4500;-60;0.1;0.01];
figure(6)
[t,x]=ode45(@(t,x)(A-B*K)*x,[0:0.01:100],x0000);
plot(t,C*x')
title('Difficult Conditions');
xlabel('time (s)');
ylabel('state variable units');
legend({'x-position(m)','xdot-velocity(m/s)','z-position(m)','zdot-velocity(m/s)','theta-angular position(rad)','thetadot-angular velocity(rad/s)'},'Location','northeast');
xlim([0,4]);

%Impossible conditions Ode graph
x00000=[0;10;1000;-50;0.1;0.01];
figure(7)
[t,x]=ode45(@(t,x)(A-B*K)*x,[0:0.01:100],x00000);
plot(t,C*x')
title('Impossible Conditions');
xlabel('time (s)');
ylabel('state variable units');
legend({'x-position(m)','xdot-velocity(m/s)','z-position(m)','zdot-velocity(m/s)','theta-angular position(rad)','thetadot-angular velocity(rad/s)'},'Location','northeast');
xlim([0,4]);


%Testing with easy initial conditions - to see if vehicle will will reach
%near z_e 
figure(8)
DesignProblem01('Controller','datafile','output3.mat','tstop',210,'initial',[0;0;3500;0;0;0])
%Testing with difficult conditions
figure(9)
DesignProblem01('Controller','datafile','output4.mat','tstop',210,'initial',[40;10;4500;-60;0.1;0.01])
%Testing with conditions where failure is inevitable
figure(10)
DesignProblem01('Controller','datafile','output5.mat','tstop',210,'initial',[0;10;1000;-50;0.1;0.01],'display',true)

