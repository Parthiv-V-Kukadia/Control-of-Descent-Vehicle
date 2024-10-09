function DesignProblem01(controller,varargin)
% DesignProblem01   run simulation of powered descent vehicle
%
%   DesignProblem01('FunctionName') uses the controller defined by
%       the function 'FunctionName' - for example, in a file with
%       the name FunctionName.m - in the simulation.
%
%   DesignProblem01('FunctionName','P1',V1,'P2','V2',...) optionally
%       defines a number of parameter values:
%
%           'team' : a name (e.g., 'FirstName LastName') that, if defined,
%                    will appear on the figure window
%
%           'datafile' : a filename (e.g., 'data.mat') where, if defined,
%                        data will be logged and saved. 
%                          
%            Note: The two datafields processdata > lthrust_true and
%            processdata > rthrust_true represent the actual values 
%            of thruster used considering thruster limits. The values in 
%            controllerdata > actuators are the thrust values generated
%            by your controller. 
%
%           'moviefile' : a filename (e.g., 'movie.mp4') where, if defined,
%                         a movie of the simulation will be saved
%
%           'snapshotfile' : a filename (e.g., 'snap.pdf') where, if
%                            defined, a PDF with a snapshot of the last
%                            frame of the simulation will be saved
%
%           'controllerdatatolog' : a cell array (e.g., {'y','xhat'}) with
%                                   the names of fields in controller.data -
%                                   if 'datafile' is defined (so data is
%                                   logged), then values in these fields will
%                                   also be logged and saved
%
%           'reference' : a 6x1 matrix
%                           [x; xdot; z; zdot; theta; thetadot]
%                       that specifies reference values for different
%                       states
%
%           'tStop' : the time at which the simulation will stop (a
%                     positive number) - default value is 30
%
%           'initial' : a 6x1 matrix
%                           [x; xdot; z; zdot; theta; thetadot]
%                       that specifies initial values - by default, these
%                       values are [0; 10; 4000; -50; 0.1; 0.01]
%
%           'display' : a flag...
%
%                       - If true, it will clear the current figure and
%                         will show the simulation. To quite, type 'q' when
%                         this figure is in the foreground.
%
%                       - If false, it will not show any graphics and will
%                         run the simulation as fast as possible (not in
%                         real-time).

% Parse the arguments
% - Create input parser
p = inputParser;
% - Parameter names must be specified in full
p.PartialMatching = false;
% - This argument is required, and must be first
addRequired(p,'controller',@ischar);
% - These parameters are optional, and can be in any order
addParameter(p,'team',[],@ischar);
addParameter(p,'datafile',[],@ischar);
addParameter(p,'moviefile',[],@ischar);
addParameter(p,'snapshotfile',[],@ischar);
addParameter(p,'controllerdatatolog',[],@iscell);
addParameter(p,'reference',[0;0;0;0;0;0],@(x) validateattributes(x,{'numeric'},{'size',[6 1]}));
addParameter(p,'tStop',30,@(x) isscalar(x) && isnumeric(x) && (x>0));
addParameter(p,'initial',[0;10;4000;-50;0.1;0.01],...
                         @(x) validateattributes(x,{'numeric'},{'size',[6 1]}));
addParameter(p,'display',true,@islogical);
% - Apply input parser
parse(p,controller,varargin{:});
% - Extract parameters
process = p.Results;
% - Check that the 'controller' function exists
if (exist(process.controller,'file')~=2)
    error('Controller ''%s'' does not exist.',process.controller);
end

% Setup the simulation
[process,controller] = SetupSimulation(process);

% Run the simulation
RunSimulation(process,controller);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTIONS THAT WILL CHANGE FOR DIFFERENT PROCESSES
%

function [process,controller] = SetupSimulation(process)

% DEFINE CONSTANTS

% Constants related to simulation.
% - State time.
process.tStart = 0;
% - Time step.
process.tStep = 1/50;
% - Names of things to log in datafile, if desired
process.processdatatolog = {'t','x','xdot','z','zdot', 'theta','thetadot','lthrust_true','rthrust_true'};

% Constants related to physical properties.
% - Acceleration due to gravity of Mars.
process.g = 3.711;
% - Mass
process.m = 950;
% - Dimensions
process.a = 2.6/2; %half of height 
process.b = 3.4/2; %half of length
% - Moment of Inertia
process.J = process.m*((2*process.a)^2 + (2*process.b)^2)/12;
% - Maximum thrust
process.maxthrust = 3000;

% DEFINE VARIABLES

% Time
process.t = 0;
% X-Position
process.x = process.initial(1,1);
% X-Velocity
process.xdot = process.initial(2,1);
% Z-Position
process.z = process.initial(3,1);
% Z-Velocity
process.zdot = process.initial(4,1);
% Orientation
process.theta = process.initial(5,1);
% Angular Velocity
process.thetadot = process.initial(6,1);
% - Actual thrust values
process.lthrust_true = 0;
process.rthrust_true = 0;


% DEFINE CONTROLLER

% Functions
% - get handles to user-defined functions 'init' and 'run'
controller = eval(process.controller);
controller.name = process.controller;
% Parameters
% - define a list of constants that will be passed to the controller
names = {'tStep','g','m','J','maxthrust'};
% - loop to create a structure with only these constants
controller.parameters = struct;
for i=1:length(names)
    controller.parameters.(names{i}) = process.(names{i});
end
% Storage
controller.data = struct;
% Status
controller.running = true;
% Init
tic
try
    [controller.data] = ...
        controller.init(controller.parameters, ...
                        controller.data);
catch exception
    warning(['The ''init'' function of controller\n     ''%s''\n' ...
             'threw the following error:\n\n' ...
             '==========================\n' ...
             '%s\n', ...
             '==========================\n\n' ...
             'Turning off controller and setting all\n' ...
             'actuator values to zero.\n'],controller.name,getReport(exception));
	controller.actuators = ZeroActuators();
    controller.running = false;
end
controller.tComputation = toc;
% Get reference values
controller.references = GetReferences(process);
% Get sensor values
controller.sensors = GetSensors(process);
% Get actuator values (run controller)
controller = RunController(controller);
% Update actuator data
process.lthrust_true = max(0,min(controller.actuators.lthrust,process.maxthrust));
process.rthrust_true = max(0,min(controller.actuators.rthrust,process.maxthrust));

end

function references = GetReferences(process)
try
    references = struct('x',process.reference(1,1),'xdot',process.reference(2,1),...
    'z',process.reference(3,1),'zdot',process.reference(4,1),'theta',process.reference(5,1),...
    'thetadot',process.reference(6,1));
    
catch exception
    warning(['The ''references'' that were passed to\n' ...
             'DesignProblem01 threw the following error:\n\n' ...
             '==========================\n' ...
             '%s\n', ...
             '==========================\n\n' ...
             'Using default reference of 0 for all states.\n'],getReport(exception));
    references = struct('x',0, 'xdot', 0, 'z', 0, 'zdot', 0, 'theta', 0, 'thetadot', 0);
end
end

function sensors = GetSensors(process)
sensors.t = process.t;
sensors.x = process.x;
sensors.xdot = process.xdot;
sensors.z = process.z;
sensors.zdot = process.zdot;
sensors.theta = process.theta;
sensors.thetadot = process.thetadot;
% Add noise
%   (nothing)
end

function [t,X] = Get_TandX_From_Process(process)
t = process.t;
X = [process.x; process.xdot; process.z; process.zdot; process.theta; process.thetadot];
end

function u = GetInput(process,actuators)
% Copy input from actuators
u = [actuators.lthrust; actuators.rthrust];

% Bound input
for i=1:length(u)
    if (u(i) < 0)
        u(i) = 0;
    elseif (u(i) > process.maxthrust)
        u(i) = process.maxthrust;
    end
end

end

function process = Get_Process_From_TandX(t,X,process)
process.t = t;
process.x = X(1,1);
process.xdot = X(2,1);
process.z = X(3,1);
process.zdot = X(4,1);
process.theta = X(5,1);
process.thetadot = X(6,1);
end

function Xdot = GetXDot(t,X,u,process)
% unpack x
x = X(1,1); xdot = X(2,1);
z = X(3,1); zdot = X(4,1);
theta = X(5,1); thetadot = X(6,1);
% compute rates of change
d_x = xdot;
d_xdot = (1/process.m)*[cos(d2r(45)+theta), -cos(d2r(45)-theta)]*u;

d_z = zdot;
d_zdot = (1/process.m)*[sin(d2r(45)+theta), sin(d2r(45)-theta)]*u -process.g;

d_theta = thetadot;
d_thetadot = (1/process.J)*[(-process.b*sind(45)+process.a*cosd(45)), ...
    (process.b*sind(45)-process.a*cosd(45))]*u;
% pack xdot
Xdot = [d_x; d_xdot; d_z; d_zdot; d_theta; d_thetadot];
end

function iscorrect = CheckActuators(actuators)
iscorrect = false;
if all(isfield(actuators,{'lthrust','rthrust'}))&&(length(fieldnames(actuators))==2)
    if isnumeric(actuators.lthrust) && isnumeric(actuators.rthrust)
        if isscalar(actuators.lthrust) && isscalar(actuators.rthrust)
            iscorrect = true;
        end
    end
end
end

function actuators = ZeroActuators()
actuators = struct('lthrust',0,'rthrust',0);
end

function fig = UpdateFigure(process,controller,fig)
if (isempty(fig))
    % CREATE FIGURE
    
    % Clear the current figure.
    clf;
    
    % Create an axis for text (it's important this is in the back,
    % so you can rotate the view and other stuff!)
    fig.text.axis = axes('position',[0 0 1 1]);
    axis([0 1 0 1]);
    hold on;
    axis off;
    fs = 14;
    if (controller.running)
        status = 'ON';
        color = 'g';
    else
        status = 'OFF';
        color = 'r';
    end
    fig.text.status=text(0.05,0.975,...
        sprintf('%s',status),...
        'fontweight','bold','fontsize',fs,...
        'color',color,'verticalalignment','top');
    fig.text.time=text(0.2,0.975,...
        sprintf('time = %6.2f / %6.2f\n',process.t,process.tStop),...
        'fontsize',fs,'verticalalignment','top','fontname','monaco');
    fig.text.teamname=text(0.05,0.06,...
        sprintf('%s',process.team),...
        'fontsize',fs,'verticalalignment','top','fontweight','bold');
    
    fig.x.axis = axes('position',[0.55,0.6,0.4,0.35],'fontsize',fs);
    axis([0,process.tStop,-Inf,Inf]);
    hold on;
    fig.x.xdot = plot(nan,nan,'linewidth',2);
    fig.x.zdot = plot(nan,nan,'linewidth',2);
    fig.x.thetadot = plot(nan,nan,'linewidth',2);
    fig.x.legend = legend({'$\dot{x}$','$\dot{z}$','$\dot{\theta}$'},'Interpreter','latex');
    xlabel('time');
    
    fig.u.axis = axes('position',[0.55,0.1,0.4,0.35],'fontsize',fs);
    axis([0,process.tStop,-Inf,Inf]);
    hold on;
    fig.u.ul_con = plot(nan,nan,'linewidth',1.5,'linestyle','--','color','red');
    fig.u.ur_con = plot(nan,nan,'linewidth',1.5,'linestyle','--','color','blue');
    fig.u.ul_true = plot(nan,nan,'linewidth',2,'color','red');
    fig.u.ur_true = plot(nan,nan,'linewidth',2,'color','blue');
    fig.u.umin = plot([0 process.tStop],[0 0],...
                      'linewidth',1,'linestyle','--','color','k');
    fig.u.umax = plot([0 process.tStop],process.maxthrust*[1 1],...
                      'linewidth',1,'linestyle','--','color','k');
    fig.u.baseline = plot([0 process.tStop],[-200,-200],...
                      'linewidth',1,'color','w');
    fig.u.legend = legend({'L-thrust - controller', 'R-thrust - controller', 'L-thrust - true', 'R-thrust - true'});
    xlabel('time');
    
    % Create an axis for the view.
    fig.view.axis = axes('position',[0.05 0.1 0.4 0.8],'fontsize',fs);
    axis([-500,500,-100,5000]);
    hold on;
    fig.view.ini = plot(nan, nan,'k+', 'MarkerSize',7);
    fig.view.xz = plot(nan,nan,'linewidth',2.5);
    fig.view.zmin = plot([-1e7 1e7],[0 0],...
                      'linewidth',2.5,'color',[0.2 0 0]);
    fig.view.legend = legend({'initial state','trajectory', 'ground'});
    grid on;
    xlabel('x position');
    ylabel('z position');
    
    % Make the figure respond to key commands.
    set(gcf,'KeyPressFcn',@onkeypress);
end

% UPDATE FIGURE

set(fig.text.time,'string',sprintf('t = %6.2f / %6.2f\n',process.t,process.tStop));
if not (controller.running)
    status = 'OFF';
    color = 'r';
else
    status = 'ON';
    color = 'g';
end
set(fig.text.status,'string',sprintf('%s',status),'color',color);


% XZ fig update
x = [get(fig.view.xz,'xdata') process.x];
z = [get(fig.view.xz,'ydata') process.z];
set(fig.view.xz,'xdata',x,'ydata',z);

xini = [get(fig.view.ini,'xdata') process.initial(1,1)];
zini = [get(fig.view.ini,'ydata') process.initial(3,1)];
set(fig.view.ini,'xdata',xini,'ydata',zini);

% Plot states
t = [get(fig.x.xdot,'xdata') process.t];
xdot = [get(fig.x.xdot,'ydata') process.xdot];
zdot = [get(fig.x.zdot,'ydata') process.zdot];
thetadot = [get(fig.x.thetadot,'ydata') process.thetadot];

set(fig.x.xdot,'xdata',t,'ydata',xdot);
set(fig.x.zdot,'xdata',t,'ydata',zdot);
set(fig.x.thetadot,'xdata',t,'ydata',thetadot);

% Plot control input
t = [get(fig.u.ul_con,'xdata') process.t];
set(fig.u.ul_con,'xdata',t,'ydata',[get(fig.u.ul_con,'ydata') controller.actuators.lthrust]);
set(fig.u.ur_con,'xdata',t,'ydata',[get(fig.u.ur_con,'ydata') controller.actuators.rthrust]);
set(fig.u.ul_true,'xdata',t,'ydata',[get(fig.u.ul_true,'ydata') process.lthrust_true]);
set(fig.u.ur_true,'xdata',t,'ydata',[get(fig.u.ur_true,'ydata') process.rthrust_true]);

drawnow;
end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTIONS THAT (I HOPE) WILL REMAIN THE SAME FOR ALL PROCESSES
%

function RunSimulation(process,controller)

% START-UP

% Create empty figure.
fig = [];

% Flag to stop simulation on keypress.
global done
done = false;

% Start making movie, if necessary.
if (~isempty(process.moviefile))
    myV = VideoWriter(process.moviefile,'MPEG-4');
    myV.Quality = 100;
    myV.FrameRate = 1/process.tStep;
    open(myV);
end

% LOOP

% Loop until break.
tStart = tic;
while (1)
    
    % Update figure (create one if fig is empty).
    if (process.display)
        fig = UpdateFigure(process,controller,fig);
    end
    
    % Update data.
    if (~isempty(process.datafile) && controller.running)
        [process,controller] = UpdateDatalog(process,controller);
    end
    
    % If making a movie, store the current figure as a frame.
    if (~isempty(process.moviefile))
        frame = getframe(gcf);
        writeVideo(myV,frame);
    end
    
    % Stop if time has reached its maximum.
    if ((process.t + eps >= process.tStop)||done)
        break;
    end
    
    % Update process (integrate equations of motion).
    [process,controller] = UpdateProcess(process,controller);
    
    % Wait if necessary, to stay real-time.
    if (process.display)
        while (toc(tStart)<process.t-process.tStart)
            % Do nothing
        end
    end
    
end

% SHUT-DOWN

% Close and save the movie, if necessary.
if (~isempty(process.moviefile))
    for i=1:myV.FrameRate
        frame = getframe(gcf);
        writeVideo(myV,frame);
    end
    close(myV);
end

% Save the data.
if (~isempty(process.datafile))
    processdata = process.log.process; %#ok<NASGU>
    controllerdata = process.log.controller; %#ok<NASGU>
    save(process.datafile,'processdata','controllerdata');
end

% Save the snapshot, if necessary.
if (~isempty(process.snapshotfile))
    set(gcf,'paperorientation','landscape');
    set(gcf,'paperunits','normalized');
    set(gcf,'paperposition',[0 0 1 1]);
    print(gcf,'-dpdf',process.snapshotfile);
end

end


function [process,controller] = UpdateDatalog(process,controller)
% Create data log if it does not already exist.
if (~isfield(process,'log'))
    process.log = struct('process',struct,...
                         'controller',struct('tComputation',[],...
                                             'sensors',struct,...
                                             'actuators',struct,...
                                             'data',struct),...
                         'count',0);
end
% Increment log count.
process.log.count = process.log.count+1;
% Write data to log.
for i=1:length(process.processdatatolog)
    name = process.processdatatolog{i};
    process.log.process.(name)(:,process.log.count) = process.(name);
end
process.log.controller.tComputation(:,process.log.count) = ...
    controller.tComputation;
names = fieldnames(controller.sensors);
for i=1:length(names)
    name = names{i};
    process.log.controller.sensors.(name)(:,process.log.count) = ...
        controller.sensors.(name);
end
names = fieldnames(controller.actuators);
for i=1:length(names)
    name = names{i};
    process.log.controller.actuators.(name)(:,process.log.count) = ...
        controller.actuators.(name);
end
for i=1:length(process.controllerdatatolog)
    name = process.controllerdatatolog{i};
    try
        process.log.controller.data.(name)(:,process.log.count) = ...
            controller.data.(name);
    catch exception
        warning(['Saving element ''%s'' of data for controller\n',...
                 '     ''%s''',...
                 'threw the following error:\n\n' ...
                 '==========================\n' ...
                 '%s\n', ...
                 '==========================\n\n' ...
                 'Turning off controller and setting all\n' ...
                 'actuator values to zero.\n'],...
                 name,controller.name,getReport(exception));
        controller.actuators = ZeroActuators();
        controller.running = false;
        return
    end
end
end


function [process,controller] = UpdateProcess(process,controller)
% Integrate equations of motion
[t0,X] = Get_TandX_From_Process(process);
u = GetInput(process,controller.actuators);
% Update actuator data
process.lthrust_true = u(1);
process.rthrust_true = u(2);

[t,X] = ode45(@(t,X) GetXDot(t,X,u,process),[t0 t0+process.tStep],X,odeset('reltol',1e-6,'abstol',1e-9));
process = Get_Process_From_TandX(t(end),X(end,:)',process);
% Get reference values
controller.references = GetReferences(process);
% Get sensor values
controller.sensors = GetSensors(process);
% Get actuator values (run controller)
controller = RunController(controller);
end

function controller = RunController(controller)
if (controller.running)
    tic
    try
        [controller.actuators,controller.data] = ...
            controller.run(controller.sensors, ...
                              controller.references, ...
                              controller.parameters, ...
                              controller.data);
    catch exception
        warning(['The ''run'' function of controller\n     ''%s''\n' ...
                 'threw the following error:\n\n' ...
                 '==========================\n' ...
                 '%s\n', ...
                 '==========================\n\n' ...
                 'Turning off controller and setting all\n' ...
                 'actuator values to zero.\n'],controller.name,getReport(exception));
        controller.actuators = ZeroActuators();
        controller.running = false;
    end
    if (~isstruct(controller.actuators) || ~CheckActuators(controller.actuators))
        warning(['The ''run'' function of controller\n     ''%s''\n' ...
                 'did not return a structure ''actuators'' with the right\n' ...
                 'format. Turning off controller and setting all\n' ...
                 'actuator values to zero.\n'],controller.name);
        controller.actuators = ZeroActuators();
        controller.running = false;
    end
    controller.tRun = toc;
else
    controller.tRun = 0;
end
end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% HELPER FUNCTIONS
%


function d = r2d(r)
d = r*180/pi;
end

function r = d2r(d)
r = d*pi/180;
end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


