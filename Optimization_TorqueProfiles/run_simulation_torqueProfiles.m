clear all; close all; clc;

% We can organize our code by filing things in different folders.  These
% folders need to be added to the Matlab path so that it can run the files
% inside them even when they are not the current folder listed at the top
% of the Matlab window.  For more information about the current folder, see
% http://www.mathworks.com/help/matlab/matlab_env/understanding-file-locations-in-matlab.html
% For more information about the Matlab path, see
% http://www.mathworks.com/help/matlab/matlab_env/what-is-the-matlab-search-path.html

%setpath                                     % add AutoDerived, Modeling, and Visualization folders to Matlab path
path = 'C:\Users\Usuario\Documents\MATLAB\Bio-inspired Robotics\Final Project Landing Robot\Bio-inspired-Robotics-Robot-Leg';
addpath([path])
addpath([path '\AutoDerived'])
p = parameters();                           % get parameters from file
thki=deg2rad(90);
thai=deg2rad(75);
z0 = [.5; thki;thai; 0; 0;0; 0]; %y, thk, tha, vy, vk, va ,uank^2                % set initial state

% set guess
tf = .5;                                        % simulation final time          
ctrl.T = [ 0 -5 0 -2 ];    %good: 0 -5 0 -2    % was: -9.0 -2.0 -8.0 -1.0 control values
ctrl.dur = 0.3; % was: .3 maybe increase it?
ctrl.land_time = 0;

% % setup and solve nonlinear programming problem
problem.objective = @(x) objective_torqueProfiles(x,z0,p);     % create anonymous function that returns objective
problem.nonlcon = @(x) constraints_torqueProfiles(x,z0,p);     % create anonymous function that returns nonlinear constraints
problem.x0 = [tf  ctrl.T];                   % initial guess for decision variables
problem.lb = [.1 -20*ones(size(ctrl.T))];     % lower bound on decision variables
problem.ub = [2  20*ones(size(ctrl.T))];     % upper bound on decision variables
problem.Aineq = []; problem.bineq = [];         % no linear inequality constraints
problem.Aeq = []; problem.beq = [];             % no linear equality constraints
problem.options = optimset('Display','iter');   % set options
problem.solver = 'fmincon';                     % required
x = fmincon(problem);                           % solve nonlinear programming problem

% Note that once you've solved the optimization problem, you'll need to 
% re-define tf, tfc, and ctrl here to reflect your solution.

tf = x(1);                                        % simulation final time
% control time points
ctrl.T = [x(2) x(3) x(4) x(5)];                               % control values
[t, z, u, indices, sols, land_time, fc, angError] = hybrid_simulation_torqueProfiles(z0,ctrl,p,[0 tf]); % run simulation

% Plot COM for your submissions
figure(1)
COM = position_foot(z,p);
plot(t,COM(2,:))
xlabel('time (s)')
ylabel('CoM Height (m)')
title('Center of Mass Trajectory')

figure(2)
plot(t,u(1,:),'r')
hold on
plot(t,u(2,:),'b')
legend('knee torque','ankle torque')
xlabel('time (s)')
ylabel('Torque (Nm)')
title('Torque Plots ')

figure(3)
plot(t,fc)
xlabel('time (s)')
ylabel('Contact Force (N)')
title('Contact Force Plots ')

%%
% Run the animation
figure(4)                          % get the coordinates of the points to animate
speed = .05;                                 % set animation speed
clf                                         % clear fig
animate_simple_GRAC(t,z,p,speed)                 % run animation