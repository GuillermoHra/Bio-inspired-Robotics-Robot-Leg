clear all; close all; clc;
tic
% We can organize our code by filing things in different folders.  These
% folders need to be added to the Matlab path so that it can run the files
% inside them even when they are not the current folder listed at the top
% of the Matlab window.  For more information about the current folder, see
% http://www.mathworks.com/help/matlab/matlab_env/understanding-file-locations-in-matlab.html
% For more information about the Matlab path, see
% http://www.mathworks.com/help/matlab/matlab_env/what-is-the-matlab-search-path.html
                                    % add AutoDerived, Modeling, and Visualization folders to Matlab path
addpath([pwd '/AutoDerived'])
p = parameters();                           % get parameters from file

z0 = [.5; p(20);p(21); 0; 0;0; 0]; %y, thk, tha, vy, vk, va ,uank^2                % set initial state

% set guess
tspan=[0 1];                                       % simulation final time
                                % control time points
ctrl = [1,.1,1,.1];% 10 100 10];                               % control values

%%
% % setup and solve nonlinear programming problem
problem.objective = @(x) objective_rmhb(x,z0,p,tspan);     % create anonymous function that returns objective
problem.nonlcon = @(x) constraints_rmhb(x,z0,p,tspan);     % create anonymous function that returns nonlinear constraints
problem.x0 = [1,.1,1,.1];                   % initial guess for decision variables
problem.lb = [ 0*ones(size(ctrl))];     % lower bound on decision variables
problem.ub = [  1000*ones(size(ctrl))];     % upper bound on decision variables
problem.Aineq = []; problem.bineq = [];         % no linear inequality constraints
problem.Aeq = []; problem.beq = [];             % no linear equality constraints
problem.options = optimset('Display','iter');   % set options
problem.solver = 'fmincon';                     % required
x = fmincon(problem);                           % solve nonlinear programming problem
%%
% Note that once you've solved the optimization problem, you'll need to 
% re-define tf, tfc, and ctrl here to reflect your solution.

                                        % simulation final time
                                  % control time points
ctrl = [x(1) x(2) x(3) x(4)];   
%ctrl.T = [x(2) ];  % control values
%%
 sol=simulate_leg_rmhb_GRAC(z0,ctrl,p,tspan);
%[t, kout,z, sols, fc] = hybrid_simulation_GRAC(z0,ctrl,p,tspan); % run simulation
t=sol.x;
z=sol.y;
toc
%%
% Plot COM for your submissions
% figure(1)
% COM = position_foot(z,p);
% plot(t,COM_GRAC_leg(2,:))
% xlabel('time (s)')
% ylabel('CoM Height (m)')
% title('Center of Mass Trajectory')
% 
% figure(2)
% plot(t,u(1,:),'r')
% hold on
% plot(t,u(2,:),'b')
% legend('knee torque','ankle torque')
% xlabel('time (s)')
% ylabel('Torque (Nm)')
% title('Torque Plots ')
% 
% figure(3)
% plot(t,fc)
% xlabel('time (s)')
% ylabel('Contact Force (N)')
% title('Contact Force Plots ')

%%
% Run the animation
figure(4)                          % get the coordinates of the points to animate
speed = .25;                                 % set animation speed
clf                                         % clear fig
animate_simple_GRAC(t,z,p,speed)                 % run animation