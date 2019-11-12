clear all; close all; clc;

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
tspan=[0 .3];                                       % simulation final time
  
n=10; %number of values to test for each control var
Kk=linspace(1,500,n);
Bk=linspace(.1,50,n);
Ka=linspace(1,500,n);
Ba=linspace(.1,50,n);

combs_to_check=allcomb(Kk,Bk,Ka,Ba);
k=length(combs_to_check);
valid_configs=[];
tic
for i=1:k
    i
    k
    ctrl=combs_to_check(i,:);
    %tic
    [sol,uout]=simulate_leg_rmhb_GRAC_paramsweep(z0,ctrl,p,tspan);
    %toc
   % animate_param_sweep(sol,p, .1)
    
    validflag=check_constraints_rmhb(sol,p);
    if validflag==1
        maxj=get_max_jerk_rmhb(sol,p);
        valid_configs=[valid_configs; ctrl maxj];
    end
end
disp('total time')
toc
    %%
    close all
    figure
    plot(valid_configs(:,end),'k*')
    %%
    
                             % control values


%%
 [sol,uout]=simulate_leg_rmhb_GRAC(z0,ctrl,p,tspan);
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
% figure(4)                          % get the coordinates of the points to animate
% speed = .25;                                 % set animation speed
% clf                                         % clear fig
% animate_simple_GRAC(t,z,p,speed)                 % run animation