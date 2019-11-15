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

z0 = [.55; p(20);0; 0]; %y, thk, tha, vy, vk, va ,uank^2                % set initial state

% set guess
tspan=[0 .5];                                       % simulation final time
 %% 
n=20; %number of values to test for each control var
Kk=linspace(.1,100,n);
Bk=linspace(.001,15,n);
%Ka=linspace(1,500,n);
%Ba=linspace(.1,50,n);

combs_to_check=allcomb(Kk,Bk);%,Ka,Ba);
k=length(combs_to_check);
valid_configs=[];
not_valid_configs=[];
tic
for i=1:k
    i
    k
    ctrl=combs_to_check(i,:);
    %tic
    [sol,uout]=simulate_leg_rmhb_GRAC_paramsweep_knee(z0,ctrl,p,tspan);
    %toc
   % animate_param_sweep(sol,p, .1)
    
    validflag=check_constraints_rmhb_knee(sol,p);
    if validflag==1
        maxj=get_max_jerk_rmhb(sol,p);
        valid_configs=[valid_configs; ctrl maxj];
    else
        not_valid_configs=[not_valid_configs; ctrl];
    end
end
disp('total time')
toc
    %%
    close all
    figure
    plot(valid_configs(:,end),'k*')
    %%
    [val, idx]=min(abs(valid_configs(:,end)));
    ctrl_opt=valid_configs(idx,1:2);
    [sol,uout]=simulate_leg_rmhb_GRAC_paramsweep_knee(z0,ctrl_opt,p,tspan);
    animate_param_sweep_knee(sol,p,.1)
    
    %%
    [val, idx]=max(abs(valid_configs(:,end)));
    ctrl_nopt=valid_configs(idx,1:2);
    [sol,uout]=simulate_leg_rmhb_GRAC_paramsweep_knee(z0,ctrl_nopt,p,tspan);
    animate_param_sweep_knee(sol,p,.1)
                             
    %%     
        
    [sol,uout]=simulate_leg_rmhb_GRAC_paramsweep_knee(z0,[2   .1000  ],p,tspan);
    max(abs(uout))
    animate_param_sweep_knee(sol,p,.1)

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