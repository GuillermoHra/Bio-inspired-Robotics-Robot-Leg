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

z0 = [.7; p(20);p(21); 0; 0;0; 0]; %y, thk, tha, vy, vk, va ,uank^2                % set initial state

% set guess
tspan=[0 1.5];                                       % simulation final time
h=linspace(.5,1,6);

%save('filename','valid_configs','solarray') -v7.3
 %% 
n=10; %number of values to test for each control var
Kk=linspace(.1,200,n);
Bk=linspace(.01,20,n);

Kk=100;
Bk=50;
Ka=linspace(.1,100,n);
Ba=linspace(.01,20,n);

combs_to_check=allcomb(Kk,Bk,Ka,Ba);
k=length(combs_to_check(:,1));
valid_configs=[];
not_valid_configs=[];
solarray=[];
tic
for j=1:length(h)
    
for i=1:length(combs_to_check(:,1))
    [i k j length(h)]
    
    ctrl=combs_to_check(i,:);
    z0 = [h(j); p(20);p(21); 0; 0;0; 0];
    %tic
    [sol,uout]=simulate_leg_rmhb_GRAC_paramsweep(z0,ctrl,p,tspan);
    sol.u=uout;
    nsol.u=uout;
    nsol.x=sol.x;
    %toc
   % animate_param_sweep(sol,p, .1)
    
    validflag=check_constraints_rmhb1(sol,p, uout)
    if validflag==1
        maxj=get_max_jerk_rmhb(sol,p);
        valid_configs=[valid_configs; ctrl z0(1) sum(uout(1,:).^2 + uout(2,:).^2)  maxj];
        
        solarray=[solarray;nsol];
        %animate_param_sweep(sol,p,.1)
    else
        not_valid_configs=[not_valid_configs; ctrl z0(1)];
    end
end
end
disp('total time')
toc
    %%
   % close all
    figure
    plot(valid_configs(:,end),'k*')
    
    figure
    X = valid_configs(:,1);
    Y = valid_configs(:,2);
    Z = valid_configs(:,end);
    plot3(X, Y, Z, '*');
    title('Knee')
    
    figure
    X = valid_configs(:,3);
    Y = valid_configs(:,4);
    Z = valid_configs(:,end);
    plot3(X, Y, Z, '*');
    title('Ankle')
    
    
       figure
    X = valid_configs(:,3);
    Y = valid_configs(:,4);
    Z = valid_configs(:,end);
    Z0=valid_configs(:,5);
   rgb = vals2colormap(Z0);
    %plot3(X,Y,Z,'marker','.','markeredgecolor',rgb)
    scatter3(X,Y,Z,10,rgb)
    xlabel('K')
    ylabel('B')
    zlabel('Max Jerk')
    colorbar
    caxis([min(Z0) max(Z0)])
    title('Ankle')
    
    
        
       figure
    X = valid_configs(:,1);
    Y = valid_configs(:,2);
    Z = valid_configs(:,end);
    Z0=valid_configs(:,5);
   rgb = vals2colormap(Z0);
    %plot3(X,Y,Z,'marker','.','markeredgecolor',rgb)
    scatter3(X,Y,Z,10,rgb)
    xlabel('K')
    ylabel('B')
    zlabel('Max Jerk')
    colorbar
    caxis([min(Z0) max(Z0)])
    title('Knee')
    
    
    
    
    figure
hold on
for i = 1 : length(solarray)
   t1 = solarray(i).u(2,:);
   plot(solarray(i).x,t1);
end
ylim([-5,5])

xlabel('Time (s)');
ylabel('Torque (Nm)');
title('Ankle Torque')
    
figure
hold on
for i = 1 : length(solarray)
   t1 = solarray(i).u(1,:);
   plot(solarray(i).x,t1);
end
xlabel('Time (s)');
title('Knee Torque')
ylim([-5,5])
ylabel('Torque (Nm)');





    
    %%
    minjerkctrlsidx=[]
    for i=1:length(h)
        z0h=h(i);
        minjerk=1000;
        minjerkidx=0;
        for j=1:length(valid_configs(:,1))
            
            if valid_configs(j,5)==z0h
                if valid_configs(j,7)<minjerk
                    minjerk=valid_configs(j,7);
                    minjerkidx=j;
                end
            end
        end
        minjerkctrlsidx=[minjerkctrlsidx;minjerkidx,z0h];
    end
    
    maxjerkctrlsidx=[]
    for i=1:length(h)
        z0h=h(i);
        maxjerk=0;
        maxjerkidx=0;
        for j=1:length(valid_configs(:,1))
            
            if valid_configs(j,5)==z0h
                if valid_configs(j,7)>maxjerk
                    maxjerk=valid_configs(j,7);
                    maxjerkidx=j;
                end
            end
        end
        maxjerkctrlsidx=[maxjerkctrlsidx;maxjerkidx,z0h];
    end
    
    
    
    figure
    hold on
    legendnames=[];
for i = 1 : length(minjerkctrlsidx)
    idx=minjerkctrlsidx(i,1);
    if idx ~=0
     t1 = solarray(idx).u(1,:);
   plot(solarray(idx).x,t1);
   legendnames=[legendnames;num2str(minjerkctrlsidx(i,2))];
    end
end
legendnames
   hleg= legend(legendnames)
   
    htitle = get(hleg,'Title');
    set(htitle,'String','Initial Height (m)')
    
    title('Knee Torque Profiles for Min Jerk at Different Initial Heights')
    xlabel('Time (s)')
    ylabel('Torque (Nm)')
    ylim([-2.1,2.1])
    
    
        
    figure
    hold on
    legendnames=[];
for i = 1 : length(minjerkctrlsidx)
    idx=minjerkctrlsidx(i,1);
    if idx ~=0
     t1 = solarray(idx).u(2,:);
   plot(solarray(idx).x,t1);
   legendnames=[legendnames;num2str(minjerkctrlsidx(i,2))];
    end
end
legendnames
   hleg= legend(legendnames)
   
    htitle = get(hleg,'Title');
    set(htitle,'String','Initial Height (m)')
    
    title('Ankle Torque Profiles for Min Jerk at Different Initial Heights')
    xlabel('Time (s)')
    ylabel('Torque (Nm)')
    ylim([-2.1,2.1])

    
    
    colors=['r','b','g','k','m','c']
    
    figure
    hold on
    legendnames=[];
    k=1;
for i = 1 : length(minjerkctrlsidx)
    idx=minjerkctrlsidx(i,1);
    if idx ~=0
        color1=colors(k);
        k=k+1;
        if k>length(colors)
            k=1;
        end
     t1 = solarray(idx).u(2,:);
   plot(solarray(idx).x,t1,'-','color',color1);
   legendnames=[legendnames;num2str(minjerkctrlsidx(i,2))];
    end
end
k=1;
for i = 1 : length(maxjerkctrlsidx)
    idx=maxjerkctrlsidx(i,1);
    if idx ~=0
     t1 = solarray(idx).u(2,:);
        color1=colors(k);
        k=k+1;
        if k>length(colors)
            k=1;
        end
   plot(solarray(idx).x,t1,'--','colo',color1);
   legendnames=[legendnames;num2str(maxjerkctrlsidx(i,2))];
    end
end

legendnames
   hleg= legend(legendnames)
   
    htitle = get(hleg,'Title');
    set(htitle,'String','Initial Height (m)')
    
    title('Ankle Torque Profiles for Max/Min Jerk at Different Initial Heights')
    xlabel('Time (s)')
    ylabel('Torque (Nm)')
    ylim([-2.1,2.1])
    pause(.1)
    
    
    
    
    
    %%
    
   % figure
%    goodsolflag=0;
%     i=1
%     while goodsolflag==0
%     [val, idx]=min(abs(valid_configs(:,end)));
%     ctrl_opt=valid_configs(idx,1:4);
%     [solc,uoutc]=simulate_leg_rmhb_GRAC_paramsweep(z0,ctrl_opt,p,tspan);
%     goodsolflag=check_constraints_rmhb1(solc,p, uoutc)
%     if goodsolflag==0
%        valid_configs(idx,:) = [];
%     end
%     if length(valid_configs)==0
%         disp('no valid configs')
%         break
%     end
%     i=i+1
%     end
%     figure
%     animate_param_sweep(solc,p,.1)
    
    %%
    figure
    [val, idx]=min(abs(valid_configs(:,end)));
    ctrl_opt=valid_configs(idx,1:4);
     %z0 = [h(j); p(20);p(21); 0; 0;0; 0];
    z0=[valid_configs(idx,5) ;p(20);p(21);0;0;0;0];
    [solc,uoutc]=simulate_leg_rmhb_GRAC_paramsweep(z0,ctrl_opt,p,tspan);
    
    animate_param_sweep(solc,p,.1)
    %%
    figure
    [val, idx]=max(abs(valid_configs(:,end)));
    ctrl_nopt=valid_configs(idx,1:4);
    z0=[valid_configs(idx,5) ;p(20);p(21);0;0;0;0];
    [sol2,uout2]=simulate_leg_rmhb_GRAC_paramsweep(z0,ctrl_nopt,p,tspan);
    animate_param_sweep(sol2,p,.1)
    
    
    %%

    figure
    animate_param_sweep_twocompare(sol1,sol2,p,.1)
    
                             
    %%  
    figure
       p = parameters();    

    animate_param_sweep_twocompare(sol2,sol1,p,.1)
    %%
%       figure
%        p = parameters();    
%     ctrl_noopt=not_valid_configs(2,1:4);
%     [sol3,uout3]=simulate_leg_rmhb_GRAC_paramsweep(z0,ctrl_noopt,p,tspan);
%     animate_param_sweep_twocompare(sol3,sol3,p,.1)
%     
    
                             
    %%     
    p = parameters();    

    [sol,uout]=simulate_leg_rmhb_GRAC_paramsweep( z0,[100   50.000   100    20.000],p,tspan);
    animate_param_sweep(sol,p,.1)

%%
 %[sol,uout]=simulate_leg_rmhb_GRAC(z0,ctrl,p,tspan);
%[t, kout,z, sols, fc] = hybrid_simulation_GRAC(z0,ctrl,p,tspan); % run simulation
%t=sol.x;
%z=sol.y;
%toc
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


%%

figure
hold on
for i = 1 : length(solarray)
   t1 = solarray(i).u(2,:);
   plot(t1);
end
ylim([-5,5])

xlabel('samples');
ylabel('Torque');
title('ankle torque')
    
figure
hold on
for i = 1 : length(solarray)
   t1 = solarray(i).u(1,:);
   plot(t1);
end
xlabel('samples');
title('knee torque')
ylim([-5,5])
ylabel('Torque');