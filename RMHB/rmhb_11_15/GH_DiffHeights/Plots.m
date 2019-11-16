close all; clear; clc;

% load('GH_80_RLimit.mat')
% 
% p = parameters();
% tspan = [0 .75]; % simulation final time
% z0 = [.8; p(20);p(21); 0; 0;0; 0];
% 
% plot(valid_configs(:,end),'k*')
% 
% figure
% X = valid_configs(:,1);
% Y = valid_configs(:,2);
% Z = valid_configs(:,5);
% plot3(X, Y, Z, '*');
% title('Knee')
% 
% figure
% X = valid_configs(:,3);
% Y = valid_configs(:,4);
% Z = valid_configs(:,5);
% plot3(X, Y, Z, '*');
% title('Ankle')
% 
% [val, idx] = min(abs(valid_configs(:,end)));
% ctrl_opt = valid_configs(idx,1:4);
% [sol,uout] = simulate_leg_rmhb_GRAC_paramsweep( z0, ctrl_opt, p, tspan);
% 
% % Plot COM for your submissions
% figure
% plot(sol.x,sol.y(1,:))
% xlabel('time (s)')
% ylabel('COM Height (m)')
% title('Center of Mass Trajectory')
% 
% figure
% plot(sol.x,uout(1,:),'r')
% hold on
% plot(sol.x,uout(2,:),'b')
% legend('knee torque','ankle torque')
% xlabel('time (s)')
% ylabel('Torque (Nm)')
% title('Torque Plots ')

load('GH_50_RLimit.mat')
[val, idx] = min(abs(valid_configs(:,end)));
ctrl_opt_50 = valid_configs(idx,1:4);
[sol_50,uout_50] = simulate_leg_rmhb_GRAC_paramsweep( z0, ctrl_opt_50, p, tspan);

load('GH_60_RLimit.mat')
[val, idx] = min(abs(valid_configs(:,end)));
ctrl_opt_60 = valid_configs(idx,1:4);
[sol_60,uout_60] = simulate_leg_rmhb_GRAC_paramsweep( z0, ctrl_opt_60, p, tspan);

load('GH_70_RLimit.mat')
[val, idx] = min(abs(valid_configs(:,end)));
ctrl_opt_70 = valid_configs(idx,1:4);
[sol_70,uout_70] = simulate_leg_rmhb_GRAC_paramsweep( z0, ctrl_opt_70, p, tspan);

load('GH_80_RLimit.mat')
[val, idx] = min(abs(valid_configs(:,end)));
ctrl_opt_80 = valid_configs(idx,1:4);
[sol_80,uout_80] = simulate_leg_rmhb_GRAC_paramsweep( z0, ctrl_opt_80, p, tspan);

% Plot torques
figure
subplot(2,1,1)
plot(sol_50.x,uout_50(1,:),'r')
hold on
plot(sol_60.x,uout_60(1,:),'b')
plot(sol_70.x,uout_70(1,:),'g')
plot(sol_80.x,uout_80(1,:),'k')

legend('50', '60', '70', '80')
xlabel('time (s)')
ylabel('Torque (Nm)')
title('Knee')

subplot(2,1,2)
hold on
K_50 = ctrl_opt_50(1)+1; % knee
D_50 = ctrl_opt_50(2)+1;
scatter(K_50, D_50, 'o', 'filled', 'r');
K_60 = ctrl_opt_60(1); % knee
D_60 = ctrl_opt_60(2);
scatter(K_60, D_60, 'o', 'filled', 'b');
K_70 = ctrl_opt_70(1)+2; % knee
D_70 = ctrl_opt_70(2)+2;
scatter(K_70, D_70, 'o', 'filled', 'g');
K_80 = ctrl_opt_80(1); % knee
D_80 = ctrl_opt_80(2);
scatter(K_80, D_80, 'o', 'filled', 'k');
legend('50', '60', '70', '80')
xlabel('K')
ylabel('D')
xlim([0 300])
ylim([0 30])

figure
subplot(2,1,1)
plot(sol_50.x,uout_50(2,:),'r')
hold on
plot(sol_60.x,uout_60(2,:),'b')
plot(sol_70.x,uout_70(2,:),'g')
plot(sol_80.x,uout_80(2,:),'k')

legend('50', '60', '70', '80')
xlabel('time (s)')
ylabel('Torque (Nm)')
title('Ankle')

subplot(2,1,2)
hold on
K_50 = ctrl_opt_50(3); % knee
D_50 = ctrl_opt_50(4);
scatter(K_50, D_50, 'o', 'filled', 'r');
K_60 = ctrl_opt_60(3); % knee
D_60 = ctrl_opt_60(4);
scatter(K_60, D_60, 'o', 'filled', 'b');
K_70 = ctrl_opt_70(3); % knee
D_70 = ctrl_opt_70(4);
scatter(K_70, D_70, 'o', 'filled', 'g');
K_80 = ctrl_opt_80(3); % knee
D_80 = ctrl_opt_80(4);
scatter(K_80, D_80, 'o', 'filled', 'k');
legend('50', '60', '70', '80')
xlabel('K')
ylabel('D')
xlim([0 300])
ylim([0 30])


% % Plot K and D for knee and ankle at different heights
% load('GH_50_RLimit.mat')
% [val, idx] = min(abs(valid_configs(:,end)));
% ctrl_opt = valid_configs(idx,1:4);
% K = ctrl_opt(1); % knee
% D = ctrl_opt(2);
% scatter(K, D, 'o', 'filled', 'r');
% hold on
% K = ctrl_opt(3); % ankle
% D = ctrl_opt(4);
% scatter(K, D, 'd', 'filled', 'r');
% 
% load('GH_60_RLimit.mat')
% [val, idx] = min(abs(valid_configs(:,end)));
% ctrl_opt = valid_configs(idx,1:4);
% K = ctrl_opt(1); % knee
% D = ctrl_opt(2);
% hold on
% scatter(K, D, 'o', 'filled', 'b');
% K = ctrl_opt(3); % ankle
% D = ctrl_opt(4);
% hold on
% scatter(K, D, 'd', 'filled', 'b');
% 
% load('GH_70_RLimit.mat')
% [val, idx] = min(abs(valid_configs(:,end)));
% ctrl_opt = valid_configs(idx,1:4);
% K = ctrl_opt(1); % knee
% D = ctrl_opt(2);
% hold on
% scatter(K, D, 'o', 'filled', 'g');
% K = ctrl_opt(3); % ankle
% D = ctrl_opt(4);
% hold on
% scatter(K, D, 'd', 'filled', 'g');
% 
% load('GH_80_RLimit.mat')
% [val, idx] = min(abs(valid_configs(:,end)));
% ctrl_opt = valid_configs(idx,1:4);
% K = ctrl_opt(1); % knee
% D = ctrl_opt(2);
% hold on
% scatter(K, D, 'o', 'filled', 'k');
% K = ctrl_opt(3); % ankle
% D = ctrl_opt(4);
% hold on
% scatter(K, D, 'd', 'filled', 'k');
% legend('50K', '50A', '60K','60A', '70K', '70A', '80K', '80A')
% xlim([0 300])
% ylim([0 30])






% % Plot 10 K and D values at different heights
% load('GH_50_RLimit.mat')
% [val, idx] = mink(abs(valid_configs(:,end)), 10);
% ctrl_opt = [];
% for i=1 : length(idx)
%     ctrl_opt = [ctrl_opt ; valid_configs(idx(i), :)];
% end
% %ctrl_opt = valid_configs(idx,1:4);
% K = ctrl_opt(:,1); % knee
% D = ctrl_opt(:,2);
% scatter(K, D, 'o', 'filled', 'r');
% 
% hold on
% K = ctrl_opt(:,3); % ankle
% D = ctrl_opt(:,4);
% scatter(K, D, 'd', 'filled', 'r');
% % title('Knee')
% 
% load('GH_60_RLimit.mat')
% [val, idx] = mink(abs(valid_configs(:,end)), 10);
% ctrl_opt = [];
% for i=1 : length(idx)
%     ctrl_opt = [ctrl_opt ; valid_configs(idx(i), :)];
% end
% %ctrl_opt = valid_configs(idx,1:4);
% K = ctrl_opt(:,1)+1; % knee
% D = ctrl_opt(:,2)+1;
% scatter(K, D, 'o', 'filled', 'b');
% 
% % figure
% K = ctrl_opt(:,3); % ankle
% D = ctrl_opt(:,4);
% scatter(K, D, 'd', 'filled', 'b');
% % title('Knee')
% 
% load('GH_70_RLimit.mat')
% [val, idx] = mink(abs(valid_configs(:,end)), 10);
% ctrl_opt = [];
% for i=1 : length(idx)
%     ctrl_opt = [ctrl_opt ; valid_configs(idx(i), :)];
% end
% %ctrl_opt = valid_configs(idx,1:4);
% K = ctrl_opt(:,1)+2; % knee
% D = ctrl_opt(:,2)+2;
% scatter(K, D, 'o', 'filled', 'g');
% 
% % figure
% K = ctrl_opt(:,3); % ankle
% D = ctrl_opt(:,4);
% scatter(K, D, 'd', 'filled', 'g');
% % title('Knee')
% 
% load('GH_80_RLimit.mat')
% [val, idx] = mink(abs(valid_configs(:,end)), 10);
% ctrl_opt = [];
% for i=1 : length(idx)
%     ctrl_opt = [ctrl_opt ; valid_configs(idx(i), :)];
% end
% %ctrl_opt = valid_configs(idx,1:4);
% K = ctrl_opt(:,1)+1; % knee
% D = ctrl_opt(:,2)+1;
% scatter(K, D, 'o', 'filled', 'k');
% 
% % figure
% K = ctrl_opt(:,3); % ankle
% D = ctrl_opt(:,4);
% scatter(K, D, 'd', 'filled', 'k');
% xlim([0 300])
% ylim([0 30])
% % title('Knee')

