function simulate_leg_GRAC()

close all
addpath([pwd '/AutoDerived'])
 %% Definte fixed paramters (obtained from CAD)
% p=[thh0;thh; m1; I1; m2 ;I2; m3 ;I3; m4; I4; l1; l2; l3 ;l4;lc2;lc3;lc4;lh0; g ]; %parameters
thh0=deg2rad(15);
thh=deg2rad(45);
m1=.5;
I1= 50*10^-6;
m2=.1;
I2= 10*10^-6;
m3=.1;
I3= 10*10^-6;
m4=.05;
I4= 5*10^-6;

l1=.05;
l2=.15;
l3=.15 ;
l4=.075;
lc2=l2/2;
lc3=l3/2;
lc4=l4/2;
lh0=.03;
g =9.81;

    %% Parameter vector make sure this matches Derive_everything_GRAC.m
    p=[thh0;thh; m1; I1; m2 ;I2; m3 ;I3; m4; I4; l1; l2; l3 ;l4;lc2;lc3;lc4;lh0; g ]; %parameters
    %p=[thh0;thh; m1; I1; m2 ;I2; m3 ;I3; m4; I4; l1; l2; l3 ;l4;lc2;lc3;lc4;lh0; g ]; %parameters
     %% Perform Dynamic simulation
    tspan = [0 .5];
    thki=deg2rad(90);
    thai=deg2rad(75);
    z0 = [.5; thki;thai; 0; 0;0; 0; 0; 0; 0];
    opts = odeset('AbsTol',1e-8,'RelTol',1e-6);
    sol = ode45(@dynamics,tspan,z0,opts,p);
    y_vals=sol.y(1,:);
    th1_vals=sol.y(2,:);
    th2_vals=sol.y(3,:);
    thm_vals=sol.y(7,:);
    % added to plot torque profiles for impedance controller
    Tknee = sol.y(9,:);
    Tankle = sol.y(10,:);
    
    %% Compute Energy
    E = energy_GRAC_leg(sol.y,p);
    %figure;
    %plot(sol.x,E);xlabel('Time (s)'); ylabel('Energy (J)');
    
    % plot torque profiles
    figure;
    plot(Tknee);
    hold on
    plot(Tankle);
    xlabel('Samples'); ylabel('Torque')
    legend('Knee', 'Ankle')
    
%      %% Compute foot position over time
%     rE = zeros(3,length(sol.x));
%     for i = 1:length(sol.x)
%         rE(:,i) = position_foot(sol.y(:,i),p);
%     end
%     figure(2); clf;
%     plot(sol.x,rE(1:2,:))
%     xlabel('Time (s)'); ylabel('Position (m)'); legend({'x','y'});
% 
%     figure(5); clf;
%     plot(sol.x,rad2deg(th1_vals))
%     hold on
%     plot(sol.x,rad2deg(th2_vals))
%     xlabel('Time (s)'); ylabel('Theta (deg)'); legend({'th1','th2'});

%animateSol(sol,p)
end

function tau = control_law(t,z,p)
%     % Controller gains, Update as necessary for Problem 1

        
     %desired angles of leg
         thki=deg2rad(90);   %MAKE SURE THESE MATCH AT THE TOP
    thai=deg2rad(75);
    
      thkd=thki;  %theta knee desired
      thad=thai;    %theta ankle desired
      thkc=z(2); %theta knee current
      thac=z(3); %theta ankle current
      thkvc=z(5);%theta knee velocity current
      thavc=z(6);%theta ankle velocity current
      %size(z)
      tau = [-(10*(thkc-thkd)+ 2*(thkvc)) ; -(50*(thac-thad)+ 5*(thavc))  ]; %WATCH FOR OVERDAMPING
      % tau = [0; -(500*(thac-thad)+ 5*(thavc))  ];
   
      %tau=[0;0];
   end

function Fc = contact_force(z,p)

    % Fixed parameters for contact
    K_c1 = 1000;
    D_c1 = 20;
    dC1  = deg2rad(0);
    

    r_E= position_foot(z, p);
    r_E_dot=velocity_foot(z,p);
    
    C=r_E(2)-dC1;
    Cdot=r_E_dot(2);
    if C>0
        Fc=0;
    else
        Fc=-K_c1*C - D_c1*Cdot;
    end
    
end

function dz = dynamics(t,z,p)
    y = z(1);     th1 = z(2);     th2 = z(3);
   yd= z(4);     th1d= z(5);     th2d = z(6);
    thm=z(7); thmd=z(8);
    
    bm=3;
    ksea=10; %sea
    Im=.01; %mot inertia param
    
    %----------------------------
    %WHAT WE NEED IS TO FIGURE OUT HOW TO SIMULATE THM vs TH2 (they are
    %different) 
    %----------------------------------------
    
    % Get mass matrix
    A = A_GRAC_leg(z,p);
    
    % Compute Controls
    tau = control_law(t,z,p);
    Fc=contact_force(z,p);
    
    taucl=tau(2);
    %tau(2)=ksea*(th2-thm);
    b = b_GRAC_leg(z,tau,Fc,p);
  
    QFc= [0 ; 0; Fc];
    
    
    %get thmdd    
    thmdd=((taucl-bm*thmd-ksea*(thm-th2)))/Im;
    %thmdd=0;
    
    % Solve for qdd.

    qdd = A\(b+QFc);
    dz = 0*z;
    
    % Form dz
    dz(1:3) = z(4:6);
    dz(4:6) = qdd;
    dz(7:8)=[thmd; thmdd];
    dz(9:10) = tau;
end


function animateSol(sol,p)
    % Prepare plot handles
    hold on
    h_GCOM = plot([0],[0],'LineWidth',2);
    h_COM_H = plot([0],[0],'LineWidth',2);
    h_HK = plot([0],[0],'LineWidth',2);
    h_KA = plot([0],[0],'LineWidth',2);
   h_AE = plot([0],[0],'LineWidth',2);
    
   h_tm = plot([0],[0],'LineWidth',2);
   thh=p(2);
    xlabel('x'); ylabel('y');
    h_title = title('t=0.0s');
    
    axis equal
    axis([-.2 .2 -.05 .6]);

    %Step through and update animation
    for t = 0:.01:sol.x(end)
            th1_vals=sol.y(2,:);
            th2_vals=sol.y(3,:);
            thm_vals=sol.y(7,:);
        % interpolate to get state at current time.
        z = interp1(sol.x',sol.y',t)';
        %keypoints = [rcm1 rh0 rk ra re];
        keypoints = keypoints_GRAC_leg(z,p);

        r_cm1 = keypoints(:,1); % Vector to base of cart
        r_h0 = keypoints(:,2);
        r_k = keypoints(:,3); % Vector to tip of pendulum
        r_a = keypoints(:,4);
        r_e = keypoints(:,5);

        set(h_title,'String',  sprintf('t=%.2f',t) ); % update title
        
        %set(h_GCOM,'XData',[0 r_cm1(1)]);
        %set(h_GCOM,'YData',[0 r_cm1(2)]);
        
        set(h_COM_H,'XData',[r_cm1(1) r_h0(1)]);
        set(h_COM_H,'YData',[r_cm1(2) r_h0(2)]);
        
        set(h_HK,'XData',[r_h0(1) r_k(1)]);
        set(h_HK,'YData',[r_h0(2) r_k(2)]);
        
        set(h_KA,'XData',[r_k(1) r_a(1)]);
        set(h_KA,'YData',[r_k(2) r_a(2)]);
        
        set(h_AE,'XData',[r_a(1) r_e(1)]);
        set(h_AE,'YData',[r_a(2) r_e(2)]);
        
        ihat = [1; 0; 0];
        jhat = [0; 1; 0];
        khat = cross(ihat,jhat);
        
       % ermhat =  sin(z(7)-z(2)+thh)*ihat + -cos(z(7)-z(2)+thh) * jhat;
        ermhat =  sin(0-z(2)+thh)*ihat + -cos(0-z(2)+thh) * jhat;
        r_m= r_a+.02*ermhat;
        %set(h_tm,'XData',[r_a(1) r_m(1)]);
        %set(h_tm,'YData',[r_a(2) r_m(2)]);
        
        pause(.1)
    end
    
    
end
    