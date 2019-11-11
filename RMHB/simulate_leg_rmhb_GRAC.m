function [ sol] = simulate_leg_rmhb_GRAC(z0,ctrl,p,tspan)

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
    thki=p(20);
    thai=p(21);
    %% Parameter vector make sure this matches Derive_everything_GRAC.m
    p=[thh0;thh; m1; I1; m2 ;I2; m3 ;I3; m4; I4; l1; l2; l3 ;l4;lc2;lc3;lc4;lh0; g ;thki;thai]; %parameters
%p=[thh0;thh; m1; I1; m2 ;I2; m3 ;I3; m4; I4; l1; l2; l3 ;l4;lc2;lc3;lc4;lh0; g ]; %parameters
     %% Perform Dynamic simulation
    

    %z0 = [.5; thki;thai; 0; 0;0]; %y, theta knee, theta ankle
    opts = odeset('AbsTol',1e-8,'RelTol',1e-6);

    sol = ode45(@dynamics,tspan,z0,opts,p,z0,ctrl);

    sol.k=ctrl;
    y_vals=sol.y(1,:);
    th1_vals=sol.y(2,:);
    th2_vals=sol.y(3,:);
   
    
end

function tau = control_law(t,z,p,z0,ctrl)
%     % Controller gains, Update as necessary for Problem 1

        
     %desired angles of leg
         thki=p(20);   %MAKE SURE THESE MATCH AT THE TOP
    thai=p(21);
    
      thkd=thki;  %theta knee desired
      thad=thai;    %theta ankle desired
      thkc=z(2); %theta knee current
      thac=z(3); %theta ankle current
      thkvc=z(5);%theta knee velocity current
      thavc=z(6);%theta ankle velocity current
      %size(z)

      kk=ctrl(1);
      bk=ctrl(2);
      ka=ctrl(3);
      ba=ctrl(4);
      tau = [-(kk*(thkc-thkd)+ .2*(thkvc)) ; -(50*(thac-thad)+ 5*(thavc))  ]; %WATCH FOR OVERDAMPING
      KSEA=100;
      tau(2)=tau(2) + KSEA*(z0(3)-z(3));
      % tau = [0; -(500*(thac-thad)+ 5*(thavc))  ];
   
         %tau=[0;0];
   
   

   end

function Fc = contact_force(z,p,z0)

    % Fixed parameters for contact
    K_c1 = 1000;
    D_c1 = 20;
    dC1  = deg2rad(0);
    
    KSEA=300;

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

function dz = dynamics(t,z,p,z0,ctrl)
    y = z(1);     th1 = z(2);     th2 = z(3);
   yd= z(4);     th1d= z(5);     th2d = z(6);
    
    
  
    
    %----------------------------
    %WHAT WE NEED IS TO FIGURE OUT HOW TO SIMULATE THM vs TH2 (they are
    %different) 
    %----------------------------------------
    
    % Get mass matrix
    A = A_GRAC_leg(z,p);
    
    % Compute Controls
    tau = control_law(t,z,p,z0,ctrl);
    Fc=contact_force(z,p,z0);
    

    b = b_GRAC_leg(z,tau,Fc,p);
  
    QFc= [0 ; 0; Fc];
    
    

    
    % Solve for qdd.

    qdd = A\(b+QFc);
    dz = 0*z;
    
    % Form dz
    dz(1:3) = z(4:6);
    dz(4:6) = qdd;
    
end


   