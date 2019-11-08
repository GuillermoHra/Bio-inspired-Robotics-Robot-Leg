function derive_everything_GRAC()
name='GRAC_leg';

syms t y dy ddy thh0 thh thk  tha dthk dtha ddthk ddtha tauk taua Fy m1 I1 m2 I2 m3 I3 m4 I4 l1 l2 l3 l4 lc2 lc3 lc4 lh0 g real

q= [y;thk; tha];  %generalized coordinates, theta knee, theta ankle
dq=[dy;dthk; dtha]; %first time derivatives
ddq=[ddy;ddthk; ddtha]; %second time derivatives
u=[tauk;taua];  %control torques for knee and ankle
Fc=Fy;           %constraint forces and moments
p=[thh0;thh; m1; I1; m2 ;I2; m3 ;I3; m4; I4; l1; l2; l3 ;l4;lc2;lc3;lc4;lh0; g ]; %parameters
    
%%% Calculate important vectors and their time derivatives.

% Define fundamental unit vectors.  The first element should be the
% horizontal (+x cartesian) component, the second should be the vertical (+y
% cartesian) component, and the third should be right-handed orthogonal.
ihat = [1; 0; 0];
jhat = [0; 1; 0];
khat = cross(ihat,jhat);


% Define other unit vectors for use in defining other vectors.
erh0hat =  sin(thh0)*ihat - cos(thh0) * jhat;
erhhat =  sin(thh)*ihat + -cos(thh) * jhat;
erkhat =  sin(-thk +thh)*ihat + -cos(-thk+thh) * jhat ;
erahat =  sin(tha-thk+thh)*ihat + -cos(tha-thk+thh) * jhat;

% A handy anonymous function for taking first and second time derivatives
% of vectors using the chain rule.  See Lecture 6 for more information. 
ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; 

% Define vectors to key points.
rcm1 = y*jhat;  %COM robot to ground
rh0= rcm1 + lh0*erh0hat;  %COM to hip joint, fixed
rk = rh0+l2*erhhat;  %Hip joint to knee, fixed  to point K
rcm2 = rh0+lc2*erhhat;  %Hip joint to COM of thigh, fixed  to point m2,I2
ra=rk+ l3*erkhat;
rcm3=rk+lc3*erkhat;
re= ra+l4*erahat;
rcm4=ra+lc4*erahat;

keypoints = [rcm1 rh0 rk ra re];

% Take time derivatives of vectors as required for kinetic energy terms.
drcm1 = ddt(rcm1);
drcm2 = ddt(rcm2);
drcm3 = ddt(rcm3);
drcm4 = ddt(rcm4);
dre   = ddt(re);  %height off ground is center of mass of foot to account for different landing configs- can change later

%%% Calculate Kinetic Energy, Potential Energy, and Generalized Forces

% F2Q calculates the contribution of a force to all generalized forces
% for forces, F is the force vector and r is the position vector of the 
% point of force application
F2Q = @(F,r) simplify(jacobian(r,q)'*(F)); 

% M2Q calculates the contribution of a moment to all generalized forces
% M is the moment vector and w is the angular velocity vector of the
% body on which the moment acts
M2Q = @(M,w) simplify(jacobian(w,dq)'*(M)); 


% Define kinetic energies. See Lecture 6 formula for kinetic energy
% of a rigid body.
T1 = (1/2)*m1*dot(drcm1, drcm1);  %COM of robot, no rotation
T2 = (1/2)*m2*dot(drcm2, drcm2); % COM of Thigh, angle fixed
T3 = (1/2)*m3*dot(drcm3, drcm3)+ (1/2)* I3 * (dthk)^2;  %KE of calf from knee
T4 = (1/2)*m4*dot(drcm4, drcm4)+ (1/2)* I4* (dtha)^2; %KE of foot from ankle


% Define potential energies. See Lecture 6 formulas for gravitational 
% potential energy of rigid bodies and elastic potential energies of
% energy storage elements.
V1 = m1*g*dot(rcm1, jhat);
V2 = m2*g*dot(rcm2, jhat);
V3 = m3*g*dot(rcm3, jhat);
V4 = m4*g*dot(rcm4, jhat);

% Define contributions to generalized forces.  See Lecture 6 formulas for
% contributions to generalized forces.
QF = F2Q(Fy*jhat,re);
Qtauk = M2Q(tauk*khat, dthk*khat);
Qtaua = M2Q(taua*khat, dtha*khat);  %need to work out the signs

T = T1 + T2 + T3 + T4;
V = V1 + V2 + V3 + V4;
Q = QF + Qtauk + Qtaua;

% Calculate rcm, the location of the center of mass
%rcm = (m1*rcm1 + m2*rcm2 + mh*rh)/(m1+m2+mh); rcm for us is the COM of the
%torso

C=y;
dC=ddt(C);



%% All the work is done!  Just turn the crank...
%%% Derive Energy Function and Equations of Motion
E = T+V;                                         % total system energy
L = T-V;                                         % the Lagrangian
eom = ddt(jacobian(L,dq)') - jacobian(L,q)' - Q;  % form the dynamics equations

size(eom)

%%% Rearrange Equations of Motion. 
A = jacobian(eom,ddq);
b = A*ddq - eom;


%%% Write functions to evaluate dynamics, etc...
z = sym(zeros(length([q;dq]),1)); % initialize the state vector
z(1:3,1) = q;  
z(4:6,1) = dq;
Jleg = jacobian(re,q);
% Write functions to a separate folder because we don't usually have to see them
directory = 'AutoDerived/';
% Write a function to evaluate the energy of the system given the current state and parameters
matlabFunction(E,'file',[directory 'energy_' name],'vars',{z p});
% Write a function to evaluate the A matrix of the system given the current state and parameters
matlabFunction(A,'file',[directory 'A_' name],'vars',{z p});
% Write a function to evaluate the b vector of the system given the current state, current control, and parameters
matlabFunction(b,'file',[directory 'b_' name],'vars',{z u Fc p});

matlabFunction(keypoints,'file',[directory 'keypoints_' name],'vars',{z p});

matlabFunction(C,'file',[directory 'C_' name],'vars',{z u p});
matlabFunction(dC,'file',[directory 'dC_' name],'vars',{z u p});
matlabFunction(re,'file',[directory 'position_foot'],'vars',{z p});
matlabFunction(dre,'file',[directory 'velocity_foot'],'vars',{z p});
% Write a function to evaluate the X and Y coordinates and speeds of the center of mass given the current state and parameters
drcm =drcm1 ;% ddt(rcm);  % Calculate center of mass velocity vector
COM = [rcm1(1); drcm1(1)]; % Concatenate  y coordinate and speeds of center of mass in array
matlabFunction(COM,'file',[directory 'COM_' name],'vars',{z p});
matlabFunction(Jleg,'file',[directory 'jacobian_foot'],'vars',{z p});


