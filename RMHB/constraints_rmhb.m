function [cineq ceq] = constraints_rmhb(x,z0,p,tspan)
% Inputs:
% x - an array of decision variables.  [tf,  T0, T1, T2]
% z0 - the initial state
% p - simulation parameters
% p = [l; c1; c2; m1; m2; mh; I1; I2; g];
% Outputs:
% cineq - an array of values of nonlinear inequality constraint functions.  
%         The constraints are satisfied when these values are less than zero.
% ceq   - an array of values of nonlinear equality constraint functions.
%         The constraints are satisfied when these values are equal to zero.
%
% Note: fmincon() requires a handle to an constraint function that accepts 
% exactly one input, the decision variables 'x', and returns exactly two 
% outputs, the values of the inequality constraint functions 'cineq' and
% the values of the equality constraint functions 'ceq'. It is convenient 
% in this case to write an objective function which also accepts z0 and p 
% (because they will be needed to evaluate the objective function).  
% However, fmincon() will only pass in x; z0 and p will have to be
% provided using an anonymous function, just as we use anonymous
% functions with ode45().


                              
ctrl = [x(1) x(2) x(3) x(4)];

%[t, z, u, indices] = hybrid_simulation_GRAC(z0,ctrl,p,[0 tf]);
[sol]= simulate_leg_rmhb_GRAC(z0,ctrl,p,tspan);
%[t, kout,zout, sols, fc] = hybrid_simulation_GRAC(z0,ctrl,p,[0 tf]);
%t_landing=t(indices(1));
 z=sol.y;
theta=z(3,:);
COM = COM_GRAC_leg(z,p);
y=COM(1,:);

th_h=p(2);
th_k=z(2,:);
th_a=z(3,:);
sums=th_h-th_k+th_a;
   % cineq = [-min(sums), max(sums)-pi/2];
    %cineq=[-min(th_a), max(th_a)-pi/2];%[-min(theta) ,max(theta)-pi/2 ];                    
cineq=[];
 %   ceq = [ctrl.tf-t_takeoff , COM(4,end) ];                                            
                                                            
% simply comment out any alternate constraints when not in use
ceq=[];%[tf-land_time-ctrl.dur];
%ceq=[-ctrl.tf+t_landing];
   
end