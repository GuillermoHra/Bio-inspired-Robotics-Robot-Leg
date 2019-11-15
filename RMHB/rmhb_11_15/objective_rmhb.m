function f = objective(x,z0,p,tspan)
% Inputs:
% x - an array of decision variables.
% z0 - the initial state
% p - simulation parameters
% 
% Outputs:
% f - scalar value of the function (to be minimized) evaluated for the
%     provided values of the decision variables.
%
% Note: fmincon() requires a handle to an objective function that accepts 
% exactly one input, the decision variables 'x', and returns exactly one 
% output, the objective function value 'f'.  It is convenient for this 
% assignment to write an objective function which also accepts z0 and p 
% (because they will be needed to evaluate the objective function).  
% However, fmincon() will only pass in x; z0 and p will have to be
% provided using an anonymous function, just as we use anonymous
% functions with ode45().

%how do we get z?
%x=[tf, tfctrl, T1, T2]

%ctrl.tf = x(2);                                  % control time points
ctrl = [x(1) x(2) x(3) x(4)]; 


 [ sol,uout]=simulate_leg_rmhb_GRAC(z0,ctrl,p,tspan);

    %[t, kout,zout, sols, fc] = hybrid_simulation_GRAC(z0,ctrl,p,[0 tf]);
    z=sol.y;
    COM = COM_GRAC_leg(z,p);
    ycom=COM(1,:);

f=max(abs(diff(diff(diff(ycom)))));
end