function f = objective(x,z0,p)
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
tf=x(1);
%ctrl.tf = x(2);                                  % control time points
%ctrl.T = [x(2) x(3) x(4) x(5)]; 
ctrl.T = [x(2)];
ctrl.tf=x(1);
ctrl.dur = 0.3;


[t, kout,zout, sols, fc] = hybrid_simulation_GRAC(z0,ctrl,p,[0 tf]);
    COM = COM_GRAC_leg(z,p);
    y=COM(1,:);
    %f =-y(end) ;                                           % negative of COM height
    
    % alternate objective functions:
%     f = x(1);                                         % final time
%ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; 

%f=z(7,end);
f=y(end);
end