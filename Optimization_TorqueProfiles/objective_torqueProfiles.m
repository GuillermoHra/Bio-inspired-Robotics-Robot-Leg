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

%path = 'C:\Users\Usuario\Documents\MATLAB\Bio-inspired Robotics\Final Project Landing Robot\Bio-inspired-Robotics-Robot-Leg';
%addpath([path '\AutoDerived'])

%how do we get z?
%x=[tf, tfctrl, T1, T2]
tf=x(1);
%ctrl.tf = x(2);                                  % control time points
ctrl.T = [x(2) x(3) x(4) x(5) x(6) x(6)]; 
ctrl.dur = 0.3;
ctrl.land_time = 0;

    [t, z, u, indices, sols, land_time, fc] = hybrid_simulation_torqueProfiles(z0,ctrl,p,[0 tf]);
    COM = COM_GRAC_leg(z,p);
    y = COM(1,:);
    
    foot_pos = position_foot(z, p);
    y_foot = foot_pos(2,:);
    
    % alternate objective functions:
    %f = x(1);                         
    %ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; 

%f=z(7,end);
f = -y(end);  % negative of COM height
%f = y_foot(end); % foot y position
% maximize u2
%f = -z(7,end);
%f = max(abs(diff(diff(diff(y)))));
% try also error of impedance controller
end