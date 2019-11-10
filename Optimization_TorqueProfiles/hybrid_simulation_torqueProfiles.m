function [tout, zout, uout, indices, sols, land_time, Fout] = hybrid_simulation(z0,ctrl,p,tspan)
path = 'C:\Users\Usuario\Documents\MATLAB\Bio-inspired Robotics\Final Project Landing Robot\Bio-inspired-Robotics-Robot-Leg';
addpath([path '\AutoDerived'])
%Inputs:
% z0 - the initial state
% ctrl- control structure
% p - simulation parameters
% tspan - [t0 tf] where t0 is start time, tf is end time
% 
% Outputs:
% tout - vector of all time points
% zout - vector of all state trajectories
% uout - vector of all control trajectories
% indicies - vector of indices indicating when each phases ended
%   for instance, tout( indices(1) ) provides the time at which
%   the first phase (stance) ended
% sols - vector of solution structures
%

    t0 = tspan(1); tend = tspan(end);   % set initial and final times

    %% Setup tolerance Options
    inttol = 1e-6;  % set integration tolerances
    iphase = 2;     % phase number is currently phase 2, falling
    sols = [];      % initialize array of solution structures

    while(t0 < tend)
        
        % now include the number of the phase in the call to the event
        % function (1== stance, 2 == flight)

        opts = odeset('Events', @(t,z) event_conditions(t,z,ctrl,p,iphase), 'abstol',inttol,'reltol',inttol);
        f = @(t,z) dynamics_continuous(t, z, ctrl,p, iphase);    % we also include the number of the phase in the call to the EoM function
        sol = ode45(f, [t0 tend], z0, opts);    % integate until an event happens or time runs out
        sol.iphase = iphase;                    % store the phase number in the solution structure
        sol.footpos=position_foot(sol.y(:,end),p);
        t0 = sol.x(end);                        % reset the integration initial time
        
       
        if isfield(sol,'ie') && ~isempty(sol.ie)
            z0 = sol.ye(:,end);                         
            if any(sol.ie == 1)                         %
                iphase = 1;
                land_time = sol.x(end);
                ctrl.land_time = land_time;
            %elseif any(sol.ie == 2)                     % 
            %    iphase = 2;                             % 
            end
        else
            sol.ie = []; sol.xe = []; sol.ye = [];      % leave this just in case no event occured
        end
        sols = [sols; sol];                             % append solution structure to array
    end

    % assemble the results from each solution structure in the array
    tout = []; zout = []; uout = []; indices = []; Fout=[];
    for ii = 1:length(sols)
        sol = sols(ii);                                     % get the next solution structure
        iphase = sol.iphase;                                % get the phase number
        sol.x(1) = sol.x(1)+1e-6;                           % the time starts just a little after the last time ended (required for animation)
        tout = [tout sol.x];                                % append time points
        zout = [zout sol.y];                                % append states
        uout = [uout control_laws(sol.x,sol.y,ctrl,p,iphase)];   % append controls
        Fout = [Fout contact_force(sol.x,sol.y,ctrl,p,iphase)];   % append contact force
        indices = [indices length(tout)];                   % append indices at which phases end
    end

end

%% Continuous dynamics
function [dz, Fc] = dynamics_continuous(t,z,ctrl,p,iphase)

    u = control_laws(t,z,ctrl,p,iphase);  % get controls at this instant

   
    % Contact model
    %if iphase == 2  % in flight phase
       %Fc = 0;
                % in stance phase
       C = position_foot(z,p); %y
       if C(2) <= 0  
        dC= velocity_foot(z,p); %dy
        Fc = -5000 * C(2) - 30*dC(2);
        if Fc < 0
            Fc = 0;
        end
       else
        Fc = 0;
       end
        %Fc
    
    A = A_GRAC_leg(z,p);                 % get full A matrix
    b = b_GRAC_leg(z,u,Fc,p);               % get full b vector
    
    x = A\b;                % solve system for accelerations (and possibly forces)
    dz(1:3,1) = z(4:6); % assign velocities to time derivative of state vector
    dz(4:6,1) = x(1:3);   % assign accelerations to time derivative of state vector
    

    dz(7) = sum(u.^2); %1              % change to integrate torque squared
end

%% Control
function u = control_laws(t,z,ctrl,p,iphase)

    if iphase == 1
        t_ctrl = t - ctrl.land_time;
        ctrl.t = linspace(0,ctrl.dur,length(ctrl.T(1:3)));
        u1 = interp1(ctrl.t,ctrl.T(1:3),t_ctrl,'linear','extrap');
        u2 = interp1(ctrl.t,ctrl.T(4:6),t_ctrl,'linear','extrap');
        u=[u1;u2];
        
    else
        %falling
        % PD Control in flight
        thk = z(2,:);            % leg angle
        dthk = z(5,:);           % leg angular velocity
        tha = z(3,:);            % leg angle
        dtha = z(6,:);           % leg angular velocity

        thkd = deg2rad(90);             % desired leg angle
        thad = deg2rad(75);             % desired leg angle
        k = 10;                  % stiffness (N/rad)
        b = .5;                  % damping (N/(rad/s))

        u1 = -k*(thk-thkd) - b*dthk;% apply PD control
        u2 = -k*(tha-thad) - b*dtha;
        u=[u1;u2];
    end

end


function Fct = contact_force(t,z,ctrl,p,iphase)
Fct=[];
        for i=1:length(z(1,:))
           C = position_foot(z(:,i),p); %y
           if C(2) <= 0  
            dC= velocity_foot(z(:,i),p); %dy
            Fc = -5000 * C(2) - 30*dC(2);
            if Fc < 0
                Fc = 0;
            end
           else
            Fc = 0;
           end
            
           Fct=[Fct Fc];
        end
                % change to integrate torque squared
end


%% Events
function [value,isterminal,direction] = event_conditions(t,z,ctrl,p,iphase)

    if iphase == 2                      % in stance phase
        
         %[x, Fc] = dynamics_continuous(t,z,ctrl,p,iphase);
    
        C = position_foot(z,p);
        value(1) = C(2);
        isterminal(1)=1;
        direction(1)=-1;
    else
        value(1)=1;
        isterminal(1)=0;
        direction(1)=1;
        
   
%    elseif iphase == 1                      % in jumping phase
%         [x, Fc] = dynamics_continuous(t,z,ctrl,p,iphase);
%         C = position_foot(z,p);
%         value(2) = C(2);
%         isterminal(2)=1;
%         direction(2)=1;
        
%         [x, Fc] = dynamics_continuous(t,z,ctrl,p,iphase);
%         value(2) = Fc;              
%         isterminal(2) = 1;              % terminate integration when ground reaction force is zero
%         direction(2) = -1;              % if it's decreasing
%        
    end
    
end
