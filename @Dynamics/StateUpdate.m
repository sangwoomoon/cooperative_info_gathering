function [ xNext, F_Next, GammaNext, GuNext ] = StateUpdate( obj, x_k, u_k, w_k, CLOCK, option )
%STATEUPDATE generates state with respect to delta_t in CLOCK. this
%function also uses the noise in Dynamics class as well as the current
%state as the initial state

if ~exist('option','var')
    % no option passed in by user
    option = 'default';
end

switch(option)
    case('default')
        % vector with initial conditions: state, state TM, process noise TM, control input TM
        switch(obj.spec)
            case('Linear')
                IC = [ x_k; reshape(eye(4),4*4,1); reshape(zeros(4),4*2,1); reshape(zeros(4,2),4*2,1)];
            case('Dubins')
                IC = [ x_k; reshape(eye(3),3*3,1); reshape(zeros(3),3*3,1); reshape(zeros(3,2),3*2,1)];
        end
    case('state')
        % vector with initial conditions: state
        IC = x_k;
end

% time vector
TimeProfile = 0:CLOCK.dt:CLOCK.dt;

% solution using ODE45
[~, xout] = ode45(@(t,x)obj.StateDerivative(t,x,u_k,w_k,option), TimeProfile, IC, obj.ODEoption);

switch(option)
    case('default')
        switch(obj.spec)
            case('Linear')
                xNext = xout(end,1:4)';
                F_Next = reshape(xout(end,5:20),4,4);
                GammaNext = reshape(xout(end,21:36),4,4);
                GuNext = reshape(xout(end,37:end),4,2);
            case('Dubins')
                xNext = xout(end,1:3)';
                F_Next = reshape(xout(end,4:12),3,3);
                GammaNext = reshape(xout(end,13:21),3,3);
                GuNext = reshape(xout(end,22:end),3,2);
        end
    case('state')
        switch(obj.spec)
            case('Linear')
                xNext = xout(end,1:4)';
                F_Next = [];
                GammaNext = [];
                GuNext = [];
            case('Dubins')
                xNext = xout(end,1:3)';
                F_Next = [];
                GammaNext = [];
                GuNext = [];
        end

end