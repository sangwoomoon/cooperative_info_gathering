function [ xNext, F_Next, GammaNext, GuNext ] = StateUpdate( obj, x_k, u_k, w_k, CLOCK, option )
%STATEUPDATE generates state with respect to delta_t in CLOCK. this
%function also uses the noise in Dynamics class as well as the current
%state as the initial state

if ~exist('option','var')
    % no option passed in by user
    option = 'default';
end

x_length = length(x_k);
w_length = length(w_k);
u_length = length(u_k);

switch(option)
    case('default')
        % vector with initial conditions: state, state TM, process noise TM, control input TM
        IC = [ x_k;...
            reshape(eye(x_length),x_length*x_length,1);... % state transition matrix
            reshape(zeros(x_length,w_length),x_length*w_length,1);... % process noise transition matrix
            reshape(zeros(x_length,u_length),x_length*u_length,1)]; % input transition matrix
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
        
        xNext = xout(end,1:x_length)';
        F_Next = reshape(xout(end,x_length+1:x_length+x_length*x_length),x_length,x_length);
        GammaNext = reshape(xout(end,x_length+x_length*x_length+1:x_length+x_length*x_length+x_length*w_length),x_length,w_length);
        GuNext = reshape(xout(end,x_length+x_length*x_length+x_length*w_length+1:end),x_length,u_length);

    case('state')
        
        xNext = xout(end,1:x_length)';
        F_Next = [];
        GammaNext = [];
        GuNext = [];
        

end