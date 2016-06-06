function dx = StateDerivate( obj, t, x, u )
%STATEDERIVATE in Linear Bias (sensor bias) Dynamics Model generates the 
%derivatives of states to be used in ODE45.m function. So it is only usable
%for agents.
%   States ::
%   It is composed by biases as well as position and velocity
%   x = [b_e, b_n, e, e_dot, n, n_dot]

    dx(1) = obj.v(1); % acceleration bias noise
    dx(2) = obj.v(2); % acceleration bias noise
    dx(3) =     x(4); 
    dx(4) = obj.v(3)+u(1); % acceleration input and its process noise
    dx(5) =     x(6);
    dx(6) = obj.v(4)+u(2); % acceleration input and its process noise

    dx = dx'; % transpose to make a column vector

end

