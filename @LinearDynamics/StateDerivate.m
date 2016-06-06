function dx = StateDerivate( obj, t, x, u )
%STATEDERIVATE in Linear Dynamics Model generates the derivatives of states
%   States ::
%   It is composed by position and velocity
%   x = [e, e_dot, n, n_dot]

    dx(1) = x(2);
    dx(2) = u(1) + obj.v(1); % acceleration input and its noise (from process)
    dx(3) = x(4);
    dx(4) = u(2) + obj.v(2); % acceleration input and its noise (from process)
    
    dx = dx'; % transpose to make a column vector

end

