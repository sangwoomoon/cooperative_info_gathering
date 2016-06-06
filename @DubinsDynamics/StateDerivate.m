function dx = StateDerivate( obj, t, x, u )
%STATEDERIVATE in Dubins Dynamics Model generates the 
%derivatives of states to be used in ODE45.m function.
%   States ::
%   It is composed by position and heading
%   x = [e, n, theta]

    dx(1) = u(1)*cos(x(3)) + obj.v(1); % cosine input plus its noise
    dx(2) = u(1)*sin(x(3)) + obj.v(2); % cosine input plus its noise
    dx(3) = u(2) + obj.v(3);               % heading input plus its noise
    
    dx = dx'; % transpose to make a column vector

end

