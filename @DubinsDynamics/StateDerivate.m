function dx = StateDerivate( obj, t, x, u, w )
%STATEDERIVATE in Dubins Dynamics Model generates the 
%derivatives of states to be used in ODE45.m function.
%
%   Modifications:
%   x is now an expanded column vector including the state, the state
%   transition matrix, the process noise transition matrix and the control
%   input transition matrix. The numerical integrator solves a set of
%   differential equations and gives either the full solution (with the
%   propagated aforementioned variables) or just the state depending on
%   what the user has set under options. (not yet implemented)
%   ======================================================================
    
    Amatrix = [ 0 0 -u(1)*cos(x(3));
                0 0  u(2)*cos(x(3));
                0 0        0       ];
    Dmatrix = eye(3);

    % state
    dx(1) = u(1)*cos(x(3)) + obj.v(1); % cosine input plus its noise
    dx(2) = u(1)*sin(x(3)) + obj.v(2); % cosine input plus its noise
    dx(3) = u(2) + obj.v(3);           % heading input plus its noise
            
    % state transition matrix
    phi = reshape(x(4:12),3,3);
    phidot = Amatrix*phi;
    
    % process noise transition matrix
    Gamma = reshape(x(13:21),3,3);
    Gammadot = Amatrix*Gamma + Dmatrix;
    
    % control input transition matrix
    Gu = reshape(x(22:27),3,2);
    Gudot = Amatrix*Gu;
    
    % differential vector
    dx = [ dx(1); dx(2); dx(3); reshape(phidot, 3*3, 1); reshape(Gammadot, 3*3, 1); reshape(Gudot, 3*2, 1)];

end

