function dx = StateDerivate( obj, t, x, u, w )
%STATEDERIVATE in Linear Dynamics Model generates the derivatives of states
%
%   Modifications:
%   x is now an expanded column vector including the state, the state
%   transition matrix, the process noise transition matrix and the control
%   input transition matrix. The numerical integrator solves a set of
%   differential equations and gives either the full solution (with the
%   propagated aforementioned variables) or just the state depending on
%   what the user has set under options. (not yet implemented)
%   ======================================================================

    Amatrix =  [0 1 0 0;
                0 0 0 0;
                0 0 0 1;
                0 0 0 0];
            
    Dmatrix =  [0 0 0 0;
                0 1 0 0;
                0 0 0 0;
                0 0 0 1];
    
    % state
    dx(1) = x(2);
    dx(2) = u(1) + obj.v(1); % acceleration input and its noise (from process)
    dx(3) = x(4);
    dx(4) = u(2) + obj.v(2); % acceleration input and its noise (from process)
    
    % state transition matrix
    phi = reshape(x(5:20),4,4);
    phidot = Amatrix*phi;
    
    % process noise transition matrix
    Gamma = reshape(x(21:36),4,4);
    Gammadot = Amatrix*Gamma + Dmatrix;
    
    % control input transition matrix
    Gu = reshape(x(37:end),4,2);
    Gudot = Amatrix*Gu;
    
    % differential vector
    dx = [ dx(1); dx(2); dx(3); dx(4); reshape(phidot, 4*4, 1); reshape(Gammadot, 4*4, 1); reshape(Gudot, 4*2, 1)];

end

