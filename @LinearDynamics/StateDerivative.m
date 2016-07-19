function dx = StateDerivative( obj, t, x, u, w, option )
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

% state
dx(1) = x(2);
dx(2) = u(1) + obj.w(1); % acceleration input and its noise (from process)
dx(3) = x(4);
dx(4) = u(2) + obj.w(2); % acceleration input and its noise (from process)

switch(option)
    
    case('default')
        
        Amatrix = [0 1 0 0;
                   0 0 0 0;
                   0 0 0 1;
                   0 0 0 0];
        
        Dmatrix = [0 0;
                   1 0;
                   0 0;
                   0 1];
                       
        x_length = length(obj.x);
        w_length = length(w);
        u_length = length(u);

        % state transition matrix
        phi = reshape(x(x_length+1:x_length+x_length*x_length),x_length,x_length);
        phidot = Amatrix*phi;

        % process noise transition matrix
        Gamma = reshape(x(x_length+x_length*x_length+1:x_length+x_length*x_length+x_length*w_length),x_length,w_length);
        Gammadot = Amatrix*Gamma + Dmatrix;

        % control input transition matrix
        Gu = reshape(x(x_length+x_length*x_length+x_length*w_length+1:end),x_length,u_length);
        Gudot = Amatrix*Gu;

        dx = [ dx'; reshape(phidot, x_length*x_length, 1); reshape(Gammadot, x_length*w_length, 1); reshape(Gudot, x_length*u_length, 1)];
        
    case('state')

        dx = dx';
end

end

