function jacobian_next = JacobianUpdate( obj, x_k, u_k, w_k, jacobian_k, CLOCK, option )
%JACOBIANUPDATE Summary of this function goes here
%   Detailed explanation goes here


%       OBSOLETE FUNCTION


% options
options = obj.ODEoption;
options.jacobian = option; % where 'option' is either 'state', 'w' or 'u'

% time update using ODE45 function
TimeProfile = 0:CLOCK.dt:CLOCK.dt;
[~, jacobianout] = ode45(@(t,jacobian)obj.TakeJacobian(t,jacobian,x_k,u_k,w_k), TimeProfile, jacobian_k, options);

switch(option.jacobian)
    case('F')
        jacobian_next = reshape(jacobianout(end,:),length(x_k),length(x_k));
    case('Gamma')
        jacobian_next = reshape(jacobianout(end,:),length(x_k),length(x_k));
    case('Gu')
        jacobian_next = reshape(jacobianout(end,:),length(x_k),length(u_k));

end

