function obj = StateUpdate( obj, u, CLOCK )
%STATEUPDATE generates external state can be accessible from the outside of
%Dynamics class, which has the acceleration input noise (currently it is
%set as Gaussian input noise)

% make random noise with respect to covariance matrix Q (set in Gaussian
% Noise, but it can be changed as Non-Gaussian noises)
obj.MakeNoise('Gaussian');

% time update using ODE45 function
TimeProfile = 0:CLOCK.dt:CLOCK.dt;
[~, xout] = ode45(@obj.StateDerivate, TimeProfile, obj.x, obj.ODEoption, u);

obj.x_e = xout(end,:)'; % should be transposed since xout is time profile

% store current state to history
obj.hist.x_e(:,end+1) = obj.x_e;

% stamp current time
obj.hist.stamp(length(obj.hist.x(1,:))) = CLOCK.ct;

end

