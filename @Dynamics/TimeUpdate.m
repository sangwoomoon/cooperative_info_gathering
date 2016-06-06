function obj = TimeUpdate( obj, u, CLOCK )
%TIMEUPDATE Summary of this function goes here
%   Detailed explanation goes here

% make random noise with respect to covariance matrix Q (set in Gaussian
% Noise, but it can be changed as Non-Gaussian noises)
obj.MakeNoise();

% time update using ODE45 function
TimeProfile = 0:CLOCK.dt:CLOCK.dt;
[~, xout] = ode45(@obj.StateDerivate, TimeProfile, obj.x, obj.ODEoption, u);

obj.x = xout(end,:)'; % should be transposed since xout is time profile

% store current state to history
obj.hist.x(:,end+1) = obj.x;

% stamp current time
obj.hist.stamp(length(obj.hist.x(1,:))) = CLOCK.ct;

end

