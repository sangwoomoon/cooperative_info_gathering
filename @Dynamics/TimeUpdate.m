function obj = TimeUpdate( obj, u, CLOCK )
%TIMEUPDATE calls the function for updating states and store updated date into current state and its history

% make random noise with respect to covariance matrix Q (set in Gaussian
% Noise, but it can be changed as Non-Gaussian noises)
w = obj.MakeNoise('Gaussian');

% call state update function for updating states
x_next = obj.StateUpdate(obj.x,u,w,CLOCK);

% store current state to history
obj.hist.x(:,end+1) = x_next;

% stamp current time
obj.hist.stamp(end+1) = CLOCK.ct;

obj.x = x_next;

end

