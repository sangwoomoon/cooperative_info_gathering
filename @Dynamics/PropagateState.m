function obj = PropagateState( obj, u, CLOCK )
%TIMEUPDATE calls the function for updating states and store updated date into current state and its history

% make random noise with respect to covariance matrix Q (set in Gaussian
% Noise, but it can be changed as Non-Gaussian noises)
obj.MakeNoise('Gaussian');

% call state update function for updating states
x_next = obj.StateUpdate(obj.x,u,obj.w,CLOCK);

% update current state
obj.x = x_next;

% store current state to history
obj.hist.x(:,end+1) = x_next;

% store current position and velocity to history structure (for plotting)
obj.hist.pos(:,end+1) = obj.GetPosition();
obj.hist.vel(:,end+1) = obj.GetVelocity();

% stamp current time
obj.hist.stamp(end+1) = CLOCK.ct;

end

