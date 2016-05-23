function o = TimeUpdate(o, CLOCK, AGENT, sRandom)

sRandom;

% R = mvnrnd(MU,SIGMA,N) returns a N-by-D matrix R of random vectors
% chosen from the multivariate normal distribution with 1-by-D mean
% vector MU, and D-by-D covariance matrix SIGMA.

%%Simulate platform movements
o.v = (mvnrnd(zeros(1,3),o.Q,1))'; % noise for heading, x, and y

o.s(3) = o.s(3) + AGENT.CONTROL.u(2)*CLOCK.dt + o.v(3);

o.s(1) = o.s(1) + AGENT.CONTROL.u(1)*cos(o.s(3))*CLOCK.dt + o.v(1);
o.s(2) = o.s(2) + AGENT.CONTROL.u(1)*sin(o.s(3))*CLOCK.dt + o.v(2);

% store current state to history
AGENT.hist.s(:,end+1) = o.s;

% stamp current time
AGENT.hist.stamp(length(AGENT.hist.s(1,:))) = CLOCK.ct;

end