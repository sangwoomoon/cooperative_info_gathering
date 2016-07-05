function o = UpdateAgentDynamics(o, CLOCK, sRandom)

sRandom;

% R = mvnrnd(MU,SIGMA,N) returns a N-by-D matrix R of random vectors
% chosen from the multivariate normal distribution with 1-by-D mean
% vector MU, and D-by-D covariance matrix SIGMA.

%%Simulate platform movements (for heading input only)
o.vp = (mvnrnd(zeros(1,1),o.Qp,1))';

% unicycle model!
o.s(3) = o.s(3) + o.CONTROL.u*CLOCK.dt + o.vp*CLOCK.dt;

o.s(1) = o.s(1) + o.speed*cos(o.s(3))*CLOCK.dt;
o.s(2) = o.s(2) + o.speed*sin(o.s(3))*CLOCK.dt;

% store current state to history
o.hist.s(:,end+1) = o.s;

% stamp current time
o.hist.stamp(length(o.hist.s(1,:))) = CLOCK.ct;



end