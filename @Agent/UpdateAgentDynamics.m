function o = UpdateAgentDynamics(o, CLOCK, sRandom)

sRandom;

% R = mvnrnd(MU,SIGMA,N) returns a N-by-D matrix R of random vectors
% chosen from the multivariate normal distribution with 1-by-D mean
% vector MU, and D-by-D covariance matrix SIGMA.

%%Simulate platform movements
o.vp = (mvnrnd(zeros(1,2),o.Qp,1))';
o.s = o.Fp*o.s + o.Gamp*o.vp + o.Gu*o.CONTROL.u;

% store current state to history
o.hist.s(:,end+1) = o.s;

% stamp current time
o.hist.stamp(length(o.hist.s(1,:))) = CLOCK.ct;



end