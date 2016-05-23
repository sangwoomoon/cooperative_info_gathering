function o = TimeUpdate(o, CLOCK, AGENT, sRandom)

sRandom;

% R = mvnrnd(MU,SIGMA,N) returns a N-by-D matrix R of random vectors
% chosen from the multivariate normal distribution with 1-by-D mean
% vector MU, and D-by-D covariance matrix SIGMA.

%%Simulate platform movements
o.v = (mvnrnd(zeros(1,2),o.Q,1))';
% actual bias would be exactly same
o.s(3:6) = o.F(3:6,3:6)*o.s(3:6) + o.Gamma(3:6,:)*o.v + o.Gu(3:6,:)*AGENT.CONTROL.u; 

% store current state to history
AGENT.hist.s(:,end+1) = o.s;

% stamp current time
AGENT.hist.stamp(length(AGENT.hist.s(1,:))) = CLOCK.ct;



end