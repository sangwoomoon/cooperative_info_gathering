function o = UpdateAgentDynamics(o, CLOCK, sRandom)

sRandom;

% R = mvnrnd(MU,SIGMA,N) returns a N-by-D matrix R of random vectors
% chosen from the multivariate normal distribution with 1-by-D mean
% vector MU, and D-by-D covariance matrix SIGMA.

%%Simulate platform movements
o.vp = (mvnrnd(zeros(1,2),o.Qp,1))';
% actual bias would be exactly same
o.s(3:6) = o.Fp(3:6,3:6)*o.s(3:6) + o.Gamp(3:6,:)*o.vp + o.Gu(3:6,:)*o.CONTROL.u; 

% store current state to history
o.hist.s(:,end+1) = o.s;

% stamp current time
o.hist.stamp(length(o.hist.s(1,:))) = CLOCK.ct;



end