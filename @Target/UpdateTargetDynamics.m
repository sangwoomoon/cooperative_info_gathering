function o = UpdateTargetDynamics(o, CLOCK, sRandom)

sRandom;


%%Simulate target movement with Euler

% vt = mvnrnd(MU,SIGMA,N) returns a N-by-D matrix R of random vectors
% chosen from the multivariate normal distribution with 1-by-D mean
% vector MU, and D-by-D covariance matrix SIGMA.
o.vt = mvnrnd(zeros(1,2),o.Qt,1);  % mvnrnd(MU(N-by-D Matrix),SIGMA(Gaussian Cov.),N)
o.x = o.Ft*o.x + o.Gt*o.vt';

% store current state to history 
o.hist.x(:,end+1) = o.x;

% stamp current time
o.hist.stamp(length(o.hist.x(1,:))) = CLOCK.ct;


end