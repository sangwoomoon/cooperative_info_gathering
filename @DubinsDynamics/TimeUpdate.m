function o = TimeUpdate(o, CLOCK, AGENT, sRandom)

sRandom;

% R = mvnrnd(MU,SIGMA,N) returns a N-by-D matrix R of random vectors
% chosen from the multivariate normal distribution with 1-by-D mean
% vector MU, and D-by-D covariance matrix SIGMA.

%%Simulate platform movements
o.v = (mvnrnd(zeros(1,3),o.Q,1))'; % noise for x, y and heading

o.s = double(subs(o.Eqn,...
    {o.s_sym(1),o.s_sym(2),o.s_sym(3),...
    AGENT.CONTROL.u_sym(1),AGENT.CONTROL.u_sym(2),...
    o.v_sym(1),o.v_sym(2),o.v_sym(3),...
    CLOCK.dt_sym},...
    {o.s(1),o.s(2),o.s(3),AGENT.CONTROL.u(1),AGENT.CONTROL.u(2),...
    o.v(1),o.v(2),o.v(3),CLOCK.dt}));

% store current state to history
o.hist.s(:,end+1) = o.s;

% stamp current time
o.hist.stamp(length(o.hist.s(1,:))) = CLOCK.ct;

end