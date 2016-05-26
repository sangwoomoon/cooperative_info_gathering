function o = TimeUpdate(o, CLOCK, AGENT)


% R = mvnrnd(MU,SIGMA,N) returns a N-by-D matrix R of random vectors
% chosen from the multivariate normal distribution with 1-by-D mean
% vector MU, and D-by-D covariance matrix SIGMA.

%%Simulate platform movements
o.v = (mvnrnd(zeros(1,4),o.Q,1))'; % noise for bias and inputs

% actual bias would be exactly same
o.x = double(subs(o.Eqn,...
    {o.x_sym(1),o.x_sym(2),o.x_sym(3),o.x_sym(4),o.x_sym(5),o.x_sym(6)...
    AGENT.CONTROL.u_sym(1),AGENT.CONTROL.u_sym(2),...
    o.v_sym(1),o.v_sym(2),o.v_sym(3),o.v_sym(4),...
    CLOCK.dt_sym},...
    {o.x(1),o.x(2),o.x(3),o.x(4),o.x(5),o.x(6),AGENT.CONTROL.u(1),AGENT.CONTROL.u(2),...
    o.v(1),o.v(2),o.v(3),o.v(4),CLOCK.dt}));

% store current state to history
o.hist.x(:,end+1) = o.x;

% stamp current time
o.hist.stamp(length(o.hist.x(1,:))) = CLOCK.ct;



end