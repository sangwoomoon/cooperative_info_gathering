function o = TakeMeasurement(o,AGENT,TARGET,ENVIRONMENT,CLOCK,sRandom)

sRandom;

%%Simulate relative position measurements to target from platform
o.rt = mvnrnd(zeros(1,2),o.Rt,1);

% BEWARE OF COORDINATE (GLOBAL)
dele = TARGET.x(1);% - AGENT.s(1); %watch indices!! [b_e,b_n,e,e_dot,n,n_dot] @xt1truehist
deln = TARGET.x(3);% - AGENT.s(3); %watch indices!! [e,e_dot,n,n_dot]  @xp1truehist

if TARGET.bLandMark ~= 1
    o.y(1) = AGENT.DYNAMICS.s(1) + dele + o.rt(1); %relative easting + bias
    o.y(2) = AGENT.DYNAMICS.s(2) + deln + o.rt(2); %relative northing + bias
else
    o.y(1) = AGENT.DYNAMICS.s(1) + o.rt(1); % bias + noise
    o.y(2) = AGENT.DYNAMICS.s(2) + o.rt(2); % bias + noise
end

%%Store measurement to history
o.hist.y(:,end+1) = o.y;

% stamp current time
o.hist.stamp(length(o.hist.y(1,:))) = CLOCK.ct;

end