function o = TakeMeasurement(o,AGENT,TARGET,ENVIRONMENT,CLOCK,sRandom)

sRandom;

for iTarget = 1 : length(TARGET)
    %%Simulate relative position measurements to target from platform
    o.rt = mvnrnd(zeros(1,2),o.Rt{iTarget},1);
    dele = TARGET(iTarget).x(1) - AGENT.s(1);
    deln = TARGET(iTarget).x(2) - AGENT.s(3); %watch indices!! [e,e_dot,n,n_dot]@xp1truehist
    o.y(2*(iTarget-1)+1) = dele + o.rt(1); %relative easting
    o.y(2*iTarget) = deln + o.rt(2); %relative northing
end
%%Simulate absolute position measurements for platform
o.rp = (mvnrnd(zeros(2,1),o.Rp,1))';
o.y(2*iTarget+1:2*(iTarget+1)) = o.Hp*AGENT.s + o.rp;

%%Store measurement to history
o.hist.y(:,end+1) = o.y;

% stamp current time
o.hist.stamp(length(o.hist.y(1,:))) = CLOCK.ct;

end