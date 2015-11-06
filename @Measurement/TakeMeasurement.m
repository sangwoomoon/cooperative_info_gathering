function o = TakeMeasurement(o,AGENT,TARGET,ENVIRONMENT,CLOCK,sRandom)

sRandom;

for iTarget = 1 : length(TARGET)
    %%Simulate relative position measurements to target from platform
    o.rt = mvnrnd(zeros(1,2),o.Rt{iTarget},1); 
    
    % BEWARE OF COORDINATE (GLOABAL)
    dele = TARGET(iTarget).x(5);% - AGENT.s(1); %watch indices!! [b_e,b_n,e,e_dot,n,n_dot] @xt1truehist
    deln = TARGET(iTarget).x(7);% - AGENT.s(3); %watch indices!! [e,e_dot,n,n_dot]  @xp1truehist
    o.y(2*(iTarget-1)+1) = TARGET(iTarget).x(2*(iTarget-1)+1) + dele + o.rt(1); %relative easting + bias
    o.y(2*iTarget) = TARGET(iTarget).x(2*(iTarget-1)+2) + deln + o.rt(2); %relative northing + bias
end

%%Store measurement to history
o.hist.y(:,end+1) = o.y;

% stamp current time
o.hist.stamp(length(o.hist.y(1,:))) = CLOCK.ct;

end