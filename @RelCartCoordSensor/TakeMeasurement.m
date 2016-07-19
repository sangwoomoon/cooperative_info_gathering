function obj = TakeMeasurement(obj,s_k,TARGET,CLOCK)


%%Simulate relative position measurements to target from platform
obj.rt = mvnrnd(zeros(1,2),obj.Rt,1);

% BEWARE OF COORDINATE (GLOBAL)
dele = TARGET.x(1);% - AGENT.s(1); %watch indices!! [b_e,b_n,e,e_dot,n,n_dot] @xt1truehist
deln = TARGET.x(3);% - AGENT.s(3); %watch indices!! [e,e_dot,n,n_dot]  @xp1truehist

if TARGET.bLandMark ~= 1
    obj.y(1) = s_k(1) + dele + obj.rt(1); %relative easting + bias
    obj.y(2) = s_k(2) + deln + obj.rt(2); %relative northing + bias
else
    obj.y(1) = obj.DYNAMICS.s(1) + obj.rt(1); % bias + noise
    obj.y(2) = obj.DYNAMICS.s(2) + obj.rt(2); % bias + noise
end

%%Store measurement to history
obj.hist.y(:,end+1) = obj.y;

% stamp current time
obj.hist.stamp(length(obj.hist.y(1,:))) = CLOCK.ct;

end