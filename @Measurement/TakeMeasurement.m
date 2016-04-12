function o = TakeMeasurement(o,AGENT,TARGET,ENVIRONMENT,CLOCK,sRandom)

sRandom;

% Simulate relative position measurements to target from platform
o.rt = mvnrnd(zeros(1,2),o.Rt,1);

% range-bearing measurement 
o.y =[sqrt((TARGET.x(1)-AGENT.s(1))^2+(TARGET.x(3)-AGENT.s(3))^2);
        atan2(TARGET.x(3)-AGENT.s(3),TARGET.x(1)-AGENT.s(1))] + o.rt';

%%Store measurement to history
o.hist.y(:,end+1) = o.y;

% stamp current time
o.hist.stamp(length(o.hist.y(1,:))) = CLOCK.ct;

end