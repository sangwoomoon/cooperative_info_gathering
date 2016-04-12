function G = TakeMeasurement( o, Xbar, AGENT )
%TAKEMEASUREMENT Summary of this function goes here
%   Detailed explanation goes here

G = [sqrt((Xbar(1)-AGENT.s(1))^2+(Xbar(3)-AGENT.s(3))^2);
    atan2(Xbar(3)-AGENT.s(3),Xbar(1)-AGENT.s(1))];

end

