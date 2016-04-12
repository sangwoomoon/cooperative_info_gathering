function o = ComputeH( o, AGENT, TARGET )
%COMPUTEH Summary of this function goes here
%   Detailed explanation goes here

H_11 = (TARGET.x(1)-AGENT.s(1))/sqrt((TARGET.x(1)-AGENT.s(1))^2+(TARGET.x(3)-AGENT.s(3))^2);
H_12 = (TARGET.x(3)-AGENT.s(3))/sqrt((TARGET.x(1)-AGENT.s(1))^2+(TARGET.x(3)-AGENT.s(3))^2);
H_21 = (AGENT.s(3)-TARGET.x(3))/((TARGET.x(1)-AGENT.s(1))^2+(TARGET.x(3)-AGENT.s(3))^2);
H_22 = (TARGET.x(1)-AGENT.s(1))/((TARGET.x(1)-AGENT.s(1))^2+(TARGET.x(3)-AGENT.s(3))^2);

o.H = blkdiag(o.H,...
        [H_11, 0, H_12, 0;
        H_21, 0 H_22, 0]);

end

