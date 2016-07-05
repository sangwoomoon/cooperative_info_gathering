function o = ComputeBeta( o, AGENT, SIMULATION, id )
%COMPUTEBETA Summary of this function goes here
%   Detailed explanation goes here

for iAgent = 1 : SIMULATION.nAgent
    
    % compute distance(range) each other ( with respect to AGENT(id) )
    dist = ComputeDistance(AGENT(id).s,AGENT(iAgent).s);
    
    % beta is from the empirical data gathered in experiments between 2
    % UAVs
    o.beta(iAgent) = 1/(1+exp(0.01*(dist-650)))+0.0015;
    
end

end

function dist = ComputeDistance(s1, s2)
    dist = sqrt((s1(1)-s2(1))^2+(s1(3)-s2(3))^2);
end


