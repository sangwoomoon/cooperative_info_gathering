% Compute probability of delivery with respect to relative distance and
% communication characteristics
%
% equation is based on a reference, "Stachura, Maciej and Frew, Eric, Communication-Aware
% Information-Gathering Experiments with an Unmanned Aircraft System."
%
% beta = 0.5 X erfc[(dist-461)/195]-0.085
%
% assume the own measurement information is perfectly delivered.
function beta = ComputeCommProb(iAgentPos,jAgentPos)
    agentDist = ComputeDistance(iAgentPos',jAgentPos');
    if agentDist == 0
        beta = 1;
    else
        beta = 0.5*erfc((agentDist-461)/195)-0.085;
    end
end

% distance computation sub-fuction between two(i,j) agents
function dist = ComputeDistance(iAgentPos,jAgentPos)
    dist = ((iAgentPos(1)-jAgentPos(1))^2 + (iAgentPos(2)-jAgentPos(2))^2)^(1/2);
end