function prob = ComputeProbComm( o, agent1, agent2 )
%COMPUTEPROBCOMM Summary of this function goes here
%   Detailed explanation goes here
  
    prob = 1/(1+exp(o.alpha*(TakeDistance(agent1, agent2)-o.eta)));

% prob = o.alpha/(TakeDistance(agent1, agent2) + o.alpha^(1/o.eta))^o.eta;

end

function dist = TakeDistance(a,b)
    dist = sqrt((a(1)-b(1))^2+(a(2)-b(2))^2);
end

