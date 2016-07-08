function prob = ComputeProbDetect( o, agent, point )
%COMPUTEPROBDETECT Summary of this function goes here
%   Detailed explanation goes here
  
prob = 1/(1+exp(o.beta*(TakeDistance(agent, point)-o.gamma)));

% prob = o.beta/(TakeDistance(agent, point) + o.beta^(1/o.gamma))^o.gamma;

end

function dist = TakeDistance(a,b)
    dist = sqrt((a(1)-b(1))^2+(a(2)-b(2))^2);
end
