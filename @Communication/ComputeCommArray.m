function o = ComputeCommArray( o )
%COMPUTECOMMMATRIX Summary of this function goes here
%   Detailed explanation goes here

% generate status of communication (whether the agent succeeds to receive
% data from another agent) with Binomial random value
for iter = 1 : length(o.beta)
   o.C(iter) = binornd(1,o.beta(iter));
end

end

