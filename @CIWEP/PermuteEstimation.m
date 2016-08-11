function [ xhat_new, Phat_new ] = PermuteEstimation( obj,commonStateIdx,exclusiveStateIdx, xhat, Phat )
%PERMUTEESTIMATION permutes to group exclusive states and common states for fusion
%process
%   Output Matrix Form : [ A C^T; C B ]
%   A : exclusive states (bias, exclusive target estimation)
%   B : common states (common target estimation)
%   C : correlation between A and B
    
% make new common Phat for fusion process (with respect to agent j)
Phat_new = [];
xhat_new = [];


permutedIdx = [exclusiveStateIdx;commonStateIdx]; % re-arrange indices to make group

for iTarget = 1 : length(permutedIdx) % with respect to raw
    Phat_row = [];
    for jTarget = 1 : length(permutedIdx) % with respect to col
        Phat_row = [Phat_row,...
            Phat(permutedIdx(iTarget,1):permutedIdx(iTarget,2),permutedIdx(jTarget,1):permutedIdx(jTarget,2))];
    end
    Phat_new = [Phat_new;Phat_row];
    xhat_new = [xhat_new;xhat(permutedIdx(iTarget,1):permutedIdx(iTarget,2))];
end



end

