function [xhatMgn, PhatMgn] = TakeMarginalEstimation( obj, commonTargetIdx, xhat, Phat )
%TAKEMARGINALESTIMATION finds marginal(common) estimation data (in terms of estimation and its
%coveraiance matrix) between two agents for fusion process

% make new common Phat for fusion process (with respect to agent j)
PhatMgn = [];
xhatMgn = [];

for iTarget = 1 : length(commonTargetIdx(:,1)) % with respect to raw
    PhatMgn_ji = [];
    for jTarget = 1 : length(commonTargetIdx(:,1)) % with respect to col
        PhatMgn_ji = [PhatMgn_ji,...
            Phat(commonTargetIdx(iTarget,1):commonTargetIdx(iTarget,2),commonTargetIdx(jTarget,1):commonTargetIdx(jTarget,2))];
    end
    PhatMgn = [PhatMgn;PhatMgn_ji];
    xhatMgn = [xhatMgn;xhat(commonTargetIdx(iTarget,1):commonTargetIdx(iTarget,2))];
end




end

